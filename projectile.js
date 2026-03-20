/* ============================================================
   projectile.js — Projectile Motion Simulation v3

   Supports ALL scenarios:
     1. Ground → Ground   (h0 = 0, hLand = 0)
     2. Platform → Ground (h0 > 0, hLand = 0)
     3. Platform → Platform (h0 > 0, hLand > 0, projectile reaches it)
     4. Ground → Platform  (h0 = 0, hLand > 0)

   Physics (unchanged):
     vx = v0 · cos(θ)
     vy = v0 · sin(θ)
     y(t) = h0 + vy·t − ½·g·t²
     x(t) = vx · t

   Landing detection (detectLanding):
     For EACH surface: solve  surface_h = h0 + vy·t − ½g·t²
     Quadratic: ½g·t² − vy·t + (surface_h − h0) = 0
     Pick only positive-t roots.
     Check if the x position at that t is within the surface's
     horizontal extent.
     Choose the smallest valid t.

   Visual layout: ALL platforms are FIXED pixel positions.
   Physics heights are labels only — they do not move anything.
   ============================================================ */

'use strict';

/* ============================================================
   ANIMATION STATE
   ============================================================ */
var proj = {
  x: 0, y: 0,           /* physics position (m)            */
  vx: 0, vy: 0,         /* physics velocity (m/s)          */
  trail: [],             /* [{x,y,age}] trail with age 0-1  */
  done: false,
  data: [],              /* CSV rows                        */
  landedOn: null,        /* 'ground' | 'platform' | null    */

  /* Impact flash — set on landing, counts down each frame */
  _impactFlash: 0,       /* 0 = none, 1.0 = just landed     */
  _impactX: 0,           /* canvas px of impact point       */
  _impactY: 0,

  /* Solver / animation params — written by updateProjectile */
  _v0: 30, _theta: 45, _g: 9.81, _m: 1,
  _h0: 0,                /* launch height (m)               */
  _hLand: 0,             /* landing platform height (m)     */
  _landPlatEnabled: false
};

/* ============================================================
   FIXED VISUAL LAYOUT
   These values define pixel positions. They NEVER change
   regardless of physics heights.
   ============================================================ */
var PROJ_LAYOUT = {
  margin:       60,    /* px — left/right canvas margin          */
  groundFrac:   0.82,  /* fraction of H for the ground line      */
  ballR:        11,    /* px — projectile radius                  */

  /* Launch platform (always left side) */
  launchPlatW:  100,   /* px width                                */
  launchPlatH:  12,    /* px thickness                            */
  launchPlatX:  60,    /* px — left edge (= margin)               */

  /* Landing platform (always right-ish, fixed) */
  landPlatW:    100,   /* px width                                */
  landPlatH:    12,    /* px thickness                            */
  landPlatXFrac: 0.62  /* fraction of (W-2*margin) from origin   */
};

/* ============================================================
   PHYSICS HELPERS
   ============================================================ */

/**
 * solveQuadraticPositiveRoots
 * Solve  a·t² + b·t + c = 0  and return all positive real roots,
 * sorted ascending.
 */
function solveQuadraticPositiveRoots(a, b, c) {
  if (Math.abs(a) < 1e-12) {
    /* Linear: b·t + c = 0  →  t = -c/b */
    if (Math.abs(b) < 1e-12) return [];
    var t = -c / b;
    return t > 1e-6 ? [t] : [];
  }
  var disc = b * b - 4 * a * c;
  if (disc < 0) return [];
  var sq = Math.sqrt(disc);
  var roots = [];
  var t1 = (-b - sq) / (2 * a);
  var t2 = (-b + sq) / (2 * a);
  if (t1 > 1e-6) roots.push(t1);
  if (t2 > 1e-6 && Math.abs(t2 - t1) > 1e-9) roots.push(t2);
  roots.sort(function(a, b) { return a - b; });
  return roots;
}

/**
 * solveLandingTime
 * Given launch conditions and a target surface height,
 * return the earliest positive t at which y(t) = surfaceH.
 *
 * y(t) = h0 + vy·t − ½g·t² = surfaceH
 * → ½g·t² − vy·t + (surfaceH − h0) = 0
 * a = ½g,  b = −vy,  c = surfaceH − h0
 */
/**
 * solveLandingTime
 * Returns the time at which the projectile DESCENDS to surfaceH,
 * i.e. the root where vy(t) = vy - g*t <= 0 (ball is falling).
 *
 * For a platform below the apex we want the LATER root (descending).
 * For a surface the ball can only reach by falling (like the ground),
 * the later root is always correct.
 * For a surface the ball never reaches above launch height, the
 * earlier root IS the only valid one.
 */
function solveLandingTime(vy, g, h0, surfaceH) {
  var a =  0.5 * g;
  var b = -vy;
  var c =  surfaceH - h0;
  var roots = solveQuadraticPositiveRoots(a, b, c);
  if (roots.length === 0) return null;

  /* If there are two roots, check each one.
     We want the root where the ball is DESCENDING (vy_at_t <= 0)
     OR the only valid one if only one exists.
     vy_at_t = vy - g*t  — negative means descending. */
  if (roots.length === 1) return roots[0];

  /* Two roots: prefer the one where ball is falling */
  var vy_t0 = vy - g * roots[0];
  var vy_t1 = vy - g * roots[1];

  /* Landing means arriving at the surface while moving downward (or flat) */
  if (vy_t1 <= 0) return roots[1]; /* descending — correct landing */
  if (vy_t0 <= 0) return roots[0]; /* first root is descending    */

  /* Both ascending? Return the later (larger) one */
  return roots[1];
}

/**
 * detectLanding
 * Determines which surface the projectile lands on and when.
 *
 * Considers:
 *   1. Ground (y = 0)
 *   2. Landing platform (y = hLand) — only if enabled
 *      and x at landing time is within the platform's horizontal range
 *
 * Returns: { t, surface: 'ground'|'platform', x, y }
 * or null if no valid landing found.
 */
function detectLanding(vx, vy, g, h0, hLand, landPlatEnabled, landPlatXmin, landPlatXmax) {
  var best = null;

  /* --- Check ground (y = 0) --- */
  /* Only relevant if projectile can fall to 0 (always true for h0 >= 0) */
  var tGround = solveLandingTime(vy, g, h0, 0);
  if (tGround !== null && tGround > 0.01) {
    var xGround = vx * tGround;
    best = { t: tGround, surface: 'ground', x: xGround, y: 0 };
  }

  /* --- Check landing platform --- */
  if (landPlatEnabled && hLand > 0) {
    /* Only relevant if platform is higher than launch OR ball can reach it */
    var tPlat = solveLandingTime(vy, g, h0, hLand);
    if (tPlat !== null && tPlat > 0.01) {
      var xPlat = vx * tPlat;
      /* Check if x falls within the platform's horizontal range */
      if (xPlat >= landPlatXmin && xPlat <= landPlatXmax) {
        /* Pick the earlier landing */
        if (best === null || tPlat < best.t) {
          best = { t: tPlat, surface: 'platform', x: xPlat, y: hLand };
        }
      }
    }
  }

  return best;
}

/* ============================================================
   SOLVER — shared with existing runSolver engine
   ============================================================ */

function solveFlightTime(vy, g, h0, hLand) {
  /* Default hLand to 0 (ground) */
  hLand = hLand || 0;
  return solveLandingTime(vy, g, h0, hLand) || 0;
}

/* Equations registry — all math unchanged */
var PROJ_EQUATIONS = [

  { name: 'Horizontal velocity', formula: 'vx = v0 × cos(θ)',
    solve: {
      vx: function(s) {
        if (s.v0 === null || s.theta === null) return null;
        return s.v0 * Math.cos(s.theta * Math.PI / 180);
      },
      v0: function(s) {
        if (s.vx === null || s.theta === null) return null;
        var ct = Math.cos(s.theta * Math.PI / 180);
        return Math.abs(ct) < 1e-9 ? null : s.vx / ct;
      }
    },
    buildSub: function(s) {
      return 'vx = ' + fvN(s.v0) + ' × cos(' + fvN(s.theta, 1) + '°) = ' + fmtSci(s.vx) + ' m/s';
    }
  },

  { name: 'Vertical velocity', formula: 'vy = v0 × sin(θ)',
    solve: {
      vy: function(s) {
        if (s.v0 === null || s.theta === null) return null;
        return s.v0 * Math.sin(s.theta * Math.PI / 180);
      },
      v0: function(s) {
        if (s.vy === null || s.theta === null) return null;
        var st = Math.sin(s.theta * Math.PI / 180);
        return Math.abs(st) < 1e-9 ? null : s.vy / st;
      },
      theta: function(s) {
        if (s.vy === null || s.v0 === null || Math.abs(s.v0) < 1e-9) return null;
        var r = s.vy / s.v0;
        if (r < -1 || r > 1) return null;
        return Math.asin(r) * 180 / Math.PI;
      }
    },
    buildSub: function(s) {
      return 'vy = ' + fvN(s.v0) + ' × sin(' + fvN(s.theta, 1) + '°) = ' + fmtSci(s.vy) + ' m/s';
    }
  },

  { name: 'Launch speed', formula: 'v0 = √(vx² + vy²)',
    solve: {
      v0: function(s) {
        if (s.vx === null || s.vy === null) return null;
        return Math.sqrt(s.vx * s.vx + s.vy * s.vy);
      }
    },
    buildSub: function(s) {
      return 'v0 = √(' + fvN(s.vx) + '² + ' + fvN(s.vy) + '²) = ' + fmtSci(s.v0) + ' m/s';
    }
  },

  { name: 'Launch angle', formula: 'θ = atan2(vy, vx)',
    solve: {
      theta: function(s) {
        if (s.vx === null || s.vy === null) return null;
        return Math.atan2(s.vy, s.vx) * 180 / Math.PI;
      }
    },
    buildSub: function(s) {
      return 'θ = atan2(' + fvN(s.vy) + ', ' + fvN(s.vx) + ') = ' + fmtSci(s.theta) + '°';
    }
  },

  { name: 'Max height', formula: 'H = h0 + vy² / (2g)',
    solve: {
      H: function(s) {
        if (s.vy === null) return null;
        return (s.h0 || 0) + s.vy * s.vy / (2 * s.g);
      },
      vy: function(s) {
        if (s.H === null) return null;
        var disc = 2 * s.g * (s.H - (s.h0 || 0));
        return disc < 0 ? null : Math.sqrt(disc);
      },
      g: function(s) {
        if (s.H === null || s.vy === null) return null;
        var dh = s.H - (s.h0 || 0);
        return dh <= 0 ? null : s.vy * s.vy / (2 * dh);
      }
    },
    buildSub: function(s) {
      return 'H = ' + fvN(s.h0 || 0) + ' + ' + fvN(s.vy) + '² / (2×' + fvN(s.g) + ') = ' + fmtSci(s.H) + ' m';
    }
  },

  /* Flight time uses the quadratic landing solver */
  { name: 'Flight time', formula: 'T = [vy + √(vy² + 2g(h0−hLand))] / g',
    solve: {
      T: function(s) {
        if (s.vy === null) return null;
        return solveFlightTime(s.vy, s.g, s.h0 || 0, s.hLand || 0);
      }
    },
    buildSub: function(s) {
      var h0 = s.h0 || 0, hL = s.hLand || 0;
      return 'T = [' + fvN(s.vy) + ' + √(' + fvN(s.vy) + '² + 2×' + fvN(s.g) + '×(' + fvN(h0) + '−' + fvN(hL) + '))] / ' + fvN(s.g) + ' = ' + fmtSci(s.T) + ' s';
    }
  },

  { name: 'Range', formula: 'R = vx × T',
    solve: {
      R: function(s) {
        if (s.vx === null || s.T === null) return null;
        return s.vx * s.T;
      },
      vx: function(s) {
        if (s.R === null || s.T === null || Math.abs(s.T) < 1e-9) return null;
        return s.R / s.T;
      },
      T: function(s) {
        if (s.R === null || s.vx === null || Math.abs(s.vx) < 1e-9) return null;
        return s.R / s.vx;
      }
    },
    buildSub: function(s) {
      return 'R = ' + fvN(s.vx) + ' × ' + fvN(s.T) + ' = ' + fmtSci(s.R) + ' m';
    }
  },

  { name: 'Kinetic Energy', formula: 'KE = ½ × m × v0²',
    solve: {
      KE: function(s) {
        if (s.m === null || s.v0 === null) return null;
        return 0.5 * s.m * s.v0 * s.v0;
      },
      m: function(s) {
        if (s.KE === null || s.v0 === null || Math.abs(s.v0) < 1e-9) return null;
        return 2 * s.KE / (s.v0 * s.v0);
      },
      v0: function(s) {
        if (s.KE === null || s.m === null || s.m <= 0 || s.KE < 0) return null;
        return Math.sqrt(2 * s.KE / s.m);
      }
    },
    buildSub: function(s) {
      return 'KE = ½ × ' + fvN(s.m) + ' × ' + fvN(s.v0) + '² = ' + fmtSci(s.KE) + ' J';
    }
  },

  { name: 'Potential Energy', formula: 'PE = m × g × h0',
    solve: {
      PE: function(s) {
        if (s.m === null || !s.h0 || s.h0 <= 0) return null;
        return s.m * s.g * s.h0;
      },
      m: function(s) {
        if (s.PE === null || !s.h0 || Math.abs(s.h0 * s.g) < 1e-12) return null;
        return s.PE / (s.g * s.h0);
      }
    },
    buildSub: function(s) {
      return 'PE = ' + fvN(s.m) + ' × ' + fvN(s.g) + ' × ' + fvN(s.h0) + ' = ' + fmtSci(s.PE) + ' J';
    }
  },

  { name: 'Total Energy', formula: 'E = KE + PE',
    solve: {
      E:  function(s) { if (s.KE === null || s.PE === null) return null; return s.KE + s.PE; },
      KE: function(s) { if (s.E  === null || s.PE === null) return null; return s.E - s.PE; },
      PE: function(s) { if (s.E  === null || s.KE === null) return null; return s.E - s.KE; }
    },
    buildSub: function(s) {
      return 'E = ' + fmtSci(s.KE) + ' + ' + fmtSci(s.PE) + ' = ' + fmtSci(s.E) + ' J';
    }
  }
];

/* Result card metadata */
var PROJ_META = {
  R:     { label:'Range (R)',      unit:'m',    card:'prc-R',     val:'proj-r-R'     },
  H:     { label:'Max Height (H)', unit:'m',    card:'prc-H',     val:'proj-r-H'     },
  T:     { label:'Flight Time',    unit:'s',    card:'prc-T',     val:'proj-r-T'     },
  v0:    { label:'v0',             unit:'m/s',  card:'prc-v0',    val:'proj-r-v0'    },
  vx:    { label:'vx',             unit:'m/s',  card:'prc-vx',    val:'proj-r-vx'    },
  vy:    { label:'vy (initial)',   unit:'m/s',  card:'prc-vy',    val:'proj-r-vy'    },
  KE:    { label:'KE',             unit:'J',    card:'prc-KE',    val:'proj-r-KE'    },
  PE:    { label:'PE',             unit:'J',    card:'prc-PE',    val:'proj-r-PE'    },
  E:     { label:'Total E',        unit:'J',    card:'prc-E',     val:'proj-r-E'     },
  theta: { label:'Angle (θ)',      unit:'deg',  card:'prc-theta', val:'proj-r-theta' },
  g:     { label:'Gravity (g)',    unit:'m/s²', card:'prc-g',     val:'proj-r-g'     }
};

/* ============================================================
   RESET
   ============================================================ */
function resetProjectile() {
  proj = {
    x: 0, y: proj._h0, vx: 0, vy: 0,
    trail: [], done: false, data: [], landedOn: null,
    _impactFlash: 0, _impactX: 0, _impactY: 0,
    _v0: proj._v0, _theta: proj._theta, _g: proj._g,
    _m: proj._m, _h0: proj._h0,
    _hLand: proj._hLand, _landPlatEnabled: proj._landPlatEnabled
  };
  updateProjectile();
}

/* ============================================================
   UPDATE — solver + UI
   ============================================================ */
function updateProjectile() {
  /* Read all inputs */
  var h0    = getNullable('proj-h0')    !== null ? getNullable('proj-h0')    : 0;
  var hLand = getNullable('proj-hLand') !== null ? getNullable('proj-hLand') : 0;
  var landEn = document.getElementById('proj-landPlat') &&
               document.getElementById('proj-landPlat').checked;

  var s = {
    v0:    getNullable('proj-v0'),
    theta: getNullable('proj-angle'),
    R:     getNullable('proj-R'),
    H:     getNullable('proj-H'),
    T:     getNullable('proj-T'),
    m:     getNullable('proj-m'),
    h0:    h0,
    hLand: landEn ? hLand : 0,
    g:     getNullable('proj-g') !== null ? getNullable('proj-g') : 9.81,
    vx: null, vy: null, KE: null, PE: null, E: null
  };

  /* Validation */
  showInputError('proj-g',      validateInput(s.g,     { positive: true, label: 'g' }));
  showInputError('proj-v0',     validateInput(s.v0,    { min: 0, label: 'v0' }));
  showInputError('proj-angle',  validateInput(s.theta, { min: -90, max: 90, label: 'θ' }));
  showInputError('proj-h0',     validateInput(h0,      { min: 0, label: 'h0' }));
  showInputError('proj-hLand',  validateInput(hLand,   { min: 0, label: 'hLand' }));

  /* User keys */
  var userKeys = {};
  Object.keys(s).forEach(function(k) { if (s[k] !== null) userKeys[k] = true; });
  var solveFor = document.getElementById('proj-solveFor').value;

  /* Run solver */
  var result = runSolver(s, PROJ_EQUATIONS, 20);

  /* Update result cards */
  Object.keys(PROJ_META).forEach(function(k) {
    var m   = PROJ_META[k];
    var vel = document.getElementById(m.val);
    if (!vel) return;
    var val = s[k];
    vel.textContent = (val !== null && val !== undefined) ? fmtSci(val) + ' ' + m.unit : '--';
    var state = (k === solveFor) ? 'target'
              : (userKeys[k] ? 'user' : (val !== null ? 'derived' : ''));
    styleCard(m.card, state);
  });

  /* Solver steps */
  renderSolverSteps('proj-solver-steps', result.log, result.derived, s, solveFor,
    'Enter any known variables above.\nLeave unknowns blank — solver chains equations automatically.');

  /* Show / hide landing platform input row */
  var hLandRow = document.getElementById('proj-hLand-row');
  if (hLandRow) hLandRow.style.display = landEn ? '' : 'none';

  /* Mode badge update */
  var badge = document.getElementById('proj-mode-badge');
  if (badge) {
    if (h0 > 0 && landEn && hLand > 0) badge.textContent = 'Platform → Platform';
    else if (h0 > 0)                    badge.textContent = 'Cliff → Ground';
    else if (landEn && hLand > 0)       badge.textContent = 'Ground → Platform';
    else                                badge.textContent = 'Ground → Ground';
  }

  /* Store for animation */
  proj._v0              = (s.v0    !== null) ? s.v0    : 30;
  proj._theta           = (s.theta !== null) ? s.theta : 45;
  proj._g               = s.g;
  proj._m               = (s.m     !== null) ? s.m     : 1;
  proj._h0              = h0;
  proj._hLand           = landEn ? hLand : 0;
  proj._landPlatEnabled = landEn && hLand > 0;

  if (!simRunning) drawProjectile();
}

/* ============================================================
   STEP — sub-stepped Euler + landing detection
   ============================================================ */
function stepProjectile(dt) {
  /* Decay impact flash every frame regardless of done state */
  if (proj._impactFlash > 0) {
    proj._impactFlash = Math.max(0, proj._impactFlash - dt * 3);
  }

  if (proj.done) { stopSim(); return; }

  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var m     = proj._m;
  var h0    = proj._h0;

  /* First-frame initialisation */
  if (simTime <= dt + 0.001) {
    proj.x            = 0;
    proj.y            = h0;
    proj.vx           = v0 * Math.cos(theta);
    proj.vy           = v0 * Math.sin(theta);
    proj.trail        = [];
    proj.data         = [];
    proj.done         = false;
    proj.landedOn     = null;
    proj._impactFlash = 0;

    /* Pre-compute expected landing for ghost trajectory + marker */
    proj._landing = _computeLanding(proj.vx, proj.vy, g, h0);
  }

  /* Sub-stepped Euler — 8 steps for tight collision accuracy */
  var SUBSTEPS = 8;
  var dts = dt / SUBSTEPS;
  var landed = false;

  for (var si = 0; si < SUBSTEPS; si++) {
    proj.vy -= g * dts;
    proj.x  += proj.vx * dts;
    proj.y  += proj.vy * dts;

    /* Ground collision */
    if (proj.y <= 0 && simTime > 0.05) {
      proj.y            = 0;
      proj.vx           = 0;
      proj.vy           = 0;
      proj.done         = true;
      proj.landedOn     = 'ground';
      proj._impactFlash = 1.0;
      landed = true;
      break;
    }

    /* Landing platform collision — only when descending */
    if (proj._landPlatEnabled && proj.vy <= 0 && simTime > 0.05) {
      var hL = proj._hLand;
      if (proj.y <= hL) {
        var platRange = _landPlatPhysicsRange(proj.vx);
        if (proj.x >= platRange.xMin && proj.x <= platRange.xMax) {
          proj.y            = hL;
          proj.vx           = 0;
          proj.vy           = 0;
          proj.done         = true;
          proj.landedOn     = 'platform';
          proj._impactFlash = 1.0;
          landed = true;
          break;
        }
      }
    }
  }

  /* Compute impact flash screen position on landing frame */
  if (landed) {
    var _W = canvas.width, _H = canvas.height;
    var _L = PROJ_LAYOUT;
    var _gY = _H * _L.groundFrac;
    var _oPx = _L.launchPlatX + _L.launchPlatW / 2;
    var _v0l = proj._v0, _tl = proj._theta * Math.PI / 180;
    var _vx0 = _v0l * Math.cos(_tl), _vy0 = _v0l * Math.sin(_tl);
    var _tG = solveFlightTime(_vy0, g, h0, 0);
    var _R  = _vx0 * _tG, _aH = h0 + _vy0 * _vy0 / (2 * g);
    var _sX = _R  > 0 ? (_W - _L.margin * 2) / Math.max(_R, 1) : 8;
    var _sY = _aH > 0 ? (_gY - _L.margin) / Math.max(_aH, 1) * 0.88 : 8;
    var _sc = Math.min(_sX, _sY, 80); if (_sc <= 0) _sc = 8;
    proj._impactX = _oPx + proj.x * _sc;
    proj._impactY = _gY  - proj.y * _sc;
  }

  /* Trail — store position with an age counter for fade */
  var trailEl = document.getElementById('proj-trail');
  if (trailEl && trailEl.checked) {
    proj.trail.push({ x: proj.x, y: proj.y });
    /* Age existing points: oldest ones will be first in array */
    if (proj.trail.length > 800) proj.trail.shift();
  }

  /* CSV */
  var speed = Math.sqrt(proj.vx * proj.vx + proj.vy * proj.vy);
  proj.data.push([
    +simTime.toFixed(3), +proj.x.toFixed(3), +proj.y.toFixed(3),
    +proj.vx.toFixed(3), +proj.vy.toFixed(3), +speed.toFixed(3),
    +(0.5 * m * speed * speed).toFixed(3)
  ]);

  /* Live cards */
  var ke = 0.5 * m * speed * speed;
  var elv  = document.getElementById('proj-r-v');
  var elke = document.getElementById('proj-r-KE');
  var elxy = document.getElementById('proj-r-xy');
  if (elv)  elv.textContent  = fmtSci(speed)  + ' m/s';
  if (elke) elke.textContent = fmtSci(ke)      + ' J';
  if (elxy) elxy.textContent = fmtSci(proj.x)  + ', ' + fmtSci(proj.y);

  updateInfoBar(simTime, proj.x, proj.y, speed);
}

/* ============================================================
   INTERNAL HELPERS
   ============================================================ */

/**
 * _computeLanding — returns the landing event object for the
 * current proj params, used to show the landing marker at rest.
 */
function _computeLanding(vx, vy, g, h0) {
  /* Landing platform physics x-range */
  var platRange = _landPlatPhysicsRange(vx);

  return detectLanding(
    vx, vy, g, h0,
    proj._hLand,
    proj._landPlatEnabled,
    platRange.xMin,
    platRange.xMax
  );
}

/**
 * _landPlatPhysicsRange
 * Returns the physics x-range (metres) that corresponds to the
 * FIXED visual landing platform position.
 *
 * This is the KEY decoupling: we convert FIXED pixel positions
 * back to physics metres using the current scale, so physics
 * landing check matches what the user sees.
 *
 * @param {number} vx - horizontal velocity (used to derive scale)
 */
function _landPlatPhysicsRange(vx) {
  /* We use the drawing scale used in drawProjectile.
     Since scale depends on trajectory, we compute it here too. */
  var W = canvas.width, H = canvas.height;
  var L = PROJ_LAYOUT;
  var margin   = L.margin;
  var groundY  = H * L.groundFrac;
  var trackW   = W - margin * 2;
  var originPx = L.launchPlatX + L.launchPlatW / 2;

  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var h0    = proj._h0;
  var hL    = proj._hLand;
  var vx0   = v0 * Math.cos(theta);
  var vy0   = v0 * Math.sin(theta);

  var T      = solveFlightTime(vy0, g, h0, 0); /* time to ground */
  var R      = vx0 * T;
  var apexH  = h0 + vy0 * vy0 / (2 * g);

  var scaleX = R > 0     ? (W - margin * 2) / Math.max(R, 1) : 8;
  var scaleY = apexH > 0 ? (groundY - margin) / Math.max(apexH, 1) * 0.88 : 8;
  var scale  = Math.min(scaleX, scaleY, 80);
  if (scale <= 0) scale = 8;

  /* Fixed pixel position of landing platform → physics metres */
  var platPixLeft  = originPx + trackW * L.landPlatXFrac;
  var platPixRight = platPixLeft + L.landPlatW;

  return {
    xMin: (platPixLeft  - originPx) / scale,
    xMax: (platPixRight - originPx) / scale
  };
}

/* ============================================================
   DRAW HELPERS
   ============================================================ */

/**
 * drawPlatforms — draw ALL surface objects at their FIXED positions.
 * Positions come from PROJ_LAYOUT, NEVER from physics values.
 *
 * @param {number} groundY  - fixed canvas y for ground
 * @param {number} W, H     - canvas dimensions
 */
function drawPlatforms(ctx, groundY, W, H) {
  var L = PROJ_LAYOUT;

  /* --- Launch platform (left, fixed) --- */
  _drawPlatBar(ctx, L.launchPlatX, groundY, L.launchPlatW, L.launchPlatH, '#a8ff3e', proj._h0);

  /* --- Landing platform (right, fixed) — only when enabled --- */
  if (proj._landPlatEnabled) {
    var trackW  = W - L.margin * 2;
    var platX   = L.launchPlatX + L.launchPlatW / 2 + trackW * L.landPlatXFrac;
    _drawPlatBar(ctx, platX, groundY, L.landPlatW, L.landPlatH, '#ff6b35', proj._hLand);
  }
}

function _drawPlatBar(ctx, x, groundY, w, h, color, physH) {
  ctx.save();

  /* Drop shadow */
  ctx.shadowColor = color; ctx.shadowBlur = 14;

  /* Main platform bar — gradient top-to-bottom */
  var grad = ctx.createLinearGradient(x, groundY - h, x, groundY);
  grad.addColorStop(0, color + 'ee');
  grad.addColorStop(0.4, color + 'bb');
  grad.addColorStop(1, color + '44');
  ctx.fillStyle = grad;
  ctx.fillRect(x, groundY - h, w, h);
  ctx.shadowBlur = 0;

  /* Top edge bright line for 3-D ledge look */
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x, groundY - h);
  ctx.lineTo(x + w, groundY - h);
  ctx.stroke();

  /* Side and bottom border */
  ctx.strokeStyle = color + '66'; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x, groundY - h);
  ctx.lineTo(x, groundY);
  ctx.lineTo(x + w, groundY);
  ctx.lineTo(x + w, groundY - h);
  ctx.stroke();

  /* Diagonal hatch pattern on face */
  ctx.strokeStyle = color + '22'; ctx.lineWidth = 1;
  for (var hx = x; hx < x + w + h; hx += 8) {
    ctx.beginPath();
    ctx.moveTo(Math.max(x, hx - h), groundY - h);
    ctx.lineTo(Math.min(x + w, hx),  groundY);
    ctx.stroke();
  }

  /* Height label above platform */
  if (physH > 0) {
    ctx.font = 'bold 10px Space Mono, monospace';
    ctx.textAlign = 'center';
    var lbl = fmtSci(physH) + ' m';
    var tw  = ctx.measureText(lbl).width;
    /* Label pill */
    ctx.fillStyle = color + '22';
    ctx.strokeStyle = color + '66'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(x + w/2 - tw/2 - 5, groundY - h - 20, tw + 10, 16, 3);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = color;
    ctx.fillText(lbl, x + w / 2, groundY - h - 8);
    ctx.textAlign = 'left';
  }
  ctx.restore();
}

/**
 * drawHeightIndicator — dashed vertical line from launch platform
 * top to ground, labelled with h0. Fixed pixel position.
 */
function drawHeightIndicator(ctx, platX, platW, groundY, physH) {
  if (physH <= 0) return;
  var midX = platX + platW * 0.75;
  var topY = groundY - PROJ_LAYOUT.launchPlatH;

  ctx.save();
  ctx.strokeStyle = 'rgba(168,255,62,0.4)';
  ctx.lineWidth   = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(midX, groundY - 4);
  ctx.lineTo(midX, topY);
  ctx.stroke();
  ctx.setLineDash([]);

  /* Arrow tips */
  function arrowTip(ax, ay, up) {
    ctx.fillStyle = 'rgba(168,255,62,0.6)';
    ctx.beginPath();
    if (up) {
      ctx.moveTo(ax, ay); ctx.lineTo(ax - 4, ay + 7); ctx.lineTo(ax + 4, ay + 7);
    } else {
      ctx.moveTo(ax, ay); ctx.lineTo(ax - 4, ay - 7); ctx.lineTo(ax + 4, ay - 7);
    }
    ctx.closePath(); ctx.fill();
  }
  arrowTip(midX, topY, true);
  arrowTip(midX, groundY - 4, false);

  /* Label */
  var lx = midX + 8, ly = (groundY + topY) / 2;
  var txt = 'h0 = ' + fmtSci(physH) + ' m';
  ctx.font = 'bold 10px Space Mono, monospace';
  var tw = ctx.measureText(txt).width;
  ctx.fillStyle = 'rgba(168,255,62,0.15)';
  ctx.strokeStyle = 'rgba(168,255,62,0.4)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(lx - 3, ly - 10, tw + 8, 16, 3);
  ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#a8ff3e';
  ctx.fillText(txt, lx + 1, ly + 3);

  ctx.restore();
}

/* ============================================================
   DRAW HELPERS — impact ring, shadow, faded trail
   ============================================================ */

/** Draw a fading impact ring at landing point */
function _drawImpactFlash(ctx, px, py, flash, color) {
  if (flash <= 0) return;
  ctx.save();
  /* Expanding ring */
  var radius = (1 - flash) * 40 + 8;
  var alpha  = flash * 0.8;
  ctx.strokeStyle = color;
  ctx.globalAlpha = alpha;
  ctx.lineWidth   = 2.5;
  ctx.shadowColor = color; ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.arc(px, py, radius, 0, Math.PI * 2);
  ctx.stroke();
  /* Inner dot */
  ctx.globalAlpha = alpha * 0.5;
  ctx.fillStyle   = color;
  ctx.beginPath();
  ctx.arc(px, py, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.restore();
}

/** Draw a subtle shadow ellipse under the ball while in flight */
function _drawBallShadow(ctx, bxPx, groundY, ballR, heightFrac) {
  /* Shadow shrinks and fades as ball rises */
  var alpha  = 0.18 * (1 - heightFrac * 0.7);
  var scaleW = 1 + heightFrac * 0.4;
  if (alpha <= 0) return;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle   = 'rgba(0,0,0,0.6)';
  ctx.beginPath();
  ctx.ellipse(bxPx, groundY - 2, ballR * scaleW, 4, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.restore();
}

/** Draw a fading trail — newer points brighter, older ones faded */
function _drawTrail(ctx, trail, wx, wy) {
  if (trail.length < 2) return;
  ctx.save();
  /* Draw segments with varying opacity based on age */
  for (var ti = 1; ti < trail.length; ti++) {
    var age   = ti / trail.length;       /* 0 = oldest, 1 = newest */
    var alpha = age * age * 0.75;        /* quadratic fade */
    var width = 1 + age * 2;            /* thinner at tail */
    if (alpha < 0.02) continue;
    var p0 = trail[ti - 1], p1 = trail[ti];
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth   = width;
    ctx.beginPath();
    ctx.moveTo(wx(p0.x), wy(p0.y));
    ctx.lineTo(wx(p1.x), wy(p1.y));
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
  ctx.restore();
}

/** Draw a glowing projectile ball with motion blur suggestion */
function _drawBall(ctx, bxPx, byPx, ballR, vx, vy, speed) {
  ctx.save();

  /* Outer glow — size scales with speed */
  var glowR = ballR + Math.min(speed * 0.15, 10);
  ctx.shadowColor = '#00d4ff';
  ctx.shadowBlur  = 18 + Math.min(speed * 0.1, 12);

  /* Gradient fill */
  var grad = ctx.createRadialGradient(
    bxPx - ballR * 0.3, byPx - ballR * 0.3, 0,
    bxPx, byPx, glowR
  );
  grad.addColorStop(0,   '#ffffff');
  grad.addColorStop(0.2, '#a0efff');
  grad.addColorStop(0.6, '#00d4ff');
  grad.addColorStop(1,   '#00d4ff00');
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(bxPx, byPx, glowR, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;

  /* Core solid ball */
  ctx.fillStyle = '#00d4ff';
  ctx.beginPath();
  ctx.arc(bxPx, byPx, ballR, 0, Math.PI * 2);
  ctx.fill();

  /* Specular highlight */
  ctx.fillStyle = 'rgba(255,255,255,0.6)';
  ctx.beginPath();
  ctx.arc(bxPx - ballR * 0.3, byPx - ballR * 0.3, ballR * 0.35, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

/* ============================================================
   MAIN DRAW
   ============================================================ */
function drawProjectile() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12';
  ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var L      = PROJ_LAYOUT;
  var margin = L.margin;

  /* ---- Fixed anchor points ---- */
  var groundY  = H * L.groundFrac;
  var originPx = L.launchPlatX + L.launchPlatW / 2; /* launch x in px */

  /* ---- Physics params ---- */
  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var h0    = proj._h0 || 0;
  var hLand = proj._hLand || 0;
  var vx0   = v0 * Math.cos(theta);
  var vy0   = v0 * Math.sin(theta);

  /* ---- Scale — derived from full trajectory to ground ---- */
  var T_toGround = solveFlightTime(vy0, g, h0, 0);
  var R_toGround = vx0 * T_toGround;
  var apexH      = h0 + vy0 * vy0 / (2 * g);
  var trackW     = W - margin * 2;
  var usableH    = groundY - margin;

  var scaleX = R_toGround > 0 ? trackW / Math.max(R_toGround, 1) : 8;
  var scaleY = apexH      > 0 ? usableH / Math.max(apexH, 1) * 0.88 : 8;
  var scale  = Math.min(scaleX, scaleY, 80);
  if (scale <= 0) scale = 8;

  /* World → canvas (origin at launch point on ground level) */
  function wx(x) { return originPx + x * scale; }
  function wy(y) { return groundY  - y * scale; }

  /* ---- Ground line — textured ---- */
  /* Ground fill */
  var gfGrad = ctx.createLinearGradient(0, groundY, 0, groundY + 28);
  gfGrad.addColorStop(0,   'rgba(0,212,255,0.12)');
  gfGrad.addColorStop(0.4, 'rgba(0,212,255,0.04)');
  gfGrad.addColorStop(1,   'rgba(0,0,0,0)');
  ctx.fillStyle = gfGrad;
  ctx.fillRect(margin, groundY, trackW, 28);

  /* Ground line with glow */
  ctx.save();
  ctx.shadowColor = '#00d4ff'; ctx.shadowBlur = 6;
  ctx.strokeStyle = 'rgba(0,212,255,0.5)';
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(margin, groundY);
  ctx.lineTo(W - margin, groundY);
  ctx.stroke();
  ctx.shadowBlur = 0;
  ctx.restore();

  /* Subtle tick marks on ground */
  ctx.strokeStyle = 'rgba(0,212,255,0.15)'; ctx.lineWidth = 1;
  for (var tx = margin; tx <= W - margin; tx += 40) {
    ctx.beginPath(); ctx.moveTo(tx, groundY); ctx.lineTo(tx, groundY + 5); ctx.stroke();
  }

  /* ---- Platforms (FIXED positions) ---- */
  drawPlatforms(ctx, groundY, W, H);

  /* ---- Height indicator ---- */
  drawHeightIndicator(ctx, L.launchPlatX, L.launchPlatW, groundY, h0);

  /* ---- Ghost trajectory ---- */
  /* Compute actual landing for trajectory end */
  var platRange = _landPlatPhysicsRange(vx0);
  var landing   = detectLanding(vx0, vy0, g, h0, hLand,
                                proj._landPlatEnabled, platRange.xMin, platRange.xMax);
  var T_end     = landing ? landing.t : T_toGround;
  var endY      = landing ? landing.y : 0;

  /* ---- Ghost trajectory — dashed arc ---- */
  ctx.save();
  ctx.strokeStyle = 'rgba(0,212,255,0.18)';
  ctx.lineWidth   = 1.5;
  ctx.setLineDash([7, 6]);
  ctx.beginPath();
  var GHOST_STEPS = 160;
  var ghostFirst  = true;
  for (var gi = 0; gi <= GHOST_STEPS; gi++) {
    var gt  = T_end * gi / GHOST_STEPS;
    var ggx = vx0 * gt;
    var ggy = h0 + vy0 * gt - 0.5 * g * gt * gt;
    if (ggy < endY - 0.001) break;
    if (ghostFirst) { ctx.moveTo(wx(ggx), wy(ggy)); ghostFirst = false; }
    else             ctx.lineTo(wx(ggx), wy(ggy));
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();

  /* ---- Fading trail ---- */
  _drawTrail(ctx, proj.trail, wx, wy);

  /* ---- Landing marker (predicted) ---- */
  if (landing) {
    var lmx = wx(landing.x), lmy = wy(landing.y);
    var lCol = (landing.surface === 'platform') ? '#ff6b35' : '#00d4ff';
    ctx.save();
    /* Crosshair */
    ctx.strokeStyle = lCol + '88'; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(lmx, lmy - 12); ctx.lineTo(lmx, lmy + 12); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(lmx - 12, lmy); ctx.lineTo(lmx + 12, lmy); ctx.stroke();
    ctx.setLineDash([]);
    /* Circle */
    ctx.strokeStyle = lCol + 'aa'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(lmx, lmy, 8, 0, Math.PI * 2); ctx.stroke();
    /* Label */
    ctx.fillStyle = lCol; ctx.font = '10px Space Mono'; ctx.textAlign = 'center';
    ctx.fillText('x=' + fmtSci(landing.x) + 'm', lmx, lmy - 16);
    ctx.textAlign = 'left';
    ctx.restore();
  }

  /* ---- Ball position ---- */
  var bxPx, byPx;
  var platTopY = groundY - L.launchPlatH;
  var speed    = Math.sqrt(proj.vx * proj.vx + proj.vy * proj.vy);

  if (!simRunning && simTime === 0) {
    /* Idle: rest on launch platform */
    bxPx = originPx;
    byPx = platTopY - L.ballR;
    speed = 0;
  } else {
    bxPx = wx(proj.x);
    byPx = wy(proj.y);
    /* Never go visually below the landing surface */
    var landSurfY = proj.landedOn === 'platform' ? wy(proj._hLand) : groundY;
    if (byPx > landSurfY - L.ballR) byPx = landSurfY - L.ballR;
  }

  /* Shadow on ground (only when airborne) */
  if (simRunning && !proj.done) {
    var heightAboveGround = proj.y;                 /* metres above ground */
    var maxH = Math.max(apexH, 1);
    var hFrac = Math.min(heightAboveGround / maxH, 1);
    _drawBallShadow(ctx, bxPx, groundY, L.ballR, hFrac);
  }

  /* Polished ball */
  _drawBall(ctx, bxPx, byPx, L.ballR, proj.vx, proj.vy, speed);

  /* Impact flash */
  if (proj._impactFlash > 0) {
    var flashCol = proj.landedOn === 'platform' ? '#ff6b35' : '#00d4ff';
    _drawImpactFlash(ctx, proj._impactX, proj._impactY, proj._impactFlash, flashCol);
  }

  /* ---- Velocity vectors ---- */
  var vecEl = document.getElementById('proj-vectors');
  if (vecEl && vecEl.checked && simRunning && !proj.done) {
    var vs = Math.max(scale * 0.4, 2);
    if (Math.abs(proj.vx) > 0.05)
      drawArrow(ctx, bxPx, byPx, bxPx + proj.vx * vs, byPx, '#a8ff3e', 'vx', 2);
    if (Math.abs(proj.vy) > 0.05)
      drawArrow(ctx, bxPx, byPx, bxPx, byPx - proj.vy * vs, '#ff6b35', 'vy', 2);
  }

  /* ---- Apex marker ---- */
  var T_up  = vy0 / g;
  var apexX = vx0 * T_up;
  if (T_up > 0 && apexH > h0 + 0.01 && R_toGround > 0) {
    var apx = wx(apexX), apy = wy(apexH);
    ctx.strokeStyle = 'rgba(168,255,62,0.3)';
    ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(apx, apy); ctx.lineTo(apx, groundY); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono'; ctx.textAlign = 'center';
    ctx.fillText('H=' + fmtSci(apexH) + 'm', apx, apy - 12);
    ctx.textAlign = 'left';
  }

  /* ---- Range label ---- */
  if (landing && landing.x > 0.01) {
    ctx.fillStyle = 'rgba(0,212,255,0.75)';
    ctx.font = '11px Space Mono'; ctx.textAlign = 'center';
    ctx.fillText('R = ' + fmtSci(landing.x) + ' m', wx(landing.x / 2), groundY + 18);
    ctx.textAlign = 'left';
  }

  /* ---- Mode badge ---- */
  var badge = document.getElementById('proj-mode-badge');
  var modeText = (h0 > 0 && proj._landPlatEnabled && hLand > 0) ? 'PLATFORM → PLATFORM'
    : h0 > 0                                                     ? 'CLIFF → GROUND'
    : (proj._landPlatEnabled && hLand > 0)                       ? 'GROUND → PLATFORM'
    :                                                               'GROUND → GROUND';
  if (!badge) {
    /* Draw mode text on canvas if badge element missing */
    ctx.fillStyle = 'rgba(107,122,153,0.6)';
    ctx.font = '10px Space Mono';
    ctx.fillText(modeText, margin + 4, groundY - L.launchPlatH - 18);
  }

  /* ---- Landed indicator banner ---- */
  if (proj.done && proj.landedOn) {
    var banCol  = proj.landedOn === 'platform' ? '#ff6b35' : '#00d4ff';
    var banText = proj.landedOn === 'platform'
      ? 'Landed on platform  •  x = ' + fmtSci(proj.x) + ' m'
      : 'Landed on ground  •  x = ' + fmtSci(proj.x) + ' m';
    ctx.save();
    ctx.font = 'bold 12px DM Sans, sans-serif';
    var btw = ctx.measureText(banText).width;
    var bx2 = W / 2 - btw / 2 - 14, by2 = groundY - 46;
    /* Pill background */
    ctx.fillStyle   = banCol + '22';
    ctx.strokeStyle = banCol + '88'; ctx.lineWidth = 1.5;
    ctx.shadowColor = banCol; ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.roundRect(bx2, by2, btw + 28, 26, 6);
    ctx.fill(); ctx.stroke();
    ctx.shadowBlur = 0;
    /* Text */
    ctx.fillStyle = banCol;
    ctx.textAlign = 'center';
    ctx.fillText(banText, W / 2, by2 + 17);
    ctx.textAlign = 'left';
    ctx.restore();
  }
}

/* ============================================================
   CSV EXPORT
   ============================================================ */
function exportProjectileCSV() {
  if (!proj.data || proj.data.length === 0) {
    alert('Run the simulation first to generate data.'); return;
  }
  exportCSV('projectile_data.csv',
    ['time_s','x_m','y_m','vx_ms','vy_ms','speed_ms','KE_J'], proj.data);
}
