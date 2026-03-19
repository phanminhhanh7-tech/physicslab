/* ============================================================
   projectile.js — Projectile Motion Simulation v2
   
   Physics model:
     vx = v0 · cos(θ)
     vy = v0 · sin(θ)
     x(t)  = vx · t
     y(t)  = h0 + vy·t − ½·g·t²
   
   Flight time (cliff mode h0 > 0) — quadratic formula:
     0 = h0 + vy·T − ½·g·T²
     ½g·T² − vy·T − h0 = 0
     T = [vy + √(vy² + 2·g·h0)] / g   (positive root)
   
   Flight time (ground mode h0 = 0):
     T = 2·vy / g
   
   Max height above ground:
     H = h0 + vy² / (2g)
   
   Platform visual: FIXED pixel position — NEVER moves.
   Height h0 is shown as a label + dashed vertical indicator.
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   ANIMATION STATE
   ------------------------------------------------------------ */
var proj = {
  x: 0, y: 0,        /* current physics position (m)         */
  vx: 0, vy: 0,      /* current velocity (m/s)               */
  trail: [],          /* [{x,y}] physics positions for trail  */
  done: false,        /* true once ball has landed            */
  data: [],           /* rows for CSV export                  */

  /* Launch parameters (set by solver, used by step + draw): */
  _v0: 30, _theta: 45, _g: 9.81, _m: 1, _h0: 0
};

/* ------------------------------------------------------------
   FIXED LAYOUT CONSTANTS
   These pixel values define the platform and ground positions
   and NEVER change regardless of h0.
   ------------------------------------------------------------ */
var PROJ_LAYOUT = {
  margin:      60,    /* px — left/right margin                */
  groundFrac:  0.82,  /* fraction of canvas height for ground  */
  platW:       90,    /* px — platform width                   */
  platH:       12,    /* px — platform height (bar)            */
  platYFrac:   0.82,  /* same as ground — platform sits ON ground */
  ballR:       11     /* px — projectile radius                */
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetProjectile() {
  proj = {
    x: 0, y: proj._h0, vx: 0, vy: 0,
    trail: [], done: false, data: [],
    _v0: proj._v0, _theta: proj._theta, _g: proj._g,
    _m: proj._m, _h0: proj._h0
  };
  updateProjectile();
}

/* ------------------------------------------------------------
   SOLVER HELPERS

   solveFlightTime — returns the positive flight time T using
   the quadratic formula, valid for both h0=0 and h0>0 (cliff).

   Equation: h0 + vy·T − ½g·T² = 0
   Rearranged: ½g·T² − vy·T − h0 = 0
   Quadratic: a=½g, b=−vy, c=−h0
   T = [vy + √(vy² + 2g·h0)] / g   (positive root)

   When h0 = 0 this reduces to T = 2vy/g (standard formula).
   ------------------------------------------------------------ */
function solveFlightTime(vy, g, h0) {
  if (g <= 0) return 0;
  h0 = h0 || 0;
  /* discriminant = vy² + 2·g·h0 */
  var disc = vy * vy + 2 * g * h0;
  if (disc < 0) return 0; /* no real solution — shouldn't happen for valid inputs */
  return (vy + Math.sqrt(disc)) / g;
}

/* ------------------------------------------------------------
   EQUATIONS REGISTRY
   All physics equations — unchanged from original.
   The solver propagates known → unknown using these rules.
   ------------------------------------------------------------ */
var PROJ_EQUATIONS = [

  /* vx = v0 · cos(θ) */
  { name: 'Horizontal velocity', formula: 'vx = v0 × cos(θ)',
    solve: {
      vx: function(s) {
        if (s.v0 === null || s.theta === null) return null;
        return s.v0 * Math.cos(s.theta * Math.PI / 180);
      },
      v0: function(s) {
        if (s.vx === null || s.theta === null) return null;
        var ct = Math.cos(s.theta * Math.PI / 180);
        if (Math.abs(ct) < 1e-9) return null;
        return s.vx / ct;
      }
    },
    buildSub: function(s) {
      return 'vx = ' + fvN(s.v0) + ' × cos(' + fvN(s.theta, 1) + '°) = ' + fmtSci(s.vx) + ' m/s';
    }
  },

  /* vy = v0 · sin(θ) */
  { name: 'Vertical velocity', formula: 'vy = v0 × sin(θ)',
    solve: {
      vy: function(s) {
        if (s.v0 === null || s.theta === null) return null;
        return s.v0 * Math.sin(s.theta * Math.PI / 180);
      },
      v0: function(s) {
        if (s.vy === null || s.theta === null) return null;
        var st = Math.sin(s.theta * Math.PI / 180);
        if (Math.abs(st) < 1e-9) return null;
        return s.vy / st;
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

  /* v0 from components */
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

  /* θ from components */
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

  /* H = h0 + vy²/(2g) — max height above ground */
  { name: 'Max height', formula: 'H = h0 + vy² / (2g)',
    solve: {
      H: function(s) {
        if (s.vy === null) return null;
        return (s.h0 || 0) + (s.vy * s.vy) / (2 * s.g);
      },
      vy: function(s) {
        if (s.H === null) return null;
        var disc = 2 * s.g * (s.H - (s.h0 || 0));
        if (disc < 0) return null;
        return Math.sqrt(disc);
      },
      g: function(s) {
        if (s.H === null || s.vy === null) return null;
        var dh = s.H - (s.h0 || 0);
        if (dh <= 0) return null;
        return (s.vy * s.vy) / (2 * dh);
      }
    },
    buildSub: function(s) {
      return 'H = ' + fvN(s.h0 || 0) + ' + ' + fvN(s.vy) + '² / (2×' + fvN(s.g) + ') = ' + fmtSci(s.H) + ' m';
    }
  },

  /* T — flight time using quadratic (works for both h0=0 and h0>0) */
  { name: 'Flight time', formula: 'T = [vy + √(vy² + 2g·h0)] / g',
    solve: {
      T: function(s) {
        if (s.vy === null) return null;
        return solveFlightTime(s.vy, s.g, s.h0 || 0);
      }
    },
    buildSub: function(s) {
      var h0 = s.h0 || 0;
      return 'T = [' + fvN(s.vy) + ' + √(' + fvN(s.vy) + '² + 2×' + fvN(s.g) + '×' + fvN(h0) + ')] / ' + fvN(s.g) + ' = ' + fmtSci(s.T) + ' s';
    }
  },

  /* R = vx · T */
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

  /* KE = ½mv0² */
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

  /* PE = m·g·h0 */
  { name: 'Potential Energy', formula: 'PE = m × g × h0',
    solve: {
      PE: function(s) {
        if (s.m === null || s.h0 === null || s.h0 <= 0) return null;
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

  /* E = KE + PE */
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

/* Metadata for result cards */
var PROJ_META = {
  R:     { label:'Range (R)',      unit:'m',   card:'prc-R',     val:'proj-r-R'     },
  H:     { label:'Max Height (H)', unit:'m',   card:'prc-H',     val:'proj-r-H'     },
  T:     { label:'Flight Time',    unit:'s',   card:'prc-T',     val:'proj-r-T'     },
  v0:    { label:'v0',             unit:'m/s', card:'prc-v0',    val:'proj-r-v0'    },
  vx:    { label:'vx',             unit:'m/s', card:'prc-vx',    val:'proj-r-vx'    },
  vy:    { label:'vy (initial)',   unit:'m/s', card:'prc-vy',    val:'proj-r-vy'    },
  KE:    { label:'KE',             unit:'J',   card:'prc-KE',    val:'proj-r-KE'    },
  PE:    { label:'PE',             unit:'J',   card:'prc-PE',    val:'proj-r-PE'    },
  E:     { label:'Total E',        unit:'J',   card:'prc-E',     val:'proj-r-E'     },
  theta: { label:'Angle (θ)',      unit:'deg', card:'prc-theta', val:'proj-r-theta' },
  g:     { label:'Gravity (g)',    unit:'m/s²',card:'prc-g',     val:'proj-r-g'     }
};

/* ------------------------------------------------------------
   UPDATE — solver + UI refresh
   ------------------------------------------------------------ */
function updateProjectile() {
  /* 1. Read inputs */
  var s = {
    v0:    getNullable('proj-v0'),
    theta: getNullable('proj-angle'),
    R:     getNullable('proj-R'),
    H:     getNullable('proj-H'),
    T:     getNullable('proj-T'),
    m:     getNullable('proj-m'),
    h0:    getNullable('proj-h0') !== null ? getNullable('proj-h0') : 0,
    g:     getNullable('proj-g')  !== null ? getNullable('proj-g')  : 9.81,
    vx: null, vy: null, KE: null, PE: null, E: null
  };

  /* 2. Validation */
  showInputError('proj-g',     validateInput(s.g,     { positive: true, label: 'g' }));
  showInputError('proj-v0',    validateInput(s.v0,    { min: 0, label: 'v0' }));
  showInputError('proj-angle', validateInput(s.theta, { min: -90, max: 90, label: 'θ' }));
  showInputError('proj-h0',    validateInput(s.h0,    { min: 0, label: 'h0' }));

  /* 3. Track user keys */
  var userKeys = {};
  Object.keys(s).forEach(function(k) { if (s[k] !== null) userKeys[k] = true; });
  var solveFor = document.getElementById('proj-solveFor').value;

  /* 4. Run solver */
  var result = runSolver(s, PROJ_EQUATIONS, 20);

  /* 5. Update result cards */
  Object.keys(PROJ_META).forEach(function(k) {
    var m   = PROJ_META[k];
    var vel = document.getElementById(m.val);
    if (!vel) return;
    var val = s[k];
    vel.textContent = (val !== null && val !== undefined) ? fmtSci(val) + ' ' + m.unit : '--';
    var state = (k === solveFor) ? 'target' : (userKeys[k] ? 'user' : (val !== null ? 'derived' : ''));
    styleCard(m.card, state);
  });

  /* 6. Solver steps */
  renderSolverSteps('proj-solver-steps', result.log, result.derived, s, solveFor,
    'Enter any known variables above.\nLeave unknowns blank — solver chains equations automatically.');

  /* 7. Store for animation */
  proj._v0    = (s.v0    !== null) ? s.v0    : 30;
  proj._theta = (s.theta !== null) ? s.theta : 45;
  proj._g     = s.g;
  proj._m     = (s.m     !== null) ? s.m     : 1;
  proj._h0    = s.h0;

  if (!simRunning) drawProjectile();
}

/* ------------------------------------------------------------
   STEP — sub-stepped Euler, frame by frame
   ------------------------------------------------------------ */
function stepProjectile(dt) {
  if (proj.done) { stopSim(); return; }

  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var m     = proj._m;
  var h0    = proj._h0;

  /* Initialise on first frame */
  if (simTime <= dt + 0.001) {
    proj.x    = 0;
    proj.y    = h0;          /* starts at launch height (m) */
    proj.vx   = v0 * Math.cos(theta);
    proj.vy   = v0 * Math.sin(theta);
    proj.trail = [];
    proj.data  = [];
    proj.done  = false;
  }

  /* Sub-stepped Euler — 4 steps per frame for accuracy */
  var SUBSTEPS = 4;
  var dts = dt / SUBSTEPS;

  for (var si = 0; si < SUBSTEPS; si++) {
    proj.vy -= g * dts;
    proj.x  += proj.vx * dts;
    proj.y  += proj.vy * dts;

    /* Ground collision — snap to y=0, never below */
    if (proj.y <= 0 && simTime > 0.05) {
      proj.y    = 0;
      proj.vx   = 0;
      proj.vy   = 0;
      proj.done = true;
      break;
    }
  }

  /* Trail */
  var trailEl = document.getElementById('proj-trail');
  if (trailEl && trailEl.checked) {
    proj.trail.push({ x: proj.x, y: proj.y });
    if (proj.trail.length > 800) proj.trail.shift();
  }

  /* CSV data */
  var speed = Math.sqrt(proj.vx * proj.vx + proj.vy * proj.vy);
  proj.data.push([
    parseFloat(simTime.toFixed(3)),
    parseFloat(proj.x.toFixed(3)),
    parseFloat(proj.y.toFixed(3)),
    parseFloat(proj.vx.toFixed(3)),
    parseFloat(proj.vy.toFixed(3)),
    parseFloat(speed.toFixed(3)),
    parseFloat((0.5 * m * speed * speed).toFixed(3))
  ]);

  /* Live cards */
  var ke = 0.5 * m * speed * speed;
  var elv  = document.getElementById('proj-r-v');
  var elke = document.getElementById('proj-r-KE');
  var elxy = document.getElementById('proj-r-xy');
  if (elv)  elv.textContent  = fmtSci(speed) + ' m/s';
  if (elke) elke.textContent = fmtSci(ke) + ' J';
  if (elxy) elxy.textContent = fmtSci(proj.x) + ', ' + fmtSci(proj.y);

  updateInfoBar(simTime, proj.x, proj.y, speed);
}

/* ============================================================
   DRAW FUNCTIONS
   ============================================================ */

/* ------------------------------------------------------------
   drawPlatform — FIXED position, NEVER moves based on h0.
   Renders a solid platform ledge at the launch point.
   ------------------------------------------------------------ */
function drawPlatform(ctx, platX, groundY, h0) {
  var L = PROJ_LAYOUT;
  var pw = L.platW, ph = L.platH;

  /* Platform bar */
  ctx.save();
  ctx.shadowColor = 'rgba(168,255,62,0.6)'; ctx.shadowBlur = 10;
  var grad = ctx.createLinearGradient(platX, groundY - ph, platX, groundY);
  grad.addColorStop(0, 'rgba(168,255,62,0.9)');
  grad.addColorStop(1, 'rgba(80,140,20,0.7)');
  ctx.fillStyle = grad;
  ctx.fillRect(platX, groundY - ph, pw, ph);
  ctx.shadowBlur = 0;
  ctx.strokeStyle = '#a8ff3e'; ctx.lineWidth = 1.5;
  ctx.strokeRect(platX, groundY - ph, pw, ph);
  ctx.restore();
}

/* ------------------------------------------------------------
   drawHeightIndicator — vertical dashed line from platform
   down to ground, labelled with h0.
   Only drawn when h0 > 0 (cliff mode).
   ------------------------------------------------------------ */
function drawHeightIndicator(ctx, platX, groundY, platformTopY, h0, scale) {
  if (h0 <= 0) return;

  var midX = platX + PROJ_LAYOUT.platW * 0.5 + 18;

  /* Dashed vertical line */
  ctx.save();
  ctx.strokeStyle = 'rgba(255,107,53,0.55)';
  ctx.lineWidth   = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(midX, groundY);
  ctx.lineTo(midX, platformTopY);
  ctx.stroke();
  ctx.setLineDash([]);

  /* Arrow tips */
  ctx.fillStyle = 'rgba(255,107,53,0.7)';
  /* top arrow */
  ctx.beginPath();
  ctx.moveTo(midX, platformTopY);
  ctx.lineTo(midX - 5, platformTopY + 8);
  ctx.lineTo(midX + 5, platformTopY + 8);
  ctx.closePath(); ctx.fill();
  /* bottom arrow */
  ctx.beginPath();
  ctx.moveTo(midX, groundY);
  ctx.lineTo(midX - 5, groundY - 8);
  ctx.lineTo(midX + 5, groundY - 8);
  ctx.closePath(); ctx.fill();
  ctx.restore();

  /* Height label */
  drawHeightLabel(ctx, midX + 8, (groundY + platformTopY) / 2, h0);
}

/* ------------------------------------------------------------
   drawHeightLabel — text badge showing "h0 = X m"
   ------------------------------------------------------------ */
function drawHeightLabel(ctx, x, y, h0) {
  var txt = 'h0 = ' + fmtSci(h0) + ' m';
  ctx.save();
  ctx.font = 'bold 11px Space Mono, monospace';
  var tw = ctx.measureText(txt).width;

  /* Background pill */
  ctx.fillStyle = 'rgba(255,107,53,0.18)';
  ctx.strokeStyle = 'rgba(255,107,53,0.5)';
  ctx.lineWidth = 1;
  var pad = 5;
  ctx.beginPath();
  ctx.roundRect(x - pad, y - 10, tw + pad * 2, 18, 4);
  ctx.fill(); ctx.stroke();

  /* Text */
  ctx.fillStyle = '#ff6b35';
  ctx.fillText(txt, x, y + 3);
  ctx.restore();
}

/* ------------------------------------------------------------
   drawProjectile — main render function
   
   Layout (all fixed pixel values):
     groundY  = canvas height × groundFrac
     platX    = margin
     platTopY = groundY - platH  (same as groundY when h0=0)
   
   Physics world → canvas pixel transform:
     The scale is derived from the trajectory extents.
     The platform is always at (margin, groundY) in pixels.
     The ball starts at (platX + platW/2, groundY - platH) visually
     regardless of h0.
   ------------------------------------------------------------ */
function drawProjectile() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12';
  ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var L = PROJ_LAYOUT;

  /* Fixed visual anchor points */
  var margin  = L.margin;
  var groundY = H * L.groundFrac;           /* fixed ground pixel y   */
  var platX   = margin;                     /* fixed platform left     */

  /* Physics params */
  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var h0    = proj._h0 || 0;

  var vx0 = v0 * Math.cos(theta);
  var vy0 = v0 * Math.sin(theta);

  /* Full trajectory extents for scale */
  var T_flight = solveFlightTime(vy0, g, h0);
  var apexH    = h0 + (vy0 * vy0) / (2 * g);   /* above ground */
  var R        = vx0 * T_flight;

  /* Scale: fit trajectory into usable canvas area above ground */
  var usableW  = W - margin * 2;
  var usableH  = groundY - margin;              /* space above ground */
  var scaleX   = (R > 0)     ? usableW / Math.max(R, 1) : 8;
  var scaleY   = (apexH > 0) ? usableH / Math.max(apexH, 1) * 0.88 : 8;
  var scale    = Math.min(scaleX, scaleY, 80);  /* cap so tiny inputs don't over-scale */

  /*
   * World → canvas pixel transforms.
   *
   * Origin (x=0, y=0) maps to (platX + platW/2, groundY).
   * This keeps the launch point always visually on the platform.
   * h0 is embedded in the physics state, NOT in the visual origin.
   */
  var originPx = platX + L.platW / 2;   /* launch point x in pixels */

  function wx(x) { return originPx + x * scale; }
  function wy(y) { return groundY  - y * scale; } /* y=0 → groundY */

  /* ---- Ground line ---- */
  ctx.strokeStyle = 'rgba(0,212,255,0.4)';
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(margin, groundY);
  ctx.lineTo(W - margin, groundY);
  ctx.stroke();

  /* ---- Ground fill (subtle) ---- */
  var gFill = ctx.createLinearGradient(0, groundY, 0, groundY + 20);
  gFill.addColorStop(0, 'rgba(0,212,255,0.08)');
  gFill.addColorStop(1, 'rgba(0,212,255,0)');
  ctx.fillStyle = gFill;
  ctx.fillRect(margin, groundY, W - margin * 2, 20);

  /* ---- Platform (FIXED — never moves) ---- */
  drawPlatform(ctx, platX, groundY, h0);

  /* ---- Height indicator (cliff mode only) ---- */
  var platformTopY = groundY - L.platH;
  drawHeightIndicator(ctx, platX, groundY, platformTopY, h0, scale);

  /* ---- Ghost trajectory preview ---- */
  ctx.strokeStyle = 'rgba(0,212,255,0.2)';
  ctx.lineWidth   = 1.5;
  ctx.setLineDash([6, 5]);
  ctx.beginPath();
  var GHOST_STEPS = 100;
  var firstPt = true;
  for (var i = 0; i <= GHOST_STEPS; i++) {
    var t  = T_flight * i / GHOST_STEPS;
    var gx = vx0 * t;
    var gy = h0 + vy0 * t - 0.5 * g * t * t;
    if (gy < 0) { /* draw to exact ground crossing */
      /* Interpolate to find exact x at y=0 */
      var prevT = T_flight * (i - 1) / GHOST_STEPS;
      var prevY = h0 + vy0 * prevT - 0.5 * g * prevT * prevT;
      if (prevY > 0) {
        var frac = prevY / (prevY - gy);
        var landT = prevT + frac * (T_flight / GHOST_STEPS);
        ctx.lineTo(wx(vx0 * landT), wy(0));
      }
      break;
    }
    if (firstPt) { ctx.moveTo(wx(gx), wy(gy)); firstPt = false; }
    else          { ctx.lineTo(wx(gx), wy(gy)); }
  }
  ctx.stroke();
  ctx.setLineDash([]);

  /* ---- Actual trail ---- */
  if (proj.trail.length > 1) {
    ctx.beginPath();
    for (var ti = 0; ti < proj.trail.length; ti++) {
      var p = proj.trail[ti];
      if (ti === 0) ctx.moveTo(wx(p.x), wy(p.y));
      else          ctx.lineTo(wx(p.x), wy(p.y));
    }
    ctx.strokeStyle = 'rgba(0,212,255,0.7)';
    ctx.lineWidth   = 2;
    ctx.stroke();
  }

  /* ---- Projectile ball ---- */
  /*
   * Visual position of ball:
   *   - Before launch: sits on top of the platform surface (platformTopY).
   *   - During flight: maps physics y to canvas using wy().
   *   - After landing: snaps to groundY.
   */
  var bxPx, byPx;
  if (!simRunning && simTime === 0) {
    /* Idle — ball rests on platform */
    bxPx = originPx;
    byPx = platformTopY - L.ballR;
  } else {
    bxPx = wx(proj.x);
    byPx = wy(proj.y);
    /* Clamp ball above ground line — never visually below it */
    if (byPx > groundY - L.ballR) byPx = groundY - L.ballR;
  }
  drawGlowCircle(ctx, bxPx, byPx, L.ballR, '#00d4ff');

  /* ---- Velocity vectors ---- */
  var vecEl = document.getElementById('proj-vectors');
  if (vecEl && vecEl.checked && simRunning) {
    var vecScale = Math.max(scale * 0.4, 2);
    if (Math.abs(proj.vx) > 0.05)
      drawArrow(ctx, bxPx, byPx, bxPx + proj.vx * vecScale, byPx,              '#a8ff3e', 'vx', 2);
    if (Math.abs(proj.vy) > 0.05)
      drawArrow(ctx, bxPx, byPx, bxPx,                      byPx - proj.vy * vecScale, '#ff6b35', 'vy', 2);
  }

  /* ---- Apex marker ---- */
  var T_up  = vy0 / g;
  var apexX = vx0 * T_up;
  if (T_up > 0 && apexH > 0.01 && R > 0) {
    var apexPx = wx(apexX), apexPy = wy(apexH);
    ctx.strokeStyle = 'rgba(168,255,62,0.35)';
    ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.moveTo(apexPx, apexPy);
    ctx.lineTo(apexPx, groundY);
    ctx.stroke(); ctx.setLineDash([]);

    ctx.fillStyle = '#a8ff3e';
    ctx.font = '11px Space Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('H=' + fmtSci(apexH) + 'm', apexPx, apexPy - 12);
    ctx.textAlign = 'left';
  }

  /* ---- Range label ---- */
  if (R > 0.01) {
    ctx.fillStyle = 'rgba(0,212,255,0.75)';
    ctx.font = '11px Space Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('R = ' + fmtSci(R) + ' m', wx(R / 2), groundY + 18);
    ctx.textAlign = 'left';
  }

  /* ---- Mode badge (cliff vs ground) ---- */
  var modeLabel = (h0 > 0) ? 'CLIFF MODE  h0 = ' + fmtSci(h0) + ' m' : 'GROUND LAUNCH';
  ctx.fillStyle = (h0 > 0) ? 'rgba(255,107,53,0.8)' : 'rgba(107,122,153,0.6)';
  ctx.font = '10px Space Mono, monospace';
  ctx.fillText(modeLabel, margin + 4, groundY - L.platH - 18);
}

/* ------------------------------------------------------------
   CSV EXPORT
   ------------------------------------------------------------ */
function exportProjectileCSV() {
  if (!proj.data || proj.data.length === 0) {
    alert('Run the simulation first to generate data.');
    return;
  }
  exportCSV(
    'projectile_data.csv',
    ['time_s', 'x_m', 'y_m', 'vx_ms', 'vy_ms', 'speed_ms', 'KE_J'],
    proj.data
  );
}
