/* ============================================================
   projectile.js — Projectile Motion Simulation
   Handles: solver, animation state, step/draw functions
   Physics assumptions: no air resistance, flat ground, constant g
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   STATE
   ------------------------------------------------------------ */
var proj = {
  x: 0, y: 0,        // current position (m)
  vx: 0, vy: 0,      // current velocity (m/s)
  trail: [],          // [{x,y}] trail points
  done: false,        // has projectile landed?
  data: [],           // [{t,x,y,vx,vy,v,KE}] for CSV export
  // Solver-computed launch params (used by step + draw):
  _v0: 30, _theta: 45, _g: 9.81, _m: 1, _h0: 0
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetProjectile() {
  proj = { x: 0, y: 0, vx: 0, vy: 0, trail: [], done: false, data: [],
           _v0: proj._v0, _theta: proj._theta, _g: proj._g,
           _m: proj._m, _h0: proj._h0 };
  updateProjectile();
}

/* ------------------------------------------------------------
   EQUATIONS REGISTRY
   Each equation: { name, formula, solve: { key: fn(state)->number|null }, buildSub: fn }
   Guards: return null if prereqs missing, division by zero, sqrt<0
   ------------------------------------------------------------ */
var PROJ_EQUATIONS = [

  /* vx = v0 * cos(theta) */
  { name: 'Horizontal velocity', formula: 'vx = v0 * cos(θ)',
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
      if (s.vx !== null)
        return 'vx = ' + fvN(s.v0) + ' × cos(' + fvN(s.theta, 1) + '°) = ' + fmtSci(s.vx) + ' m/s';
      return 'v0 = ' + fmtSci(s.vx) + ' / cos(' + fvN(s.theta, 1) + '°) = ' + fmtSci(s.v0) + ' m/s';
    }
  },

  /* vy = v0 * sin(theta) */
  { name: 'Vertical velocity', formula: 'vy = v0 * sin(θ)',
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

  /* v0 from vx and vy */
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

  /* theta from vx and vy */
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

  /* H = h0 + vy²/(2g)  — max height */
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

  /* T = vy/g + √(2H/g)  — total flight time (launch from h0 to ground) */
  { name: 'Flight time', formula: 'T = vy/g + √(2H/g)',
    solve: {
      T: function(s) {
        if (s.vy === null || s.H === null) return null;
        var g = s.g;
        return s.vy / g + Math.sqrt(2 * s.H / g);
      },
      g: function(s) {
        if (s.T === null || s.H === null || s.vy === null) return null;
        // Iterative — skip (too complex to invert analytically)
        return null;
      }
    },
    buildSub: function(s) {
      return 'T = ' + fvN(s.vy) + '/' + fvN(s.g) + ' + √(2×' + fvN(s.H, 2) + '/' + fvN(s.g) + ') = ' + fmtSci(s.T) + ' s';
    }
  },

  /* R = vx * T  — range */
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

  /* PE = mgh0 */
  { name: 'Potential Energy', formula: 'PE = m × g × h0',
    solve: {
      PE: function(s) {
        if (s.m === null || s.h0 === null) return null;
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
  R:     { label:'Range (R)',      unit:'m',   card:'prc-R',     val:'proj-r-R' },
  H:     { label:'Max Height (H)', unit:'m',   card:'prc-H',     val:'proj-r-H' },
  T:     { label:'Flight Time',    unit:'s',   card:'prc-T',     val:'proj-r-T' },
  v0:    { label:'v0',             unit:'m/s', card:'prc-v0',    val:'proj-r-v0' },
  vx:    { label:'vx',             unit:'m/s', card:'prc-vx',    val:'proj-r-vx' },
  vy:    { label:'vy (initial)',    unit:'m/s', card:'prc-vy',    val:'proj-r-vy' },
  KE:    { label:'KE',             unit:'J',   card:'prc-KE',    val:'proj-r-KE' },
  PE:    { label:'PE',             unit:'J',   card:'prc-PE',    val:'proj-r-PE' },
  E:     { label:'Total E',        unit:'J',   card:'prc-E',     val:'proj-r-E' },
  theta: { label:'Angle (θ)',      unit:'deg', card:'prc-theta', val:'proj-r-theta' },
  g:     { label:'Gravity (g)',    unit:'m/s²',card:'prc-g',     val:'proj-r-g' }
};

/* ------------------------------------------------------------
   UPDATE (solver + UI)
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
    g:     getNullable('proj-g') !== null ? getNullable('proj-g') : 9.81,
    vx: null, vy: null, KE: null, PE: null, E: null
  };

  /* 2. Input validation */
  showInputError('proj-g',     validateInput(s.g,     { positive: true, label: 'g' }));
  showInputError('proj-v0',    validateInput(s.v0,    { min: 0, label: 'v0' }));
  showInputError('proj-angle', validateInput(s.theta, { min: -90, max: 90, label: 'θ' }));

  /* 3. Track user-provided keys */
  var userKeys = {};
  Object.keys(s).forEach(function(k){ if (s[k] !== null) userKeys[k] = true; });
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
   STEP (physics integration, called each animation frame)
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
    proj.x     = 0;
    proj.y     = h0;
    proj.vx    = v0 * Math.cos(theta);
    proj.vy    = v0 * Math.sin(theta);
    proj.trail = [];
    proj.data  = [];
  }

  /* Euler integration */
  proj.vy -= g * dt;
  proj.x  += proj.vx * dt;
  proj.y  += proj.vy * dt;

  /* Trail */
  if (document.getElementById('proj-trail') && document.getElementById('proj-trail').checked) {
    proj.trail.push({ x: proj.x, y: proj.y });
    if (proj.trail.length > 600) proj.trail.shift();
  }

  /* Record data for CSV */
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

  /* Land detection */
  if (proj.y <= 0 && proj.vy < 0 && simTime > 0.05) {
    proj.y = 0; proj.done = true;
  }

  /* Update live result cards */
  var ke = 0.5 * m * speed * speed;
  var el_v  = document.getElementById('proj-r-v');
  var el_ke = document.getElementById('proj-r-KE');
  var el_xy = document.getElementById('proj-r-xy');
  if (el_v)  el_v.textContent  = fmtSci(speed) + ' m/s';
  if (el_ke) el_ke.textContent = fmtSci(ke) + ' J';
  if (el_xy) el_xy.textContent = fmtSci(proj.x) + ', ' + fmtSci(proj.y);

  updateInfoBar(simTime, proj.x, proj.y, speed);
}

/* ------------------------------------------------------------
   DRAW
   ------------------------------------------------------------ */
function drawProjectile() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12';
  ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var v0    = proj._v0;
  var theta = proj._theta * Math.PI / 180;
  var g     = proj._g;
  var h0    = proj._h0;

  /* --- Scale calculation --- */
  var sinT = Math.sin(theta), cosT = Math.cos(theta);
  var vy0  = v0 * sinT, vx0 = v0 * cosT;
  // Time to apex + time to fall from apex to ground
  var T_up    = (g > 0) ? vy0 / g : 0;
  var apexH   = h0 + (g > 0 ? vy0 * vy0 / (2 * g) : 0);
  var T_down  = (g > 0 && apexH >= 0) ? Math.sqrt(2 * apexH / g) : 0;
  var T_total = T_up + T_down;
  var R       = vx0 * T_total;

  var margin  = 60;
  var usableW = W - margin * 2;
  var usableH = H - margin * 2;
  var scaleX  = (R    > 0) ? usableW / Math.max(R, 1) : 8;
  var scaleY  = (apexH > 0) ? usableH / Math.max(apexH, 1) * 0.85 : 8;
  var scale   = Math.min(scaleX, scaleY);

  // World → canvas transform
  function wx(x) { return margin + x * scale; }
  function wy(y) { return H - margin - y * scale; }

  /* --- Ground line --- */
  ctx.strokeStyle = 'rgba(0,212,255,0.35)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(margin, wy(0));
  ctx.lineTo(W - margin, wy(0));
  ctx.stroke();

  /* --- Initial height marker --- */
  if (h0 > 0) {
    ctx.strokeStyle = 'rgba(168,255,62,0.3)';
    ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(margin, wy(h0));
    ctx.lineTo(margin + 60, wy(h0));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono';
    ctx.fillText('h0=' + h0 + 'm', margin + 5, wy(h0) - 5);
  }

  /* --- Ghost trajectory preview (dashed) --- */
  ctx.strokeStyle = 'rgba(0,212,255,0.18)';
  ctx.lineWidth = 1.5; ctx.setLineDash([5, 5]);
  ctx.beginPath();
  var steps = 80;
  for (var i = 0; i <= steps; i++) {
    var t  = T_total * i / steps;
    var gx = vx0 * t;
    var gy = h0 + vy0 * t - 0.5 * g * t * t;
    if (gy < 0) break;
    if (i === 0) ctx.moveTo(wx(gx), wy(gy));
    else         ctx.lineTo(wx(gx), wy(gy));
  }
  ctx.stroke(); ctx.setLineDash([]);

  /* --- Trail --- */
  if (proj.trail.length > 1) {
    ctx.beginPath();
    for (var ti = 0; ti < proj.trail.length; ti++) {
      var p = proj.trail[ti];
      var alpha = ti / proj.trail.length;
      if (ti === 0) ctx.moveTo(wx(p.x), wy(p.y));
      else          ctx.lineTo(wx(p.x), wy(p.y));
    }
    ctx.strokeStyle = 'rgba(0,212,255,0.6)';
    ctx.lineWidth = 2; ctx.stroke();
  }

  /* --- Projectile ball --- */
  var bx = wx(proj.x), by = wy(proj.y);
  drawGlowCircle(ctx, bx, by, 11, '#00d4ff');

  /* --- Velocity vectors --- */
  if (document.getElementById('proj-vectors') && document.getElementById('proj-vectors').checked) {
    var vecScale = 3;
    // Horizontal (vx)
    if (Math.abs(proj.vx) > 0.1)
      drawArrow(ctx, bx, by, bx + proj.vx * vecScale, by, '#a8ff3e', 'vx', 2);
    // Vertical (vy)
    if (Math.abs(proj.vy) > 0.1)
      drawArrow(ctx, bx, by, bx, by - proj.vy * vecScale, '#ff6b35', 'vy', 2);
  }

  /* --- Apex marker --- */
  if (T_up > 0 && R > 0) {
    var apexX = vx0 * T_up;
    ctx.strokeStyle = 'rgba(168,255,62,0.4)';
    ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(wx(apexX), wy(apexH));
    ctx.lineTo(wx(apexX), wy(0));
    ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono';
    ctx.textAlign = 'center';
    ctx.fillText('H=' + fmtSci(apexH) + 'm', wx(apexX), wy(apexH) - 10);
    ctx.textAlign = 'left';
  }

  /* --- Range label --- */
  if (R > 0) {
    ctx.fillStyle = 'rgba(0,212,255,0.7)'; ctx.font = '11px Space Mono';
    ctx.textAlign = 'center';
    ctx.fillText('R=' + fmtSci(R) + 'm', wx(R / 2), wy(0) + 20);
    ctx.textAlign = 'left';
  }
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
    ['time_s','x_m','y_m','vx_ms','vy_ms','speed_ms','KE_J'],
    proj.data
  );
}
