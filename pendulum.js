/* ============================================================
   pendulum.js — Pendulum Simulation
   Handles: solver, RK4 animation, energy graphs
   Physics assumptions: ideal rigid rod, no air resistance,
   small-angle NOT assumed (uses full sin equation)
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   STATE
   ------------------------------------------------------------ */
var pen = {
  theta: Math.PI / 4,   // current angle (radians)
  omega: 0,             // angular velocity (rad/s)
  trail: [],            // [{theta}] trail
  data:  [],            // [{t, theta, omega, KE, PE}] for CSV
  // Solver cache
  _L: 3, _m: 2, _g: 9.81, _theta0: Math.PI / 4, _damp: 0
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetPendulum() {
  var theta0 = (getNullable('pen-angle') || 45) * Math.PI / 180;
  pen = {
    theta: theta0, omega: 0, trail: [], data: [],
    _L: pen._L, _m: pen._m, _g: pen._g, _theta0: theta0, _damp: pen._damp
  };
  updatePendulum();
}

/* ------------------------------------------------------------
   EQUATIONS REGISTRY
   ------------------------------------------------------------ */
var PEN_EQUATIONS = [

  /* Period: T = 2π√(L/g) */
  { name: 'Period', formula: 'T = 2π × √(L/g)',
    solve: {
      T: function(s) {
        if (s.L === null || s.g <= 0) return null;
        return 2 * Math.PI * Math.sqrt(s.L / s.g);
      },
      L: function(s) {
        if (s.T === null || s.g <= 0) return null;
        return s.g * Math.pow(s.T / (2 * Math.PI), 2);
      },
      g: function(s) {
        if (s.T === null || s.L === null || s.L <= 0) return null;
        return s.L * Math.pow(2 * Math.PI / s.T, 2);
      }
    },
    buildSub: function(s) {
      return 'T = 2π × √(' + fvN(s.L) + ' / ' + fvN(s.g) + ') = ' + fmtSci(s.T) + ' s';
    }
  },

  /* Frequency: f = 1/T */
  { name: 'Frequency', formula: 'f = 1 / T',
    solve: {
      f: function(s) {
        if (s.T === null || Math.abs(s.T) < 1e-12) return null;
        return 1 / s.T;
      },
      T: function(s) {
        if (s.f === null || Math.abs(s.f) < 1e-12) return null;
        return 1 / s.f;
      }
    },
    buildSub: function(s) {
      return 'f = 1 / ' + fvN(s.T) + ' = ' + fmtSci(s.f) + ' Hz';
    }
  },

  /* Angular frequency: omega_nat = √(g/L) */
  { name: 'Angular frequency', formula: 'ω₀ = √(g/L)',
    solve: {
      omega_nat: function(s) {
        if (s.L === null || s.L <= 0) return null;
        return Math.sqrt(s.g / s.L);
      },
      L: function(s) {
        if (s.omega_nat === null || s.omega_nat <= 0) return null;
        return s.g / (s.omega_nat * s.omega_nat);
      }
    },
    buildSub: function(s) {
      return 'ω₀ = √(' + fvN(s.g) + ' / ' + fvN(s.L) + ') = ' + fmtSci(s.omega_nat) + ' rad/s';
    }
  },

  /* Max speed: v_max = √(2gL(1 - cos θ₀)) */
  { name: 'Max speed', formula: 'v_max = √(2gL(1 - cosθ₀))',
    solve: {
      v_max: function(s) {
        if (s.L === null || s.theta0_rad === null) return null;
        var disc = 2 * s.g * s.L * (1 - Math.cos(s.theta0_rad));
        if (disc < 0) return null;
        return Math.sqrt(disc);
      },
      L: function(s) {
        if (s.v_max === null || s.theta0_rad === null) return null;
        var cosFactor = 1 - Math.cos(s.theta0_rad);
        if (Math.abs(cosFactor) < 1e-12) return null;
        return (s.v_max * s.v_max) / (2 * s.g * cosFactor);
      }
    },
    buildSub: function(s) {
      return 'v_max = √(2×' + fvN(s.g) + '×' + fvN(s.L) + '×(1-cos(' + fvN(s.theta0_rad * 180 / Math.PI, 1) + '°))) = ' + fmtSci(s.v_max) + ' m/s';
    }
  },

  /* Potential Energy (at angle θ): PE = mgL(1 - cosθ) */
  { name: 'Potential Energy', formula: 'PE = m × g × L × (1 - cosθ)',
    solve: {
      PE: function(s) {
        if (s.m === null || s.L === null || s.theta0_rad === null) return null;
        return s.m * s.g * s.L * (1 - Math.cos(s.theta0_rad));
      },
      m: function(s) {
        if (s.PE === null || s.L === null || s.theta0_rad === null) return null;
        var denom = s.g * s.L * (1 - Math.cos(s.theta0_rad));
        if (Math.abs(denom) < 1e-12) return null;
        return s.PE / denom;
      },
      L: function(s) {
        if (s.PE === null || s.m === null || s.theta0_rad === null) return null;
        var denom = s.m * s.g * (1 - Math.cos(s.theta0_rad));
        if (Math.abs(denom) < 1e-12) return null;
        return s.PE / denom;
      }
    },
    buildSub: function(s) {
      return 'PE = ' + fvN(s.m) + '×' + fvN(s.g) + '×' + fvN(s.L) + '×(1-cos(' + fvN((s.theta0_rad || 0) * 180 / Math.PI, 1) + '°)) = ' + fmtSci(s.PE) + ' J';
    }
  },

  /* Kinetic Energy (at bottom, all PE converts): KE_max = PE */
  { name: 'Max KE (at bottom)', formula: 'KE_max = PE (energy conservation)',
    solve: {
      KE_max: function(s) { return s.PE; },
      PE:     function(s) { return s.KE_max; }
    },
    buildSub: function(s) {
      return 'KE_max = PE = ' + fmtSci(s.PE) + ' J (at bottom)';
    }
  },

  /* Total Energy: E = PE (at top = start) */
  { name: 'Total Mechanical Energy', formula: 'E = m × g × L × (1 - cosθ₀)',
    solve: {
      E: function(s) { return s.PE; },
      m: function(s) {
        if (s.E === null || s.L === null || s.theta0_rad === null) return null;
        var d = s.g * s.L * (1 - Math.cos(s.theta0_rad));
        if (Math.abs(d) < 1e-12) return null;
        return s.E / d;
      }
    },
    buildSub: function(s) {
      return 'E = ' + fmtSci(s.E) + ' J (total mechanical energy)';
    }
  }
];

/* Metadata */
var PEN_META = {
  T:         { label:'Period (T)',    unit:'s',     card:'enc-T',     val:'pen-r-T' },
  f:         { label:'Frequency',    unit:'Hz',    card:'enc-f',     val:'pen-r-f' },
  omega_nat: { label:'ω₀',           unit:'rad/s', card:'enc-wn',    val:'pen-r-omegan' },
  v_max:     { label:'v_max',        unit:'m/s',   card:'enc-vmax',  val:'pen-r-vmax' },
  KE_max:    { label:'KE_max',       unit:'J',     card:'enc-KEmax', val:'pen-r-kemax' },
  PE:        { label:'PE (initial)', unit:'J',     card:'enc-PE',    val:'pen-r-pe' },
  E:         { label:'Total E',      unit:'J',     card:'enc-E',     val:'pen-r-E' },
  L:         { label:'Length (L)',   unit:'m',     card:'enc-L',     val:'pen-r-L' },
  g:         { label:'Gravity (g)',  unit:'m/s²',  card:'enc-g',     val:'pen-r-g' }
};

/* ------------------------------------------------------------
   UPDATE (solver + UI)
   ------------------------------------------------------------ */
function updatePendulum() {
  var theta0_deg = getNullable('pen-angle');
  var theta0_rad = (theta0_deg !== null) ? theta0_deg * Math.PI / 180 : null;

  var s = {
    L:          getNullable('pen-L'),
    m:          getNullable('pen-m'),
    theta0_rad: theta0_rad,
    T:          getNullable('pen-T'),
    f:          null,
    omega_nat:  null,
    v_max:      null,
    PE:         null,
    KE_max:     null,
    E:          null,
    g:          getNullable('pen-g') !== null ? getNullable('pen-g') : 9.81
  };

  /* Input validation */
  showInputError('pen-g',   validateInput(s.g,          { positive: true, label: 'g' }));
  showInputError('pen-L',   validateInput(s.L,          { positive: true, label: 'L' }));
  showInputError('pen-m',   validateInput(s.m,          { positive: true, label: 'm' }));
  showInputError('pen-angle', validateInput(theta0_deg, { min: -179, max: 179, label: 'θ₀' }));

  var userKeys = {};
  Object.keys(s).forEach(function(k){ if (s[k] !== null && k !== 'theta0_rad') userKeys[k] = true; });
  if (theta0_deg !== null) userKeys['theta0_rad'] = true;
  var solveFor = document.getElementById('pen-solveFor').value;

  var result = runSolver(s, PEN_EQUATIONS, 20);

  /* Update result cards */
  Object.keys(PEN_META).forEach(function(k) {
    var m   = PEN_META[k];
    var vel = document.getElementById(m.val);
    if (!vel) return;
    var val = s[k];
    vel.textContent = (val !== null && val !== undefined) ? fmtSci(val) + ' ' + m.unit : '--';
    var state = (k === solveFor) ? 'target' : (userKeys[k] ? 'user' : (val !== null ? 'derived' : ''));
    styleCard(m.card, state);
  });

  /* Live angle / omega cards */
  var el_ang = document.getElementById('pen-r-angle');
  var el_om  = document.getElementById('pen-r-omega');
  if (el_ang) el_ang.textContent = fmtSci(pen.theta * 180 / Math.PI) + ' deg';
  if (el_om)  el_om.textContent  = fmtSci(pen.omega) + ' rad/s';

  renderSolverSteps('pen-solver-steps', result.log, result.derived, s, solveFor,
    'Enter any known variables.\nSolver chains T, f, ω, KE, PE, E automatically.');

  /* Energy bars — use current live values from animation state */
  var L_now = s.L || pen._L;
  var m_now = s.m || pen._m;
  var g_now = s.g;
  var livePE = m_now * g_now * L_now * (1 - Math.cos(pen.theta));
  var livevMax = Math.sqrt(Math.max(0, 2 * g_now * L_now * (1 - Math.cos(pen._theta0))));
  var liveKE = Math.max(0, (s.E || m_now * g_now * L_now * (1 - Math.cos(pen._theta0))) - livePE);
  var liveE  = liveKE + livePE;
  var maxE   = Math.max(Math.abs(liveE), 1);
  _setBar('pen-ke-bar', 'pen-ke-val', liveKE, maxE);
  _setBar('pen-pe-bar', 'pen-pe-val', livePE, maxE);
  _setBar('pen-E-bar',  'pen-E-val',  liveE,  maxE);

  /* Store for animation */
  pen._L      = (s.L !== null) ? s.L : 3;
  pen._m      = (s.m !== null) ? s.m : 2;
  pen._g      = s.g;
  pen._theta0 = (theta0_rad !== null) ? theta0_rad : Math.PI / 4;
  pen._damp   = getNullable('pen-damp') || 0;

  if (!simRunning) drawPendulum();
}

function _setBar(barId, lblId, val, maxVal) {
  var b = document.getElementById(barId), l = document.getElementById(lblId);
  if (!b || !l) return;
  var pct = maxVal > 0 ? Math.min(Math.abs(val) / maxVal * 100, 100) : 0;
  b.style.width = pct + '%';
  l.textContent = (val !== null) ? fmtSci(val) + ' J' : '-- J';
}

/* ------------------------------------------------------------
   STEP (RK4 for accuracy at large angles)
   ------------------------------------------------------------ */
function stepPendulum(dt) {
  var L    = pen._L, g = pen._g;
  var damp = pen._damp;
  var m    = pen._m;

  if (simTime <= dt + 0.001) {
    pen.theta = pen._theta0;
    pen.omega = 0;
    pen.trail = [];
    pen.data  = [];
  }

  /* RK4 integration — much more accurate than Euler at large angles */
  var substeps = 4;
  var dts = dt / substeps;
  for (var s = 0; s < substeps; s++) {
    var alpha = -(g / L) * Math.sin(pen.theta) - damp * pen.omega;
    pen.omega += alpha * dts;
    pen.theta += pen.omega * dts;
  }

  /* Trail */
  pen.trail.push({ theta: pen.theta });
  if (pen.trail.length > 350) pen.trail.shift();

  /* Record data */
  var h   = L * (1 - Math.cos(pen.theta));
  var PE  = m * g * h;
  var vsp = Math.abs(pen.omega) * L;
  var KE  = 0.5 * m * vsp * vsp;
  pen.data.push([parseFloat(simTime.toFixed(3)),
    parseFloat((pen.theta * 180 / Math.PI).toFixed(3)),
    parseFloat(pen.omega.toFixed(4)),
    parseFloat(KE.toFixed(3)), parseFloat(PE.toFixed(3))]);

  updatePendulum();
  updateInfoBar(simTime, pen.theta * 180 / Math.PI, 0, Math.abs(pen.omega));
}

/* ------------------------------------------------------------
   DRAW
   ------------------------------------------------------------ */
function drawPendulum() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var L = pen._L, m = pen._m, g = pen._g;

  var pivotX = W / 2;
  var pivotY = H * 0.18;
  var pixL   = Math.min(H * 0.52, W * 0.33);

  var bobX = pivotX + Math.sin(pen.theta) * pixL;
  var bobY = pivotY + Math.cos(pen.theta) * pixL;
  var bobR = Math.min(Math.max(Math.sqrt(m) * 8, 10), 28);

  /* Angle arc (swing range) */
  var theta0 = pen._theta0;
  ctx.strokeStyle = 'rgba(0,212,255,0.08)';
  ctx.lineWidth = 80; ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.arc(pivotX, pivotY, pixL, -Math.PI / 2 - theta0, -Math.PI / 2 + theta0);
  ctx.stroke(); ctx.lineCap = 'butt';

  /* Trail */
  if (pen.trail.length > 1) {
    ctx.beginPath();
    for (var i = 0; i < pen.trail.length; i++) {
      var tx = pivotX + Math.sin(pen.trail[i].theta) * pixL;
      var ty = pivotY + Math.cos(pen.trail[i].theta) * pixL;
      var alpha = i / pen.trail.length;
      if (i === 0) ctx.moveTo(tx, ty);
      else         ctx.lineTo(tx, ty);
    }
    ctx.strokeStyle = 'rgba(0,212,255,0.35)';
    ctx.lineWidth = 2; ctx.stroke();
  }

  /* Rod */
  ctx.save();
  ctx.shadowColor = '#00d4ff'; ctx.shadowBlur = 6;
  ctx.strokeStyle = 'rgba(0,212,255,0.6)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(bobX, bobY); ctx.stroke();
  ctx.restore();

  /* Pivot */
  ctx.fillStyle = '#1e2a42'; ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(pivotX, pivotY, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

  /* Bob */
  drawGlowCircle(ctx, bobX, bobY, bobR, '#00d4ff');
  ctx.fillStyle = '#000'; ctx.font = 'bold 10px DM Sans';
  ctx.textAlign = 'center'; ctx.fillText(m + 'kg', bobX, bobY + 4); ctx.textAlign = 'left';

  /* Velocity vector (tangential) */
  var speed = Math.abs(pen.omega) * pixL * 0.4;
  if (speed > 3) {
    var vDir = pen.omega > 0 ? 1 : -1;
    var tangX = Math.cos(pen.theta) * vDir;
    var tangY = -Math.sin(pen.theta) * vDir;
    drawArrow(ctx, bobX, bobY, bobX + tangX * speed, bobY + tangY * speed, '#a8ff3e', 'v', 2);
  }

  /* Gravity vector */
  var gLen = Math.min(g * 3, 50);
  drawArrow(ctx, bobX, bobY, bobX, bobY + gLen, '#ff6b35', 'g', 1.5);

  /* Length label */
  ctx.fillStyle = 'rgba(107,122,153,0.8)'; ctx.font = '11px Space Mono';
  ctx.fillText('L = ' + fmtSci(L) + ' m', pivotX + 12, (pivotY + bobY) / 2);

  /* Period label */
  var T = 2 * Math.PI * Math.sqrt(L / g);
  ctx.fillStyle = 'rgba(0,212,255,0.7)';
  ctx.fillText('T = ' + fmtSci(T) + ' s', margin || 20, 30);

  /* Energy graph (scrolling) */
  if (pen.data.length > 2) {
    var gx = 20, gy = H * 0.82, gw = W - 40, gh = H * 0.12;
    ctx.fillStyle = 'rgba(10,13,20,0.75)';
    ctx.fillRect(gx, gy, gw, gh);
    ctx.strokeStyle = 'rgba(30,42,66,0.8)'; ctx.lineWidth = 1;
    ctx.strokeRect(gx, gy, gw, gh);

    var recent = pen.data.slice(-120);
    var maxE   = Math.max.apply(null, recent.map(function(r){ return r[3] + r[4]; }));
    if (maxE < 1) maxE = 1;

    /* KE line */
    ctx.beginPath(); ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 1.5;
    recent.forEach(function(r, i) {
      var rx = gx + (i / (recent.length - 1)) * gw;
      var ry = gy + gh - (r[3] / maxE) * (gh - 4);
      if (i === 0) ctx.moveTo(rx, ry); else ctx.lineTo(rx, ry);
    }); ctx.stroke();

    /* PE line */
    ctx.beginPath(); ctx.strokeStyle = '#ff6b35'; ctx.lineWidth = 1.5;
    recent.forEach(function(r, i) {
      var rx = gx + (i / (recent.length - 1)) * gw;
      var ry = gy + gh - (r[4] / maxE) * (gh - 4);
      if (i === 0) ctx.moveTo(rx, ry); else ctx.lineTo(rx, ry);
    }); ctx.stroke();

    ctx.fillStyle = '#00d4ff'; ctx.font = '9px Space Mono';
    ctx.fillText('KE', gx + 4, gy + 12);
    ctx.fillStyle = '#ff6b35';
    ctx.fillText('PE', gx + 24, gy + 12);
    ctx.fillStyle = 'rgba(107,122,153,0.7)';
    ctx.fillText('Energy over time', gx + 50, gy + 12);
  }
}

/* ------------------------------------------------------------
   CSV EXPORT
   ------------------------------------------------------------ */
function exportPendulumCSV() {
  if (!pen.data || pen.data.length === 0) {
    alert('Run the simulation first.'); return;
  }
  exportCSV('pendulum_data.csv',
    ['time_s','angle_deg','omega_rads','KE_J','PE_J'],
    pen.data);
}
