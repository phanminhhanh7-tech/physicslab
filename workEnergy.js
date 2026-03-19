/* ============================================================
   workEnergy.js — Work & Energy Simulation
   Handles: 13-equation symbolic solver, block animation
   Physics assumptions: 1D horizontal surface, point-mass block,
   constant force, no air resistance
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   STATE
   ------------------------------------------------------------ */
var wrk = {
  x: 0,            // normalised position 0–1 on track
  v: 0,            // scaled velocity for animation
  done: false,
  ke_history: [],  // KE values for in-canvas graph
  data: [],        // [{t,x,v,KE}] for CSV export
  _computed: {}    // cache of solver output for step/draw
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetWork() {
  /* Clear all inputs except gravity */
  ['work-m','work-F','work-angle','work-d','work-vi','work-vf',
   'work-W','work-t','work-h','work-KE','work-P','work-mu'
  ].forEach(function(id) {
    var el = document.getElementById(id);
    if (el) el.value = '';
  });
  wrk = { x: 0, v: 0, done: false, ke_history: [], data: [], _computed: {} };
  updateWork();
}

/* ------------------------------------------------------------
   HELPER — cosine of theta (defaults to 1 = theta=0)
   ------------------------------------------------------------ */
function cosTh(s) {
  return (s.theta !== null && s.theta !== undefined)
    ? Math.cos(s.theta * Math.PI / 180)
    : 1.0;
}

/* ------------------------------------------------------------
   EQUATIONS REGISTRY — 13 equations, 20+ solve targets
   Each equation carries its own buildSub() for formatted output.
   ------------------------------------------------------------ */
var WORK_EQUATIONS = [

  /* 1. Work: W = F*d*cos(θ) */
  { name: 'Work', formula: 'W = F × d × cos(θ)',
    solve: {
      W: function(s) {
        if (s.F === null || s.d === null) return null;
        return s.F * s.d * cosTh(s);
      },
      F: function(s) {
        if (s.W === null || s.d === null) return null;
        var ct = cosTh(s);
        if (Math.abs(s.d * ct) < 1e-12) return null;
        return s.W / (s.d * ct);
      },
      d: function(s) {
        if (s.W === null || s.F === null) return null;
        var ct = cosTh(s);
        if (Math.abs(s.F * ct) < 1e-12) return null;
        return s.W / (s.F * ct);
      },
      theta: function(s) {
        if (s.W === null || s.F === null || s.d === null) return null;
        if (Math.abs(s.F * s.d) < 1e-12) return null;
        var r = s.W / (s.F * s.d);
        if (r < -1 || r > 1) return null;
        return Math.acos(r) * 180 / Math.PI;
      }
    },
    buildSub: function(s) {
      switch (s._last) {
        case 'W':     return 'W = ' + fvN(s.F) + ' × ' + fvN(s.d) + ' × cos(' + fvN(s.theta || 0, 1) + '°) = ' + fmtSci(s.W) + ' J';
        case 'F':     return 'F = ' + fmtSci(s.W) + ' / (' + fvN(s.d) + ' × cos(' + fvN(s.theta || 0, 1) + '°)) = ' + fmtSci(s.F) + ' N';
        case 'd':     return 'd = ' + fmtSci(s.W) + ' / (' + fvN(s.F) + ' × cos(' + fvN(s.theta || 0, 1) + '°)) = ' + fmtSci(s.d) + ' m';
        case 'theta': return 'θ = arccos(' + fmtSci(s.W) + ' / (' + fvN(s.F) + ' × ' + fvN(s.d) + ')) = ' + fmtSci(s.theta) + '°';
        default:      return 'W = F × d × cos(θ) = ' + fmtSci(s.W) + ' J';
      }
    }
  },

  /* 2. Kinetic Energy: KE = ½mv² */
  { name: 'Kinetic Energy', formula: 'KE = ½ × m × vf²',
    solve: {
      KE: function(s) {
        if (s.m === null || s.vf === null) return null;
        return 0.5 * s.m * s.vf * s.vf;
      },
      vf: function(s) {
        if (s.KE === null || s.m === null || s.m <= 0 || s.KE < 0) return null;
        return Math.sqrt(2 * s.KE / s.m);
      },
      m: function(s) {
        if (s.KE === null || s.vf === null || Math.abs(s.vf) < 1e-12) return null;
        return 2 * s.KE / (s.vf * s.vf);
      }
    },
    buildSub: function(s) {
      return 'KE = ½ × ' + fvN(s.m) + ' × ' + fvN(s.vf) + '² = ' + fmtSci(s.KE) + ' J';
    }
  },

  /* 3. Initial KE: KEi = ½mvi² */
  { name: 'Initial KE', formula: 'KEi = ½ × m × vi²',
    solve: {
      KEi: function(s) {
        if (s.m === null || s.vi === null) return null;
        return 0.5 * s.m * s.vi * s.vi;
      },
      vi: function(s) {
        if (s.KEi === null || s.m === null || s.m <= 0 || s.KEi < 0) return null;
        return Math.sqrt(2 * s.KEi / s.m);
      },
      m: function(s) {
        if (s.KEi === null || s.vi === null || Math.abs(s.vi) < 1e-12) return null;
        return 2 * s.KEi / (s.vi * s.vi);
      }
    },
    buildSub: function(s) {
      return 'KEi = ½ × ' + fvN(s.m) + ' × ' + fvN(s.vi) + '² = ' + fmtSci(s.KEi) + ' J';
    }
  },

  /* 4. Work-Energy Theorem: W = ½m(vf² - vi²) */
  { name: 'Work-Energy Theorem', formula: 'W = ½ × m × (vf² - vi²)',
    solve: {
      W: function(s) {
        if (s.m === null || s.vf === null || s.vi === null) return null;
        return 0.5 * s.m * (s.vf * s.vf - s.vi * s.vi);
      },
      vf: function(s) {
        if (s.W === null || s.m === null || s.vi === null || s.m <= 0) return null;
        var disc = s.vi * s.vi + 2 * s.W / s.m;
        if (disc < 0) return null;
        return Math.sqrt(disc);
      },
      vi: function(s) {
        if (s.W === null || s.m === null || s.vf === null || s.m <= 0) return null;
        var disc = s.vf * s.vf - 2 * s.W / s.m;
        if (disc < 0) return null;
        return Math.sqrt(disc);
      },
      m: function(s) {
        if (s.W === null || s.vf === null || s.vi === null) return null;
        var dv2 = s.vf * s.vf - s.vi * s.vi;
        if (Math.abs(dv2) < 1e-12) return null;
        return 2 * s.W / dv2;
      }
    },
    buildSub: function(s) {
      return 'W = ½ × ' + fvN(s.m) + ' × (' + fvN(s.vf) + '² - ' + fvN(s.vi) + '²) = ' + fmtSci(s.W) + ' J';
    }
  },

  /* 5. W = ΔKE */
  { name: 'W = ΔKE', formula: 'W = KE - KEi',
    solve: {
      W:   function(s) { if (s.KE === null || s.KEi === null) return null; return s.KE - s.KEi; },
      KE:  function(s) { if (s.W  === null || s.KEi === null) return null; return s.W + s.KEi; },
      KEi: function(s) { if (s.W  === null || s.KE  === null) return null; return s.KE - s.W; }
    },
    buildSub: function(s) {
      return 'W = ' + fmtSci(s.KE) + ' - ' + fmtSci(s.KEi) + ' = ' + fmtSci(s.W) + ' J';
    }
  },

  /* 6. Potential Energy: PE = mgh */
  { name: 'Potential Energy', formula: 'PE = m × g × h',
    solve: {
      PE: function(s) {
        if (s.m === null || s.h === null) return null;
        return s.m * s.g * s.h;
      },
      h: function(s) {
        if (s.PE === null || s.m === null || Math.abs(s.m * s.g) < 1e-12) return null;
        return s.PE / (s.m * s.g);
      },
      m: function(s) {
        if (s.PE === null || s.h === null || Math.abs(s.g * s.h) < 1e-12) return null;
        return s.PE / (s.g * s.h);
      },
      g: function(s) {
        if (s.PE === null || s.m === null || s.h === null || Math.abs(s.m * s.h) < 1e-12) return null;
        return s.PE / (s.m * s.h);
      }
    },
    buildSub: function(s) {
      return 'PE = ' + fvN(s.m) + ' × ' + fvN(s.g) + ' × ' + fvN(s.h) + ' = ' + fmtSci(s.PE) + ' J';
    }
  },

  /* 7. Mechanical Energy: E = KE + PE */
  { name: 'Mechanical Energy', formula: 'E = KE + PE',
    solve: {
      E:  function(s) { if (s.KE === null || s.PE === null) return null; return s.KE + s.PE; },
      KE: function(s) { if (s.E  === null || s.PE === null) return null; return s.E - s.PE; },
      PE: function(s) { if (s.E  === null || s.KE === null) return null; return s.E - s.KE; }
    },
    buildSub: function(s) {
      return 'E = ' + fmtSci(s.KE) + ' + ' + fmtSci(s.PE) + ' = ' + fmtSci(s.E) + ' J';
    }
  },

  /* 8. Energy Conservation: KEi + PEi + W = KE + PE */
  { name: 'Energy Conservation', formula: 'KEi + PEi + W = KE + PE',
    solve: {
      W: function(s) {
        if (s.KE === null || s.PE === null || s.KEi === null || s.PEi === null) return null;
        return (s.KE + s.PE) - (s.KEi + (s.PEi || 0));
      },
      KE: function(s) {
        if (s.W === null || s.PE === null || s.KEi === null) return null;
        return s.KEi + (s.PEi || 0) + s.W - s.PE;
      }
    },
    buildSub: function(s) {
      return '(' + fmtSci(s.KEi) + ' + ' + fmtSci(s.PEi || 0) + ') + ' + fmtSci(s.W) + ' = ' + fmtSci(s.KE) + ' + ' + fmtSci(s.PE);
    }
  },

  /* 9. Power: P = W / t */
  { name: 'Power', formula: 'P = W / t',
    solve: {
      P: function(s) {
        if (s.W === null || s.t === null || Math.abs(s.t) < 1e-12) return null;
        return s.W / s.t;
      },
      W: function(s) {
        if (s.P === null || s.t === null) return null;
        return s.P * s.t;
      },
      t: function(s) {
        if (s.W === null || s.P === null || Math.abs(s.P) < 1e-12) return null;
        return s.W / s.P;
      }
    },
    buildSub: function(s) {
      return 'P = ' + fmtSci(s.W) + ' / ' + fvN(s.t) + ' = ' + fmtSci(s.P) + ' W';
    }
  },

  /* 10. Newton's 2nd Law: F = ma */
  { name: "Newton's 2nd Law", formula: 'F = m × a',
    solve: {
      F: function(s) {
        if (s.m === null || s.a === null) return null;
        return s.m * s.a;
      },
      a: function(s) {
        if (s.F === null || s.m === null || s.m <= 0) return null;
        return s.F * cosTh(s) / s.m;
      },
      m: function(s) {
        if (s.F === null || s.a === null || Math.abs(s.a) < 1e-12) return null;
        return s.F / s.a;
      }
    },
    buildSub: function(s) {
      return 'a = ' + fmtSci(s.F) + ' / ' + fvN(s.m) + ' = ' + fmtSci(s.a) + ' m/s²';
    }
  },

  /* 11. Kinematics: vf² = vi² + 2ad */
  { name: 'Kinematics', formula: 'vf² = vi² + 2 × a × d',
    solve: {
      vf: function(s) {
        if (s.vi === null || s.a === null || s.d === null) return null;
        var r = s.vi * s.vi + 2 * s.a * s.d;
        if (r < 0) return null;
        return Math.sqrt(r);
      },
      vi: function(s) {
        if (s.vf === null || s.a === null || s.d === null) return null;
        var r = s.vf * s.vf - 2 * s.a * s.d;
        if (r < 0) return null;
        return Math.sqrt(r);
      },
      a: function(s) {
        if (s.vf === null || s.vi === null || s.d === null || Math.abs(s.d) < 1e-12) return null;
        return (s.vf * s.vf - s.vi * s.vi) / (2 * s.d);
      },
      d: function(s) {
        if (s.vf === null || s.vi === null || s.a === null || Math.abs(s.a) < 1e-12) return null;
        return (s.vf * s.vf - s.vi * s.vi) / (2 * s.a);
      }
    },
    buildSub: function(s) {
      return 'vf = √(' + fvN(s.vi) + '² + 2×' + fvN(s.a, 2) + '×' + fvN(s.d) + ') = ' + fmtSci(s.vf) + ' m/s';
    }
  },

  /* 12. Friction Work: Wf = -μmgd */
  { name: 'Friction Work', formula: 'Wf = −μ × m × g × d',
    solve: {
      Wf: function(s) {
        if (s.mu === null || s.m === null || s.d === null) return null;
        return -(s.mu * s.m * s.g * s.d);
      },
      mu: function(s) {
        if (s.Wf === null || s.m === null || s.d === null) return null;
        var denom = s.m * s.g * s.d;
        if (Math.abs(denom) < 1e-12) return null;
        return -s.Wf / denom;
      },
      d: function(s) {
        if (s.Wf === null || s.mu === null || s.m === null) return null;
        var denom = s.mu * s.m * s.g;
        if (Math.abs(denom) < 1e-12) return null;
        return -s.Wf / denom;
      }
    },
    buildSub: function(s) {
      return 'Wf = −' + fvN(s.mu, 3) + ' × ' + fvN(s.m) + ' × ' + fvN(s.g) + ' × ' + fvN(s.d) + ' = ' + fmtSci(s.Wf) + ' J';
    }
  },

  /* 13. Net Work: Wnet = W + Wf */
  { name: 'Net Work', formula: 'Wnet = W + Wf',
    solve: {
      Wnet: function(s) {
        if (s.W === null) return null;
        return s.W + (s.Wf !== null ? s.Wf : 0);
      },
      W: function(s) {
        if (s.Wnet === null || s.Wf === null) return null;
        return s.Wnet - s.Wf;
      },
      Wf: function(s) {
        if (s.Wnet === null || s.W === null) return null;
        return s.Wnet - s.W;
      }
    },
    buildSub: function(s) {
      return 'Wnet = ' + fmtSci(s.W) + ' + ' + fmtSci(s.Wf || 0) + ' = ' + fmtSci(s.Wnet) + ' J';
    }
  }
];

/* Metadata: maps variable key → { unit, label, cardId, valId } */
var WORK_META = {
  W:    { unit:'J',     label:'Work (W)',     card:'wrc-W',     val:'work-r-W'    },
  P:    { unit:'W',     label:'Power (P)',    card:'wrc-P',     val:'work-r-P'    },
  KE:   { unit:'J',     label:'KE',          card:'wrc-KE',    val:'work-r-KE'   },
  KEi:  { unit:'J',     label:'KEᵢ',         card:'wrc-KEi',   val:'work-r-KEi'  },
  PE:   { unit:'J',     label:'PE',          card:'wrc-PE',    val:'work-r-PE'   },
  E:    { unit:'J',     label:'Total E',     card:'wrc-E',     val:'work-r-E'    },
  vf:   { unit:'m/s',   label:'vf',          card:'wrc-vf',    val:'work-r-vf'   },
  vi:   { unit:'m/s',   label:'vi',          card:'wrc-vi',    val:'work-r-vi'   },
  F:    { unit:'N',     label:'Force (F)',   card:'wrc-F',     val:'work-r-F'    },
  d:    { unit:'m',     label:'Distance (d)',card:'wrc-d',     val:'work-r-d'    },
  h:    { unit:'m',     label:'Height (h)',  card:'wrc-h',     val:'work-r-h'    },
  a:    { unit:'m/s²',  label:'Accel (a)',   card:'wrc-a',     val:'work-r-a'    },
  theta:{ unit:'deg',   label:'Angle (θ)',   card:'wrc-theta', val:'work-r-theta'},
  Wf:   { unit:'J',     label:'W_friction',  card:'wrc-Wf',    val:'work-r-Wf'   },
  Wnet: { unit:'J',     label:'W_net',       card:'wrc-Wnet',  val:'work-r-Wnet' },
  mu:   { unit:'',      label:'Friction μ',  card:'wrc-mu',    val:'work-r-mu'   },
  t:    { unit:'s',     label:'Time (t)',    card:'wrc-t',     val:'work-r-t'    },
  m:    { unit:'kg',    label:'Mass (m)',    card:'wrc-m',     val:'work-r-m'    },
  g:    { unit:'m/s²',  label:'Gravity (g)', card:'wrc-g',     val:'work-r-g'    }
};

/* ------------------------------------------------------------
   UPDATE (solver + UI)
   ------------------------------------------------------------ */
function updateWork() {
  /* 1. Read all inputs */
  var state = {
    m:     getNullable('work-m'),
    F:     getNullable('work-F'),
    theta: getNullable('work-angle'),
    d:     getNullable('work-d'),
    vi:    getNullable('work-vi'),
    vf:    getNullable('work-vf'),
    W:     getNullable('work-W'),
    t:     getNullable('work-t'),
    h:     getNullable('work-h'),
    KE:    getNullable('work-KE'),
    P:     getNullable('work-P'),
    mu:    getNullable('work-mu'),
    g:     getNullable('work-g') || 9.81,
    /* solver-only: */
    PE: null, E: null, KEi: null, PEi: null,
    a: null, Wf: null, Wnet: null
  };

  /* 2. Input validation */
  showInputError('work-m',   validateInput(state.m,   { positive: true, label: 'm' }));
  showInputError('work-g',   validateInput(state.g,   { positive: true, label: 'g' }));
  showInputError('work-mu',  validateInput(state.mu,  { min: 0, max: 2,  label: 'μ' }));
  showInputError('work-angle', validateInput(state.theta, { min: -90, max: 90, label: 'θ' }));

  /* 3. Track user-provided keys */
  var userKeys = {};
  Object.keys(state).forEach(function(k){ if (state[k] !== null) userKeys[k] = true; });
  var solveFor = document.getElementById('work-solveFor').value;

  /* 4. Run propagation solver */
  var result = runSolver(state, WORK_EQUATIONS, 20);
  var log     = result.log;
  var derived = result.derived;

  /* 5. Update all result cards */
  Object.keys(WORK_META).forEach(function(k) {
    var meta  = WORK_META[k];
    var valEl = document.getElementById(meta.val);
    if (!valEl) return;
    var val = state[k];
    valEl.textContent = (val !== null && val !== undefined)
      ? fmtSci(val) + (meta.unit ? ' ' + meta.unit : '')
      : '--';
    var st = (k === solveFor) ? 'target'
           : (userKeys[k]    ? 'user'
           : (val !== null   ? 'derived' : ''));
    styleCard(meta.card, st);
  });

  /* 6. Solver steps panel */
  renderSolverSteps('work-solver-steps', log, derived, state, solveFor,
    'Enter any known variables (leave unknowns blank).\nThe solver chains up to 13 equations automatically.');

  /* 7. Energy bars */
  var keV = state.KE, peV = state.PE, wV = state.W;
  var maxE = Math.max(Math.abs(keV || 0), Math.abs(peV || 0), Math.abs(wV || 0), 1);
  _wrkBar('work-ke-bar', 'work-ke-val', keV, maxE);
  _wrkBar('work-pe-bar', 'work-pe-val', peV, maxE);
  _wrkBar('work-W-bar',  'work-W-val',  wV,  maxE);

  /* 8. Pass solved values to animation engine */
  wrk._computed = {
    m:     state.m     || 10,
    F:     state.F     || 0,
    theta: state.theta || 0,
    d:     state.d     || 20,
    vi:    state.vi    || 0,
    mu:    state.mu    || 0,
    g:     state.g     || 9.81
  };

  if (!simRunning) drawWork();
}

function _wrkBar(barId, lblId, val, maxE) {
  var bar = document.getElementById(barId);
  var lbl = document.getElementById(lblId);
  if (!bar || !lbl) return;
  if (val === null || val === undefined) {
    bar.style.width = '0%'; lbl.textContent = '-- J'; return;
  }
  bar.style.width = Math.min(Math.abs(val) / maxE * 100, 100) + '%';
  lbl.textContent = fmtSci(val) + ' J';
}

/* ------------------------------------------------------------
   STEP
   
   Position is stored as a fraction of the track (0.0 = left,
   1.0 = right end at 85%).  Velocity is in physical m/s, converted
   to track-fraction/s for position updates using the distance d.
   ------------------------------------------------------------ */
function stepWork(dt) {
  var c  = wrk._computed || {};
  var m  = c.m  || 10;
  var F  = c.F  || 0;
  var th = (c.theta || 0) * Math.PI / 180;
  var d  = c.d  || 20;   /* track length in metres */
  var vi = c.vi || 0;
  var mu = c.mu || 0;
  var g  = c.g  || 9.81;

  /* Initialise on first frame */
  if (simTime <= dt + 0.001) {
    wrk.v      = vi;           /* physical velocity in m/s   */
    wrk.x      = 0;            /* position in metres         */
    wrk.done   = false;
    wrk.ke_history = [];
    wrk.data   = [];
  }

  if (wrk.done) { stopSim(); return; }

  /* Net force and acceleration — physics equations unchanged */
  var Fnet = F * Math.cos(th) - mu * m * g;
  var a    = (m > 0) ? Fnet / m : 0;

  /* Sub-stepped Euler for smoother motion */
  var SUBSTEPS = 4;
  var dts = dt / SUBSTEPS;
  for (var si = 0; si < SUBSTEPS; si++) {
    wrk.v += a * dts;
    wrk.x += wrk.v * dts;

    /* Wall clamps: snap to boundary and zero velocity */
    if (wrk.x <= 0 && wrk.v < 0) {
      wrk.x = 0;
      wrk.v = 0;
      wrk.done = true;
      break;
    }
    if (wrk.x >= d * 0.85 && wrk.v > 0) {
      wrk.x = d * 0.85;
      wrk.v = 0;
      wrk.done = true;
      break;
    }
  }

  /* Kill micro-crawl */
  if (Math.abs(wrk.v) < 0.001) wrk.v = 0;

  /* KE history for the graph */
  var ke = 0.5 * m * wrk.v * wrk.v;
  wrk.ke_history.push(ke);
  if (wrk.ke_history.length > 200) wrk.ke_history.shift();

  /* Record data for CSV — x in metres, v in m/s */
  wrk.data.push([
    parseFloat(simTime.toFixed(3)),
    parseFloat(wrk.x.toFixed(3)),
    parseFloat(wrk.v.toFixed(3)),
    parseFloat(ke.toFixed(3))
  ]);

  updateInfoBar(simTime, wrk.x, 0, Math.abs(wrk.v));
}

/* ------------------------------------------------------------
   DRAW
   ------------------------------------------------------------ */
function drawWork() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var c = wrk._computed || {};
  var m = c.m || 10, F = c.F || 0;
  var th_deg = c.theta || 0, mu = c.mu || 0;
  var d = c.d || 20, g = c.g || 9.81;
  var h = getNullable('work-h') || 0;

  var surfY  = H * 0.62;
  var margin = 80;
  var trackW = W - margin * 2;

  /* Surface */
  var surfGrad = ctx.createLinearGradient(0, surfY, 0, surfY + 12);
  surfGrad.addColorStop(0, 'rgba(0,212,255,0.5)');
  surfGrad.addColorStop(1, 'rgba(0,212,255,0.1)');
  ctx.fillStyle = surfGrad; ctx.fillRect(margin, surfY, trackW, 12);

  /* Friction overlay */
  if (mu > 0) {
    ctx.fillStyle = 'rgba(255,107,53,' + Math.min(mu * 0.3, 0.5) + ')';
    ctx.fillRect(margin, surfY, trackW, 12);
    ctx.fillStyle = '#ff6b35'; ctx.font = '11px Space Mono, monospace';
    ctx.fillText('μ = ' + mu, margin + 8, surfY + 26);
  }

  /* Distance markers */
  ctx.strokeStyle = 'rgba(107,122,153,0.3)'; ctx.lineWidth = 1;
  ctx.fillStyle   = 'rgba(107,122,153,0.6)'; ctx.font = '10px Space Mono';
  ctx.textAlign   = 'center';
  for (var i = 0; i <= 10; i++) {
    var mx = margin + trackW * i / 10;
    ctx.beginPath(); ctx.moveTo(mx, surfY); ctx.lineTo(mx, surfY - 8); ctx.stroke();
    ctx.fillText(parseFloat((d * i / 10).toFixed(1)) + 'm', mx, surfY - 12);
  }
  ctx.textAlign = 'left';

  /* Block — x is in metres, convert to canvas pixels */
  var trackFrac = (d > 0) ? Math.min(wrk.x / d, 1) : 0;
  var bx = margin + trackFrac * trackW;
  var bw = 58, bh = 44;
  var by = surfY - bh;

  ctx.shadowColor = '#00d4ff'; ctx.shadowBlur = 18;
  var bGrad = ctx.createLinearGradient(bx - bw / 2, by, bx - bw / 2, by + bh);
  bGrad.addColorStop(0, 'rgba(0,212,255,0.9)');
  bGrad.addColorStop(1, 'rgba(0,60,90,0.7)');
  ctx.fillStyle = bGrad; ctx.fillRect(bx - bw / 2, by, bw, bh);
  ctx.shadowBlur = 0;
  ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 2;
  ctx.strokeRect(bx - bw / 2, by, bw, bh);
  ctx.fillStyle = '#000'; ctx.font = 'bold 12px DM Sans';
  ctx.textAlign = 'center'; ctx.fillText(m + 'kg', bx, by + bh / 2 + 5); ctx.textAlign = 'left';

  /* Applied Force arrow */
  if (F > 0) {
    var th_rad  = th_deg * Math.PI / 180;
    var arrowLen = Math.min(F * 0.8, 90);
    drawArrow(ctx, bx, by + bh / 2,
      bx + arrowLen * Math.cos(th_rad),
      by + bh / 2 - arrowLen * Math.sin(th_rad),
      '#ff3e8a', 'F=' + fmtSci(F) + 'N', 2.5);
  }

  /* Friction arrow */
  if (mu > 0 && m > 0 && simRunning) {
    var Ff   = mu * m * g;
    var frLen = Math.min(Ff * 0.8, 70);
    drawArrow(ctx, bx, by + bh - 6, bx - frLen, by + bh - 6, '#ff6b35', 'Ff=' + fmtSci(Ff) + 'N', 1.5);
  }

  /* Height indicator */
  if (h > 0) {
    ctx.strokeStyle = 'rgba(168,255,62,0.4)';
    ctx.lineWidth = 1.5; ctx.setLineDash([3, 4]);
    var hPx = h * 8;
    ctx.beginPath();
    ctx.moveTo(bx + bw / 2 + 16, surfY);
    ctx.lineTo(bx + bw / 2 + 16, surfY - hPx);
    ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono';
    ctx.fillText('h=' + h + 'm', bx + bw / 2 + 22, surfY - hPx / 2);
  }

  /* Velocity readout — wrk.v is now in physical m/s */
  ctx.fillStyle = '#a8ff3e'; ctx.font = 'bold 13px Space Mono, monospace';
  ctx.fillText('v = ' + fmt(Math.abs(wrk.v), 2) + ' m/s', margin + trackW - 155, surfY - bh - 14);

  /* KE history graph */
  if (wrk.ke_history.length > 2) {
    var gx = margin, gy = H * 0.78, gw = trackW, gh = H * 0.13;
    ctx.fillStyle = 'rgba(10,13,20,0.75)'; ctx.fillRect(gx, gy, gw, gh);
    ctx.strokeStyle = 'rgba(30,42,66,0.8)'; ctx.lineWidth = 1; ctx.strokeRect(gx, gy, gw, gh);

    var maxKE = Math.max.apply(null, wrk.ke_history.concat([1]));
    ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 2;
    ctx.beginPath();
    wrk.ke_history.forEach(function(ke, i) {
      var kx = gx + (i / (wrk.ke_history.length - 1)) * gw;
      var ky = gy + gh - (ke / maxKE) * (gh - 6);
      if (i === 0) ctx.moveTo(kx, ky); else ctx.lineTo(kx, ky);
    });
    ctx.stroke();
    ctx.fillStyle = 'rgba(107,122,153,0.7)'; ctx.font = '10px Space Mono';
    ctx.fillText('KE over time', gx + 6, gy + 13);
    ctx.fillText('max: ' + fmtSci(maxKE) + ' J', gx + gw - 100, gy + 13);
  }
}

/* ------------------------------------------------------------
   CSV EXPORT
   ------------------------------------------------------------ */
function exportWorkCSV() {
  if (!wrk.data || wrk.data.length === 0) {
    alert('Run the simulation first.'); return;
  }
  exportCSV('work_energy_data.csv',
    ['time_s','x_m','v_ms','KE_J'],
    wrk.data);
}
