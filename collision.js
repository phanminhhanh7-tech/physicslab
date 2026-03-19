/* ============================================================
   collision.js — Collision Simulation
   Handles: elastic + inelastic collisions, solver, animation
   Physics assumptions: 1D head-on, no friction, point masses
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   STATE
   ------------------------------------------------------------ */
var col = {
  b1: { x: 0.15, v: 0 },  // ball 1: normalized position, velocity
  b2: { x: 0.75, v: 0 },  // ball 2
  collided: false,
  phase: 'pre',            // 'pre' | 'post'
  data: [],                // [{t, x1, v1, x2, v2, KE}] for CSV
  // Solver cache
  _m1: 5, _m2: 3, _v1: 8, _v2: -3
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetCollision() {
  col = {
    b1: { x: 0.15, v: 0 }, b2: { x: 0.75, v: 0 },
    collided: false, phase: 'pre', data: [],
    _m1: col._m1, _m2: col._m2, _v1: col._v1, _v2: col._v2
  };
  ['col-m1','col-m2','col-v1','col-v2','col-v1a','col-v2a',
   'col-KEb','col-KEa','col-KElost','col-pt'].forEach(function(id) {
    var el = document.getElementById(id);
    if (el) el.value = '';
  });
  updateCollision();
}

/* ------------------------------------------------------------
   EQUATIONS REGISTRY
   ------------------------------------------------------------ */
var COL_EQUATIONS = [

  /* Momentum before: p1b = m1 * v1 */
  { name: 'Momentum 1 (before)', formula: 'p1 = m1 × v1',
    solve: {
      p1b: function(s) {
        if (s.m1 === null || s.v1 === null) return null;
        return s.m1 * s.v1;
      },
      m1: function(s) {
        if (s.p1b === null || s.v1 === null || Math.abs(s.v1) < 1e-12) return null;
        return s.p1b / s.v1;
      },
      v1: function(s) {
        if (s.p1b === null || s.m1 === null || Math.abs(s.m1) < 1e-12) return null;
        return s.p1b / s.m1;
      }
    },
    buildSub: function(s) {
      return 'p1 = ' + fvN(s.m1) + ' × ' + fvN(s.v1) + ' = ' + fmtSci(s.p1b) + ' kg·m/s';
    }
  },

  /* Momentum before: p2b = m2 * v2 */
  { name: 'Momentum 2 (before)', formula: 'p2 = m2 × v2',
    solve: {
      p2b: function(s) {
        if (s.m2 === null || s.v2 === null) return null;
        return s.m2 * s.v2;
      },
      m2: function(s) {
        if (s.p2b === null || s.v2 === null || Math.abs(s.v2) < 1e-12) return null;
        return s.p2b / s.v2;
      },
      v2: function(s) {
        if (s.p2b === null || s.m2 === null || Math.abs(s.m2) < 1e-12) return null;
        return s.p2b / s.m2;
      }
    },
    buildSub: function(s) {
      return 'p2 = ' + fvN(s.m2) + ' × ' + fvN(s.v2) + ' = ' + fmtSci(s.p2b) + ' kg·m/s';
    }
  },

  /* Total momentum: pt = p1b + p2b */
  { name: 'Total momentum', formula: 'pt = p1 + p2',
    solve: {
      pt: function(s) {
        if (s.p1b === null || s.p2b === null) return null;
        return s.p1b + s.p2b;
      },
      p1b: function(s) {
        if (s.pt === null || s.p2b === null) return null;
        return s.pt - s.p2b;
      },
      p2b: function(s) {
        if (s.pt === null || s.p1b === null) return null;
        return s.pt - s.p1b;
      }
    },
    buildSub: function(s) {
      return 'pt = ' + fmtSci(s.p1b) + ' + ' + fmtSci(s.p2b) + ' = ' + fmtSci(s.pt) + ' kg·m/s';
    }
  },

  /* KE before: KEb = ½m1v1² + ½m2v2² */
  { name: 'KE before collision', formula: 'KEb = ½m1v1² + ½m2v2²',
    solve: {
      KEb: function(s) {
        if (s.m1 === null || s.m2 === null || s.v1 === null || s.v2 === null) return null;
        return 0.5 * s.m1 * s.v1 * s.v1 + 0.5 * s.m2 * s.v2 * s.v2;
      }
    },
    buildSub: function(s) {
      return 'KEb = ½×' + fvN(s.m1) + '×' + fvN(s.v1) + '² + ½×' + fvN(s.m2) + '×' + fvN(s.v2) + '² = ' + fmtSci(s.KEb) + ' J';
    }
  },

  /* Elastic collision — v1 after */
  { name: 'Elastic v1 after', formula: 'v1a = [(m1-m2)v1 + 2m2v2] / (m1+m2)',
    solve: {
      v1a: function(s) {
        if (s.m1 === null || s.m2 === null || s.v1 === null || s.v2 === null) return null;
        if (s._type !== 'elastic') return null;
        var mt = s.m1 + s.m2;
        if (Math.abs(mt) < 1e-12) return null;
        return ((s.m1 - s.m2) * s.v1 + 2 * s.m2 * s.v2) / mt;
      }
    },
    buildSub: function(s) {
      var mt = (s.m1 || 0) + (s.m2 || 0);
      return 'v1a = [(' + fvN(s.m1) + '-' + fvN(s.m2) + ')×' + fvN(s.v1) + ' + 2×' + fvN(s.m2) + '×' + fvN(s.v2) + '] / ' + fvN(mt) + ' = ' + fmtSci(s.v1a) + ' m/s';
    }
  },

  /* Elastic collision — v2 after */
  { name: 'Elastic v2 after', formula: 'v2a = [2m1v1 + (m2-m1)v2] / (m1+m2)',
    solve: {
      v2a: function(s) {
        if (s.m1 === null || s.m2 === null || s.v1 === null || s.v2 === null) return null;
        if (s._type !== 'elastic') return null;
        var mt = s.m1 + s.m2;
        if (Math.abs(mt) < 1e-12) return null;
        return (2 * s.m1 * s.v1 + (s.m2 - s.m1) * s.v2) / mt;
      }
    },
    buildSub: function(s) {
      var mt = (s.m1 || 0) + (s.m2 || 0);
      return 'v2a = [2×' + fvN(s.m1) + '×' + fvN(s.v1) + ' + (' + fvN(s.m2) + '-' + fvN(s.m1) + ')×' + fvN(s.v2) + '] / ' + fvN(mt) + ' = ' + fmtSci(s.v2a) + ' m/s';
    }
  },

  /* Perfectly inelastic — shared final velocity */
  { name: 'Inelastic vf', formula: 'vf = (m1v1 + m2v2) / (m1+m2)',
    solve: {
      v1a: function(s) {
        if (s.m1 === null || s.m2 === null || s.pt === null) return null;
        if (s._type !== 'inelastic') return null;
        var mt = s.m1 + s.m2;
        if (Math.abs(mt) < 1e-12) return null;
        return s.pt / mt;
      },
      v2a: function(s) {
        if (s.m1 === null || s.m2 === null || s.pt === null) return null;
        if (s._type !== 'inelastic') return null;
        var mt = s.m1 + s.m2;
        if (Math.abs(mt) < 1e-12) return null;
        return s.pt / mt;
      }
    },
    buildSub: function(s) {
      var mt = (s.m1 || 0) + (s.m2 || 0);
      return 'vf = ' + fmtSci(s.pt) + ' / ' + fvN(mt) + ' = ' + fmtSci(s.v1a) + ' m/s';
    }
  },

  /* KE after: KEa = ½m1v1a² + ½m2v2a² */
  { name: 'KE after collision', formula: 'KEa = ½m1v1a² + ½m2v2a²',
    solve: {
      KEa: function(s) {
        if (s.m1 === null || s.m2 === null || s.v1a === null || s.v2a === null) return null;
        return 0.5 * s.m1 * s.v1a * s.v1a + 0.5 * s.m2 * s.v2a * s.v2a;
      }
    },
    buildSub: function(s) {
      return 'KEa = ½×' + fvN(s.m1) + '×' + fvN(s.v1a) + '² + ½×' + fvN(s.m2) + '×' + fvN(s.v2a) + '² = ' + fmtSci(s.KEa) + ' J';
    }
  },

  /* KE lost: KElost = KEb - KEa */
  { name: 'KE lost', formula: 'KElost = KEb - KEa',
    solve: {
      KElost: function(s) {
        if (s.KEb === null || s.KEa === null) return null;
        return s.KEb - s.KEa;
      },
      KEb: function(s) {
        if (s.KElost === null || s.KEa === null) return null;
        return s.KEa + s.KElost;
      },
      KEa: function(s) {
        if (s.KElost === null || s.KEb === null) return null;
        return s.KEb - s.KElost;
      }
    },
    buildSub: function(s) {
      return 'KElost = ' + fmtSci(s.KEb) + ' - ' + fmtSci(s.KEa) + ' = ' + fmtSci(s.KElost) + ' J';
    }
  },

  /* Back-solve m1 from elastic post-collision v1a, v2a, v2 */
  { name: 'Elastic mass from v after', formula: 'm1 from elastic v1a, v2a',
    solve: {
      m1: function(s) {
        if (s.v1a === null || s.v2a === null || s.v1 === null || s.v2 === null) return null;
        if (s._type !== 'elastic') return null;
        // v2a = 2m1v1/(m1+m2) + (m2-m1)v2/(m1+m2) — solve for m1 given m2
        // From momentum: m1v1 + m2v2 = m1v1a + m2v2a
        // => m1(v1 - v1a) = m2(v2a - v2)
        if (s.m2 === null) return null;
        var dv1 = s.v1 - s.v1a;
        if (Math.abs(dv1) < 1e-12) return null;
        return s.m2 * (s.v2a - s.v2) / dv1;
      },
      m2: function(s) {
        if (s.v1a === null || s.v2a === null || s.v1 === null || s.v2 === null) return null;
        if (s._type !== 'elastic') return null;
        if (s.m1 === null) return null;
        var dv2 = s.v2a - s.v2;
        if (Math.abs(dv2) < 1e-12) return null;
        return s.m1 * (s.v1 - s.v1a) / dv2;
      }
    },
    buildSub: function(s) {
      return 'm1(v1-v1a) = m2(v2a-v2) → m1 = ' + fmtSci(s.m1) + ' kg';
    }
  },

  /* Back-solve v1 from total momentum + v2 + masses */
  { name: 'v1 from momentum', formula: 'v1 = (pt - m2*v2) / m1',
    solve: {
      v1: function(s) {
        if (s.pt === null || s.m1 === null || s.m2 === null || s.v2 === null) return null;
        if (Math.abs(s.m1) < 1e-12) return null;
        return (s.pt - s.m2 * s.v2) / s.m1;
      },
      v2: function(s) {
        if (s.pt === null || s.m1 === null || s.m2 === null || s.v1 === null) return null;
        if (Math.abs(s.m2) < 1e-12) return null;
        return (s.pt - s.m1 * s.v1) / s.m2;
      }
    },
    buildSub: function(s) {
      return 'v1 = (' + fmtSci(s.pt) + ' - ' + fvN(s.m2) + '×' + fvN(s.v2) + ') / ' + fvN(s.m1) + ' = ' + fmtSci(s.v1) + ' m/s';
    }
  }
];

/* Metadata */
var COL_META = {
  v1a:    { label:'v1 after',  unit:'m/s',    card:'crc-v1a',    val:'col-r-v1a' },
  v2a:    { label:'v2 after',  unit:'m/s',    card:'crc-v2a',    val:'col-r-v2a' },
  p1b:    { label:'p1 before', unit:'kg·m/s', card:'crc-p1b',    val:'col-r-p1b' },
  p2b:    { label:'p2 before', unit:'kg·m/s', card:'crc-p2b',    val:'col-r-p2b' },
  KEb:    { label:'KE before', unit:'J',      card:'crc-KEb',    val:'col-r-keb' },
  KEa:    { label:'KE after',  unit:'J',      card:'crc-KEa',    val:'col-r-kea' },
  pt:     { label:'p total',   unit:'kg·m/s', card:'crc-pt',     val:'col-r-pt'  },
  KElost: { label:'KE lost',   unit:'J',      card:'crc-KElost', val:'col-r-kel' }
};

/* ------------------------------------------------------------
   UPDATE (solver + UI)
   ------------------------------------------------------------ */
function updateCollision() {
  var type = document.getElementById('col-type').value;
  var s = {
    m1:     getNullable('col-m1'),
    m2:     getNullable('col-m2'),
    v1:     getNullable('col-v1'),
    v2:     getNullable('col-v2'),
    v1a:    getNullable('col-v1a'),
    v2a:    getNullable('col-v2a'),
    KEb:    getNullable('col-KEb'),
    KEa:    getNullable('col-KEa'),
    KElost: getNullable('col-KElost'),
    pt:     getNullable('col-pt'),
    p1b: null, p2b: null,
    _type: type
  };

  /* Input validation */
  showInputError('col-m1', validateInput(s.m1, { positive: true, label: 'm1' }));
  showInputError('col-m2', validateInput(s.m2, { positive: true, label: 'm2' }));

  var userKeys = {};
  Object.keys(s).forEach(function(k){ if (s[k] !== null && k !== '_type') userKeys[k] = true; });
  var solveFor = document.getElementById('col-solveFor').value;

  var result = runSolver(s, COL_EQUATIONS, 20);

  /* Update result cards */
  Object.keys(COL_META).forEach(function(k) {
    var m   = COL_META[k];
    var vel = document.getElementById(m.val);
    if (!vel) return;
    var val = s[k];
    vel.textContent = (val !== null && val !== undefined) ? fmtSci(val) + ' ' + m.unit : '--';
    var state = (k === solveFor) ? 'target' : (userKeys[k] ? 'user' : (val !== null ? 'derived' : ''));
    styleCard(m.card, state);
  });

  renderSolverSteps('col-solver-steps', result.log, result.derived, s, solveFor,
    'Enter m1, v1, m2, v2 above.\nSolver computes all collision outcomes.');

  /* Store for animation */
  col._m1   = (s.m1 !== null) ? s.m1 : 5;
  col._m2   = (s.m2 !== null) ? s.m2 : 3;
  col._v1   = (s.v1 !== null) ? s.v1 : 8;
  col._v2   = (s.v2 !== null) ? s.v2 : -3;
  col._type = type;

  if (!simRunning) {
    col.b1.v   = col._v1 * 0.05;
    col.b2.v   = col._v2 * 0.05;
    col.b1.x   = 0.15;
    col.b2.x   = 0.75;
    col.phase  = 'pre';
    col.collided = false;
    drawCollision();
  }
}

/* ------------------------------------------------------------
   STEP
   ------------------------------------------------------------ */
function stepCollision(dt) {
  var m1     = col._m1, m2 = col._m2;
  var v1init = col._v1, v2init = col._v2;
  var type   = col._type || 'elastic';

  if (simTime <= dt + 0.001) {
    col.b1 = { x: 0.12, v: v1init * 0.05 };
    col.b2 = { x: 0.78, v: v2init * 0.05 };
    col.collided = false; col.phase = 'pre';
    col.data = [];
  }

  var r1 = 0.07, r2 = 0.07;

  if (!col.collided) {
    col.b1.x += col.b1.v * dt;
    col.b2.x += col.b2.v * dt;

    /* Collision detection */
    if (Math.abs(col.b1.x - col.b2.x) <= r1 + r2) {
      col.collided = true; col.phase = 'post';
      var v1 = col.b1.v / 0.05, v2 = col.b2.v / 0.05;
      var mt = m1 + m2;
      var v1a, v2a;
      if (type === 'elastic') {
        v1a = ((m1 - m2) * v1 + 2 * m2 * v2) / mt;
        v2a = (2 * m1 * v1 + (m2 - m1) * v2) / mt;
      } else {
        var vf = (m1 * v1 + m2 * v2) / mt;
        v1a = vf; v2a = vf;
      }
      col.b1.v = v1a * 0.05;
      col.b2.v = v2a * 0.05;
    }
  } else {
    col.b1.x += col.b1.v * dt;
    col.b2.x += col.b2.v * dt;
  }

  /* Wall bounce */
  if (col.b1.x < r1)       { col.b1.x = r1;       col.b1.v *= -0.9; }
  if (col.b2.x > 1 - r2)   { col.b2.x = 1 - r2;   col.b2.v *= -0.9; }

  /* Record data */
  var v1a = col.b1.v / 0.05, v2a = col.b2.v / 0.05;
  var KE  = 0.5 * m1 * v1a * v1a + 0.5 * m2 * v2a * v2a;
  col.data.push([parseFloat(simTime.toFixed(3)),
    parseFloat(col.b1.x.toFixed(4)), parseFloat(v1a.toFixed(3)),
    parseFloat(col.b2.x.toFixed(4)), parseFloat(v2a.toFixed(3)),
    parseFloat(KE.toFixed(3))]);

  updateInfoBar(simTime, col.b1.x, 0, Math.abs(v1a));
}

/* ------------------------------------------------------------
   DRAW
   ------------------------------------------------------------ */
function drawCollision() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var m1 = col._m1 || 5, m2 = col._m2 || 3;
  var v1i = col._v1 || 8, v2i = col._v2 || -3;
  var type = col._type || 'elastic';

  var surfY  = H * 0.62;
  var margin = 60;
  var trackW = W - margin * 2;

  /* Surface */
  ctx.strokeStyle = 'rgba(0,212,255,0.4)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(margin, surfY); ctx.lineTo(W - margin, surfY); ctx.stroke();
  ctx.strokeStyle = 'rgba(0,212,255,0.15)'; ctx.lineWidth = 1;
  ctx.strokeRect(margin, surfY - 8, trackW, 8);

  /* Velocity-scaled ball radii */
  var r1px = Math.min(Math.max(Math.sqrt(m1) * 14, 14), 42);
  var r2px = Math.min(Math.max(Math.sqrt(m2) * 14, 14), 42);

  /* Ball positions */
  var b1x = margin + col.b1.x * trackW;
  var b2x = margin + col.b2.x * trackW;
  var by  = surfY - r1px;

  function drawBall(cx, cy, r, color, label) {
    ctx.save();
    ctx.shadowColor = color; ctx.shadowBlur = 20;
    var grad = ctx.createRadialGradient(cx - r * 0.3, cy - r * 0.3, 0, cx, cy, r);
    grad.addColorStop(0, color + 'ee'); grad.addColorStop(1, color + '55');
    ctx.fillStyle = grad;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.stroke(); ctx.shadowBlur = 0;
    ctx.fillStyle = '#000'; ctx.font = 'bold 11px DM Sans';
    ctx.textAlign = 'center'; ctx.fillText(label, cx, cy + 4);
    ctx.textAlign = 'left';
    ctx.restore();
  }

  drawBall(b1x, surfY - r1px, r1px, '#00d4ff', m1 + 'kg');
  drawBall(b2x, surfY - r2px, r2px, '#ff6b35', m2 + 'kg');

  /* Velocity arrows (momentum proportional) */
  var v1now = col.b1.v / 0.05, v2now = col.b2.v / 0.05;
  var arrowScale = 4;
  if (Math.abs(v1now) > 0.1)
    drawArrow(ctx, b1x, surfY - r1px, b1x + v1now * arrowScale, surfY - r1px, '#00d4ff', 'v1', 2);
  if (Math.abs(v2now) > 0.1)
    drawArrow(ctx, b2x, surfY - r2px, b2x + v2now * arrowScale, surfY - r2px, '#ff6b35', 'v2', 2);

  /* Phase label */
  ctx.fillStyle = col.phase === 'post' ? '#a8ff3e' : '#6b7a99';
  ctx.font = 'bold 12px Space Mono';
  ctx.fillText(type.toUpperCase() + ' — ' + col.phase.toUpperCase(), margin, surfY + 24);

  /* KE bar charts */
  var KEnow = 0.5 * m1 * v1now * v1now + 0.5 * m2 * v2now * v2now;
  var KEinit = 0.5 * m1 * v1i * v1i + 0.5 * m2 * v2i * v2i;
  var barY = H * 0.78, bh = 16, maxW = trackW / 2 - 20;
  var KEmax = Math.max(KEnow, KEinit, 1);

  ctx.fillStyle = 'rgba(10,13,20,0.7)'; ctx.fillRect(margin, barY - 20, trackW, bh + 30);
  ctx.fillStyle = 'rgba(107,122,153,0.5)'; ctx.font = '10px Space Mono';
  ctx.fillText('KE before: ' + fmtSci(KEinit) + ' J', margin, barY - 5);
  ctx.fillText('KE now:    ' + fmtSci(KEnow)  + ' J', margin + trackW / 2 + 20, barY - 5);

  /* KE before bar */
  ctx.fillStyle = 'rgba(30,42,66,0.6)'; ctx.fillRect(margin, barY, maxW, bh);
  ctx.fillStyle = 'rgba(0,212,255,0.5)';
  ctx.fillRect(margin, barY, Math.min(KEinit / KEmax * maxW, maxW), bh);

  /* KE now bar */
  ctx.fillStyle = 'rgba(30,42,66,0.6)'; ctx.fillRect(margin + maxW + 20, barY, maxW, bh);
  ctx.fillStyle = col.phase === 'post' ? 'rgba(168,255,62,0.5)' : 'rgba(0,212,255,0.5)';
  ctx.fillRect(margin + maxW + 20, barY, Math.min(KEnow / KEmax * maxW, maxW), bh);
}

/* ------------------------------------------------------------
   CSV EXPORT
   ------------------------------------------------------------ */
function exportCollisionCSV() {
  if (!col.data || col.data.length === 0) {
    alert('Run the simulation first to generate data.'); return;
  }
  exportCSV('collision_data.csv',
    ['time_s','x1_norm','v1_ms','x2_norm','v2_ms','KE_J'],
    col.data);
}
