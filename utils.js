/* ============================================================
   utils.js — PhysicsLab shared utilities
   Contains: formatting helpers, DOM helpers, physics solver
   engine, canvas drawing primitives, CSV export, tooltip system
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   FORMATTING HELPERS
   ------------------------------------------------------------ */

/**
 * Format a number for display. Uses exponential notation for
 * very large or very small values, otherwise 4 significant figures.
 * @param {number|null} n
 * @returns {string}
 */
function fmtSci(n) {
  if (n === null || n === undefined || !isFinite(n)) return '--';
  var abs = Math.abs(n);
  if (abs === 0) return '0';
  if (abs >= 1e5 || (abs < 0.001 && abs > 0)) return n.toExponential(3);
  return parseFloat(n.toPrecision(4)).toString();
}

/**
 * Format a number to a fixed number of decimal places.
 * Returns '?' for null/undefined (used in solver substitution strings).
 * @param {number|null} n
 * @param {number} [d=3]
 * @returns {string}
 */
function fvN(n, d) {
  d = (d === undefined) ? 3 : d;
  if (n === null || n === undefined || !isFinite(n)) return '?';
  return n.toFixed(d);
}

/**
 * Format a number for the old setResult() API (fixed decimals).
 * @param {number|null} n
 * @param {number} [dec=2]
 * @returns {string}
 */
function fmt(n, dec) {
  dec = (dec === undefined) ? 2 : dec;
  if (n === null || n === undefined || isNaN(n)) return '--';
  if (Math.abs(n) >= 10000 || (Math.abs(n) < 0.001 && n !== 0)) return n.toExponential(2);
  return n.toFixed(dec);
}

/* ------------------------------------------------------------
   DOM HELPERS
   ------------------------------------------------------------ */

/**
 * Read an input element and return its float value,
 * or null if empty / not a number.
 * @param {string} id - element id
 * @returns {number|null}
 */
function getNullable(id) {
  var el = document.getElementById(id);
  if (!el) return null;
  var v = el.value.trim();
  if (v === '') return null;
  var n = parseFloat(v);
  return isNaN(n) ? null : n;
}

/**
 * Read an input, return its float value, or a default.
 * @param {string} id
 * @param {number} [def=0]
 * @returns {number}
 */
function getVal(id, def) {
  def = (def === undefined) ? 0 : def;
  var el = document.getElementById(id);
  return el ? (parseFloat(el.value) || def) : def;
}

/**
 * Set a result element's text.
 * @param {string} id
 * @param {number|null} val
 * @param {string} [unit='']
 * @param {number} [dec=2]
 */
function setResult(id, val, unit, dec) {
  unit = unit || ''; dec = (dec === undefined) ? 2 : dec;
  var el = document.getElementById(id);
  if (!el) return;
  el.textContent = unit ? fmt(val, dec) + ' ' + unit : fmt(val, dec);
}

/**
 * Apply solved/user/target CSS classes to a result card element.
 * @param {string} cardId
 * @param {'target'|'user'|'derived'|''} state
 */
function styleCard(cardId, state) {
  var el = document.getElementById(cardId);
  if (!el) return;
  el.classList.remove('is-target','is-user','is-derived');
  if (state === 'target')  el.classList.add('is-target');
  if (state === 'user')    el.classList.add('is-user');
  if (state === 'derived') el.classList.add('is-derived');
}

/* ------------------------------------------------------------
   INPUT VALIDATION
   ------------------------------------------------------------ */

/**
 * Validate a parsed numeric value against optional constraints.
 * Returns an error message string, or null if valid.
 * @param {number|null} value
 * @param {object} opts - { min, max, nonzero, positive, label }
 * @returns {string|null}
 */
function validateInput(value, opts) {
  opts = opts || {};
  if (value === null) return null; // blank = unknown, always OK
  var label = opts.label || 'Value';
  if (!isFinite(value)) return label + ' must be a number';
  if (opts.positive && value <= 0) return label + ' must be > 0';
  if (opts.nonzero  && value === 0) return label + ' must not be zero';
  if (opts.min !== undefined && value < opts.min) return label + ' must be >= ' + opts.min;
  if (opts.max !== undefined && value > opts.max) return label + ' must be <= ' + opts.max;
  return null;
}

/**
 * Show or clear an error on an input element.
 * @param {string} inputId
 * @param {string|null} msg - error message or null to clear
 */
function showInputError(inputId, msg) {
  var el = document.getElementById(inputId);
  if (!el) return;
  var errId = inputId + '-err';
  var existing = document.getElementById(errId);
  if (msg) {
    el.classList.add('input-error');
    if (!existing) {
      var div = document.createElement('div');
      div.id = errId; div.className = 'input-error-msg';
      el.parentNode.insertBefore(div, el.nextSibling);
      existing = div;
    }
    existing.textContent = msg;
  } else {
    el.classList.remove('input-error');
    if (existing) existing.remove();
  }
}

/* ------------------------------------------------------------
   GENERIC PHYSICS SOLVER ENGINE
   Architecture:
     - state dict: keys = variable names, values = number|null
     - equation registry: array of { name, formula, solve:{key:fn} }
     - iterative propagation: repeat until stable or maxPasses
   ------------------------------------------------------------ */

/**
 * Run the propagation solver.
 * Each equation function receives the state dict and returns a
 * computed value, or null if prerequisites are missing / invalid.
 *
 * @param {object} state    - mutable state dict (values will be filled in)
 * @param {Array}  equations - equation registry
 * @param {number} [maxPasses=20]
 * @returns {{ log: Array, derived: object, passes: number }}
 */
function runSolver(state, equations, maxPasses) {
  maxPasses = maxPasses || 20;
  var log = [];          // array of step records
  var derived = {};      // keys that solver filled (not user-provided)

  var changed = true;
  var passes = 0;

  while (changed && passes < maxPasses) {
    changed = false;
    passes++;

    for (var ei = 0; ei < equations.length; ei++) {
      var eq = equations[ei];
      var solveKeys = Object.keys(eq.solve);

      for (var ki = 0; ki < solveKeys.length; ki++) {
        var key = solveKeys[ki];

        // Only fill if currently unknown
        if (state[key] !== null && state[key] !== undefined) continue;

        var result = eq.solve[key](state);

        // Guard: must be a finite number
        if (result === null || result === undefined || !isFinite(result)) continue;

        state[key] = result;
        derived[key] = true;
        changed = true;

        // Record the step for display
        log.push({
          eqName:    eq.name,
          formula:   eq.formula,
          varSolved: key,
          value:     result,
          buildSub:  eq.buildSub || null
        });
      }
    }
  }

  return { log: log, derived: derived, passes: passes };
}

/* ------------------------------------------------------------
   SOLVER STEP DISPLAY
   ------------------------------------------------------------ */

/**
 * Render solver step cards into a container element.
 * @param {string} containerId
 * @param {Array}  log        - from runSolver()
 * @param {object} derived    - from runSolver()
 * @param {object} state      - solved state dict
 * @param {string} solveFor   - target variable key
 * @param {string} emptyHint  - message to show when no steps
 */
function renderSolverSteps(containerId, log, derived, state, solveFor, emptyHint) {
  var box = document.getElementById(containerId);
  if (!box) return;

  if (log.length === 0) {
    box.innerHTML =
      '<div class="step-empty-msg">' + (emptyHint || 'Enter variables above.') + '</div>';
    return;
  }

  // Sort: target steps last so they stand out
  var targetSteps = log.filter(function(s){ return s.varSolved === solveFor; });
  var otherSteps  = log.filter(function(s){ return s.varSolved !== solveFor; });
  var display     = otherSteps.concat(targetSteps).slice(-5);

  var legend =
    '<div class="step-legend">' +
    '<span class="step-legend-item"><span class="step-legend-dot" style="background:rgba(255,107,53,0.6)"></span>User input</span>' +
    '<span class="step-legend-item"><span class="step-legend-dot" style="background:rgba(168,255,62,0.6)"></span>Solver derived</span>' +
    '<span class="step-legend-item"><span class="step-legend-dot" style="background:var(--accent)"></span>Target</span>' +
    '</div>';

  var summary = '';
  if (log.length > 5) {
    summary = '<div style="font-size:10px;color:var(--text-dim);font-family:Space Mono;margin-bottom:6px;">' +
      log.length + ' steps — showing last 5</div>';
  }

  var cards = display.map(function(step) {
    var isTarget = (step.varSolved === solveFor);
    var subStr = step.buildSub ? step.buildSub(state) : (step.formula + ' = ' + fmtSci(step.value));
    return '<div class="step-card' + (isTarget ? ' is-target' : '') + '">' +
      '<div class="step-eq-name">' + step.eqName + '</div>' +
      '<div class="step-formula">' + step.formula + '</div>' +
      '<div class="step-sub">' + subStr + '</div>' +
      '</div>';
  }).join('');

  box.innerHTML = legend + summary + cards;
}

/* ------------------------------------------------------------
   CANVAS DRAWING HELPERS
   ------------------------------------------------------------ */

/**
 * Draw a background grid on a canvas.
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} w - canvas width
 * @param {number} h - canvas height
 */
function drawGrid(ctx, w, h) {
  var spacing = 50;
  ctx.strokeStyle = 'rgba(30,42,66,0.7)';
  ctx.lineWidth = 0.5;
  for (var x = spacing; x < w; x += spacing) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (var y = spacing; y < h; y += spacing) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  // Axis lines
  ctx.strokeStyle = 'rgba(50,70,110,0.8)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, h - 2); ctx.lineTo(w, h - 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(2, 0);     ctx.lineTo(2, h);     ctx.stroke();
}

/**
 * Draw an arrow with optional label.
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} x1 - tail x
 * @param {number} y1 - tail y
 * @param {number} x2 - head x
 * @param {number} y2 - head y
 * @param {string} color
 * @param {string} [label='']
 * @param {number} [width=2]
 */
function drawArrow(ctx, x1, y1, x2, y2, color, label, width) {
  label = label || ''; width = width || 2;
  var dx = x2 - x1, dy = y2 - y1;
  var len = Math.sqrt(dx * dx + dy * dy);
  if (len < 2) return;

  var hs = Math.min(12, len * 0.4); // arrowhead size
  var angle = Math.atan2(dy, dx);

  ctx.save();
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
  ctx.shadowColor = color; ctx.shadowBlur = 6;

  // Shaft
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();

  // Arrowhead
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - hs * Math.cos(angle - 0.4), y2 - hs * Math.sin(angle - 0.4));
  ctx.lineTo(x2 - hs * Math.cos(angle + 0.4), y2 - hs * Math.sin(angle + 0.4));
  ctx.closePath(); ctx.fill();

  // Label
  if (label) {
    ctx.shadowBlur = 0;
    ctx.font = '11px Space Mono, monospace';
    ctx.fillStyle = color;
    ctx.fillText(label, (x1 + x2) / 2 + 6, (y1 + y2) / 2 - 5);
  }
  ctx.restore();
}

/**
 * Draw a glowing circle (for projectile bob, etc.)
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} x
 * @param {number} y
 * @param {number} r - radius
 * @param {string} color
 */
function drawGlowCircle(ctx, x, y, r, color) {
  ctx.save();
  ctx.shadowColor = color; ctx.shadowBlur = 24;
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2);
  var grad = ctx.createRadialGradient(x, y, 0, x, y, r);
  grad.addColorStop(0, color); grad.addColorStop(1, color + '88');
  ctx.fillStyle = grad; ctx.fill();
  ctx.shadowBlur = 0;
  ctx.restore();
}

/* ------------------------------------------------------------
   CSV EXPORT
   ------------------------------------------------------------ */

/**
 * Export a 2D array of rows to a CSV file download.
 * @param {string}   filename
 * @param {string[]} headers
 * @param {Array[]}  rows
 */
function exportCSV(filename, headers, rows) {
  var lines = [headers.join(',')];
  rows.forEach(function(row) {
    lines.push(row.map(function(v) {
      // Quote any cell containing commas
      var s = String(v);
      return s.indexOf(',') >= 0 ? '"' + s + '"' : s;
    }).join(','));
  });
  var blob = new Blob([lines.join('\n')], { type: 'text/csv' });
  var url  = URL.createObjectURL(blob);
  var a    = document.createElement('a');
  a.href = url; a.download = filename;
  document.body.appendChild(a); a.click();
  document.body.removeChild(a); URL.revokeObjectURL(url);
}

/* ------------------------------------------------------------
   TOOLTIP SYSTEM
   ------------------------------------------------------------ */

/**
 * Initialise hover tooltips on all elements with data-tip attribute.
 * Call once after DOM is ready.
 */
function initTooltips() {
  var tip = document.getElementById('globalTooltip');
  if (!tip) return;

  // Use event delegation on the whole document
  document.addEventListener('mouseover', function(e) {
    var el = e.target.closest('[data-tip]');
    if (!el) return;
    var text = el.getAttribute('data-tip');
    var title = el.getAttribute('data-tip-title') || '';
    tip.innerHTML = (title ? '<strong>' + title + '</strong>' : '') + text;
    tip.classList.add('visible');
    positionTooltip(e);
  });

  document.addEventListener('mousemove', function(e) {
    if (tip.classList.contains('visible')) positionTooltip(e);
  });

  document.addEventListener('mouseout', function(e) {
    var el = e.target.closest('[data-tip]');
    if (el) tip.classList.remove('visible');
  });

  function positionTooltip(e) {
    var margin = 14;
    var tw = tip.offsetWidth, th = tip.offsetHeight;
    var x = e.clientX + margin;
    var y = e.clientY + margin;
    // Prevent going off screen right/bottom
    if (x + tw > window.innerWidth  - 10) x = e.clientX - tw - margin;
    if (y + th > window.innerHeight - 10) y = e.clientY - th - margin;
    tip.style.left = x + 'px';
    tip.style.top  = y + 'px';
  }
}

/* ------------------------------------------------------------
   INFO BAR
   ------------------------------------------------------------ */

/**
 * Update the bottom info bar with current simulation values.
 * @param {number} t - time in seconds
 * @param {number} x - x position
 * @param {number} y - y position
 * @param {number} v - speed
 */
function updateInfoBar(t, x, y, v) {
  var et = document.getElementById('info-t');
  var ex = document.getElementById('info-x');
  var ey = document.getElementById('info-y');
  var ev = document.getElementById('info-v');
  if (et) et.textContent = fmt(t, 2) + ' s';
  if (ex) ex.textContent = fmt(x, 2) + ' m';
  if (ey) ey.textContent = fmt(y, 2) + ' m';
  if (ev) ev.textContent = fmt(v, 2) + ' m/s';
}

/* ============================================================
   SHARED ANIMATION UTILITIES
   Used by ALL simulators for consistent visual quality.
   ============================================================ */

/**
 * drawEnvironment — render a consistent background for any sim.
 * Draws the grid + a subtle vignette in the corners.
 */
function drawEnvironment(ctx, W, H) {
  /* Background */
  ctx.fillStyle = '#080b12';
  ctx.fillRect(0, 0, W, H);

  /* Grid */
  drawGrid(ctx, W, H);

  /* Subtle corner vignette for depth */
  var vg = ctx.createRadialGradient(W/2, H/2, Math.min(W,H)*0.3, W/2, H/2, Math.max(W,H)*0.75);
  vg.addColorStop(0, 'rgba(0,0,0,0)');
  vg.addColorStop(1, 'rgba(0,0,0,0.25)');
  ctx.fillStyle = vg;
  ctx.fillRect(0, 0, W, H);
}

/**
 * drawSurface — draw a glowing horizontal surface line
 * with a gradient fill below it.
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} x1, y, x2   - line endpoints
 * @param {string} color        - e.g. '#00d4ff'
 * @param {number} fillDepth    - px of gradient below line
 */
function drawSurface(ctx, x1, y, x2, color, fillDepth) {
  fillDepth = fillDepth || 20;
  ctx.save();

  /* Glow line */
  ctx.shadowColor = color;
  ctx.shadowBlur  = 8;
  ctx.strokeStyle = color;
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(x1, y);
  ctx.lineTo(x2, y);
  ctx.stroke();
  ctx.shadowBlur = 0;

  /* Gradient fill */
  var gf = ctx.createLinearGradient(0, y, 0, y + fillDepth);
  gf.addColorStop(0, color.replace(')', ',0.10)').replace('rgb', 'rgba'));
  gf.addColorStop(1, 'rgba(0,0,0,0)');
  /* Fallback for hex colors */
  if (color.startsWith('#')) {
    gf = ctx.createLinearGradient(0, y, 0, y + fillDepth);
    gf.addColorStop(0, 'rgba(0,212,255,0.10)');
    gf.addColorStop(1, 'rgba(0,0,0,0)');
  }
  ctx.fillStyle = gf;
  ctx.fillRect(x1, y, x2 - x1, fillDepth);

  ctx.restore();
}

/**
 * drawWall — draw a visible wall at a fixed x position.
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} x        - wall x pixel
 * @param {number} y1, y2   - top and bottom of wall
 * @param {string} side     - 'left' | 'right'
 * @param {string} color
 */
function drawWall(ctx, x, y1, y2, side, color) {
  color = color || 'rgba(0,212,255,0.4)';
  ctx.save();

  /* Wall line */
  ctx.strokeStyle = color;
  ctx.lineWidth   = 2;
  ctx.shadowColor = color;
  ctx.shadowBlur  = 6;
  ctx.beginPath();
  ctx.moveTo(x, y1);
  ctx.lineTo(x, y2);
  ctx.stroke();
  ctx.shadowBlur = 0;

  /* Hatch marks indicating wall */
  ctx.strokeStyle = color;
  ctx.lineWidth   = 1;
  var step = 12;
  var len  = 8;
  for (var wy = y1; wy < y2; wy += step) {
    ctx.beginPath();
    if (side === 'left') {
      ctx.moveTo(x, wy);
      ctx.lineTo(x - len, wy + len);
    } else {
      ctx.moveTo(x, wy);
      ctx.lineTo(x + len, wy + len);
    }
    ctx.stroke();
  }
  ctx.restore();
}

/**
 * drawLabel — draw a small text label with a background pill.
 * @param {CanvasRenderingContext2D} ctx
 * @param {string} text
 * @param {number} x, y      - centre position
 * @param {string} color
 * @param {string} [font]
 */
function drawLabel(ctx, text, x, y, color, font) {
  ctx.save();
  ctx.font = font || '11px Space Mono, monospace';
  var tw = ctx.measureText(text).width;
  var pad = 6;
  /* Pill background */
  ctx.fillStyle   = (color || '#00d4ff') + '22';
  ctx.strokeStyle = (color || '#00d4ff') + '66';
  ctx.lineWidth   = 1;
  ctx.beginPath();
  ctx.roundRect(x - tw/2 - pad, y - 10, tw + pad*2, 18, 4);
  ctx.fill();
  ctx.stroke();
  /* Text */
  ctx.fillStyle   = color || '#00d4ff';
  ctx.textAlign   = 'center';
  ctx.fillText(text, x, y + 3);
  ctx.textAlign   = 'left';
  ctx.restore();
}

/**
 * updateObject — move an object by its velocity for dt seconds.
 * Works in any coordinate space (physics metres or pixel space).
 * @param {{ x: number, v: number }} obj - must have x and v
 * @param {number} dt
 */
function updateObject(obj, dt) {
  obj.x += obj.v * dt;
}

/**
 * handleWallCollision — clamp obj.x to [Wmin+r, Wmax-r] and
 * reverse velocity (with restitution) if moving into wall.
 * Only reverses once per contact (direction guard prevents jitter).
 *
 * @param {{ x: number, v: number }} obj
 * @param {number} r      - radius in same units as x
 * @param {number} Wmin   - left wall
 * @param {number} Wmax   - right wall
 * @param {number} [restitution=0.8]
 */
function handleWallCollision(obj, r, Wmin, Wmax, restitution) {
  restitution = (restitution === undefined) ? 0.8 : restitution;
  var VEL_KILL = 0.8; /* px/s below which we zero out */

  /* Left wall */
  if (obj.x - r < Wmin) {
    obj.x = Wmin + r;
    if (obj.v < 0) {
      obj.v = -obj.v * restitution;
      if (Math.abs(obj.v) < VEL_KILL) obj.v = 0;
    }
  }

  /* Right wall */
  if (obj.x + r > Wmax) {
    obj.x = Wmax - r;
    if (obj.v > 0) {
      obj.v = -obj.v * restitution;
      if (Math.abs(obj.v) < VEL_KILL) obj.v = 0;
    }
  }
}

/**
 * detectCollision — check if two circles overlap.
 * @param {number} x1, x2   - centres
 * @param {number} r1, r2   - radii
 * @returns {boolean}
 */
function detectCollision(x1, r1, x2, r2) {
  return Math.abs(x2 - x1) < (r1 + r2);
}
