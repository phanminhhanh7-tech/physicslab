/* ============================================================
   collision.js — General Collision Solver
   Supports: elastic, inelastic (with e), perfectly inelastic
   Variables: m1, m2, v1i, v2i, v1f, v2f, vf, e
   Solver: algebraic, handles any combination of knowns
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   STATE
   ------------------------------------------------------------ */
var col = {
  b1: { x: 0.12, v: 0 },
  b2: { x: 0.78, v: 0 },
  collided: false,
  phase: 'pre',
  data: [],
  _m1: 5, _m2: 3, _v1i: 8, _v2i: -3,
  _v1f: null, _v2f: null,
  _type: 'elastic'
};

/* ------------------------------------------------------------
   RESET
   ------------------------------------------------------------ */
function resetCollision() {
  var prev = { m1: col._m1, m2: col._m2, v1i: col._v1i, v2i: col._v2i };
  col = {
    b1: { x: 0.12, v: 0 }, b2: { x: 0.78, v: 0 },
    collided: false, phase: 'pre', data: [],
    _m1: prev.m1, _m2: prev.m2,
    _v1i: prev.v1i, _v2i: prev.v2i,
    _v1f: null, _v2f: null, _type: 'elastic'
  };
  ['col-m1','col-m2','col-v1i','col-v2i','col-v1f','col-v2f',
   'col-vf','col-e','col-KEb','col-KEa','col-KElost','col-pt',
   'col-p1i','col-p2i'].forEach(function(id) {
    var el = document.getElementById(id);
    if (el) el.value = '';
  });
  updateCollision();
}

/* ============================================================
   GENERAL ALGEBRAIC SOLVER
   ============================================================

   Physics laws used:
   (1) Momentum conservation:  m1*v1i + m2*v2i = m1*v1f + m2*v2f
   (2) Elastic KE conservation: ½m1*v1i² + ½m2*v2i² = ½m1*v1f² + ½m2*v2f²
   (3) Coefficient of restitution: e = (v2f - v1f) / (v1i - v2i)
   (4) Perfectly inelastic: vf = (m1*v1i + m2*v2i) / (m1+m2)  [v1f = v2f = vf]

   Strategy:
   - Build a known-values dict from all inputs
   - Run a propagation loop (up to 30 passes) using all
     algebraic rearrangements of the four laws
   - Each rule fires only when its prerequisites are met
     and the target is still unknown
   - Combines rules when needed (e.g. momentum + restitution
     to solve simultaneously for v1f and v2f)
   ============================================================ */

/**
 * Main collision solver.
 * @param {object} known  - dict of variable name → number (null = unknown)
 * @param {string} type   - 'elastic' | 'inelastic' | 'perfect'
 * @returns {{ state: object, log: Array, error: string|null }}
 */
function solveCollision(known, type) {

  /* Clone inputs so we don't mutate the original */
  var s = {};
  var inputKeys = ['m1','m2','v1i','v2i','v1f','v2f','vf','e',
                   'KEb','KEa','KElost','pt','p1i','p2i'];
  inputKeys.forEach(function(k) { s[k] = (known[k] !== undefined) ? known[k] : null; });

  var log = [];     /* step records for display */
  var derived = {}; /* which keys the solver filled */

  /* ---- helper: record a step ---- */
  function record(name, formula, key, value, subFn) {
    s[key] = value;
    derived[key] = true;
    log.push({ eqName: name, formula: formula, varSolved: key,
               value: value, buildSub: subFn });
  }

  /* ---- safe helpers ---- */
  var K = 1e-12; // zero-guard threshold
  function ok()  { /* check all args non-null */
    for (var i = 0; i < arguments.length; i++)
      if (arguments[i] === null || arguments[i] === undefined) return false;
    return true;
  }
  function unknown(k) { return s[k] === null || s[k] === undefined; }
  function known_(k)  { return !unknown(k); }

  /* ============================================================
     RULE SET — fired repeatedly until no new values emerge
     ============================================================ */
  var MAX_PASSES = 30;
  var changed = true;
  var pass = 0;

  while (changed && pass < MAX_PASSES) {
    changed = false;
    pass++;

    /* ----------------------------------------------------------
       BLOCK A: Simple one-variable derivations
       ---------------------------------------------------------- */

    /* A1. Momentum before: pt = m1*v1i + m2*v2i */
    if (unknown('pt') && ok(s.m1, s.v1i, s.m2, s.v2i)) {
      var pt = s.m1 * s.v1i + s.m2 * s.v2i;
      record('Momentum conservation', 'pt = m1·v1i + m2·v2i', 'pt', pt,
        function(st) { return 'pt = ' + fvN(st.m1) + '×' + fvN(st.v1i) + ' + ' + fvN(st.m2) + '×' + fvN(st.v2i) + ' = ' + fmtSci(st.pt) + ' kg·m/s'; });
      changed = true;
    }

    /* A2. p1i = m1*v1i */
    if (unknown('p1i') && ok(s.m1, s.v1i)) {
      record('Momentum object 1', 'p1i = m1·v1i', 'p1i', s.m1 * s.v1i,
        function(st) { return 'p1i = ' + fvN(st.m1) + '×' + fvN(st.v1i) + ' = ' + fmtSci(st.p1i) + ' kg·m/s'; });
      changed = true;
    }

    /* A3. p2i = m2*v2i */
    if (unknown('p2i') && ok(s.m2, s.v2i)) {
      record('Momentum object 2', 'p2i = m2·v2i', 'p2i', s.m2 * s.v2i,
        function(st) { return 'p2i = ' + fvN(st.m2) + '×' + fvN(st.v2i) + ' = ' + fmtSci(st.p2i) + ' kg·m/s'; });
      changed = true;
    }

    /* A4. pt from p1i + p2i */
    if (unknown('pt') && ok(s.p1i, s.p2i)) {
      record('Total momentum', 'pt = p1i + p2i', 'pt', s.p1i + s.p2i,
        function(st) { return 'pt = ' + fmtSci(st.p1i) + ' + ' + fmtSci(st.p2i) + ' = ' + fmtSci(st.pt) + ' kg·m/s'; });
      changed = true;
    }

    /* A5. Back-solve: m1 from pt, m2, v1i, v2i */
    if (unknown('m1') && ok(s.pt, s.m2, s.v1i, s.v2i)) {
      var dv1 = s.v1i;
      if (Math.abs(dv1) > K) {
        var m1_ = (s.pt - s.m2 * s.v2i) / dv1;
        if (m1_ > 0) {
          record('Momentum → m1', 'm1 = (pt − m2·v2i) / v1i', 'm1', m1_,
            function(st) { return 'm1 = (' + fmtSci(st.pt) + ' − ' + fvN(st.m2) + '×' + fvN(st.v2i) + ') / ' + fvN(st.v1i) + ' = ' + fmtSci(st.m1) + ' kg'; });
          changed = true;
        }
      }
    }

    /* A6. Back-solve: m2 from pt, m1, v1i, v2i */
    if (unknown('m2') && ok(s.pt, s.m1, s.v1i, s.v2i)) {
      var dv2_ = s.v2i;
      if (Math.abs(dv2_) > K) {
        var m2_ = (s.pt - s.m1 * s.v1i) / dv2_;
        if (m2_ > 0) {
          record('Momentum → m2', 'm2 = (pt − m1·v1i) / v2i', 'm2', m2_,
            function(st) { return 'm2 = (' + fmtSci(st.pt) + ' − ' + fvN(st.m1) + '×' + fvN(st.v1i) + ') / ' + fvN(st.v2i) + ' = ' + fmtSci(st.m2) + ' kg'; });
          changed = true;
        }
      }
    }

    /* A7. v1i from pt, m1, m2, v2i */
    if (unknown('v1i') && ok(s.pt, s.m1, s.m2, s.v2i) && Math.abs(s.m1) > K) {
      record('Momentum → v1i', 'v1i = (pt − m2·v2i) / m1', 'v1i',
        (s.pt - s.m2 * s.v2i) / s.m1,
        function(st) { return 'v1i = (' + fmtSci(st.pt) + ' − ' + fvN(st.m2) + '×' + fvN(st.v2i) + ') / ' + fvN(st.m1) + ' = ' + fmtSci(st.v1i) + ' m/s'; });
      changed = true;
    }

    /* A8. v2i from pt, m1, m2, v1i */
    if (unknown('v2i') && ok(s.pt, s.m1, s.m2, s.v1i) && Math.abs(s.m2) > K) {
      record('Momentum → v2i', 'v2i = (pt − m1·v1i) / m2', 'v2i',
        (s.pt - s.m1 * s.v1i) / s.m2,
        function(st) { return 'v2i = (' + fmtSci(st.pt) + ' − ' + fvN(st.m1) + '×' + fvN(st.v1i) + ') / ' + fvN(st.m2) + ' = ' + fmtSci(st.v2i) + ' m/s'; });
      changed = true;
    }

    /* A9. KEb = ½m1*v1i² + ½m2*v2i² */
    if (unknown('KEb') && ok(s.m1, s.v1i, s.m2, s.v2i)) {
      record('KE before', 'KEb = ½m1v1i² + ½m2v2i²', 'KEb',
        0.5 * s.m1 * s.v1i * s.v1i + 0.5 * s.m2 * s.v2i * s.v2i,
        function(st) { return 'KEb = ½×' + fvN(st.m1) + '×' + fvN(st.v1i) + '² + ½×' + fvN(st.m2) + '×' + fvN(st.v2i) + '² = ' + fmtSci(st.KEb) + ' J'; });
      changed = true;
    }

    /* A10. KEa = ½m1*v1f² + ½m2*v2f² */
    if (unknown('KEa') && ok(s.m1, s.v1f, s.m2, s.v2f)) {
      record('KE after', 'KEa = ½m1v1f² + ½m2v2f²', 'KEa',
        0.5 * s.m1 * s.v1f * s.v1f + 0.5 * s.m2 * s.v2f * s.v2f,
        function(st) { return 'KEa = ½×' + fvN(st.m1) + '×' + fvN(st.v1f) + '² + ½×' + fvN(st.m2) + '×' + fvN(st.v2f) + '² = ' + fmtSci(st.KEa) + ' J'; });
      changed = true;
    }

    /* A11. KElost = KEb - KEa */
    if (unknown('KElost') && ok(s.KEb, s.KEa)) {
      record('KE lost', 'KElost = KEb − KEa', 'KElost', s.KEb - s.KEa,
        function(st) { return 'KElost = ' + fmtSci(st.KEb) + ' − ' + fmtSci(st.KEa) + ' = ' + fmtSci(st.KElost) + ' J'; });
      changed = true;
    }

    /* A12. KEb from KElost + KEa */
    if (unknown('KEb') && ok(s.KElost, s.KEa)) {
      record('KE before from lost', 'KEb = KEa + KElost', 'KEb', s.KEa + s.KElost,
        function(st) { return 'KEb = ' + fmtSci(st.KEa) + ' + ' + fmtSci(st.KElost) + ' = ' + fmtSci(st.KEb) + ' J'; });
      changed = true;
    }

    /* A13. KEa from KEb - KElost */
    if (unknown('KEa') && ok(s.KElost, s.KEb)) {
      record('KE after from lost', 'KEa = KEb − KElost', 'KEa', s.KEb - s.KElost,
        function(st) { return 'KEa = ' + fmtSci(st.KEb) + ' − ' + fmtSci(st.KElost) + ' = ' + fmtSci(st.KEa) + ' J'; });
      changed = true;
    }

    /* ----------------------------------------------------------
       BLOCK B: Post-collision velocities
       ---------------------------------------------------------- */

    /* B1. PERFECTLY INELASTIC: vf = pt / (m1+m2), then v1f = v2f = vf */
    if (type === 'perfect') {
      if (unknown('vf') && ok(s.pt, s.m1, s.m2)) {
        var mt = s.m1 + s.m2;
        if (Math.abs(mt) > K) {
          record('Perfectly inelastic', 'vf = pt / (m1+m2)', 'vf', s.pt / mt,
            function(st) { return 'vf = ' + fmtSci(st.pt) + ' / (' + fvN(st.m1) + '+' + fvN(st.m2) + ') = ' + fmtSci(st.vf) + ' m/s'; });
          changed = true;
        }
      }
      if (unknown('v1f') && ok(s.vf)) {
        record('v1f = vf (stick)', 'v1f = vf', 'v1f', s.vf,
          function(st) { return 'v1f = vf = ' + fmtSci(st.vf) + ' m/s (objects stick together)'; });
        changed = true;
      }
      if (unknown('v2f') && ok(s.vf)) {
        record('v2f = vf (stick)', 'v2f = vf', 'v2f', s.vf,
          function(st) { return 'v2f = vf = ' + fmtSci(st.vf) + ' m/s (objects stick together)'; });
        changed = true;
      }
      /* Back-solve m1+m2 from vf and pt */
      if (unknown('m1') && ok(s.vf, s.pt, s.m2) && Math.abs(s.vf) > K) {
        var m1_pi = s.pt / s.vf - s.m2;
        if (m1_pi > 0) {
          record('Perfect inelastic → m1', 'm1 = pt/vf − m2', 'm1', m1_pi,
            function(st) { return 'm1 = ' + fmtSci(st.pt) + '/' + fvN(st.vf) + ' − ' + fvN(st.m2) + ' = ' + fmtSci(st.m1) + ' kg'; });
          changed = true;
        }
      }
      if (unknown('m2') && ok(s.vf, s.pt, s.m1) && Math.abs(s.vf) > K) {
        var m2_pi = s.pt / s.vf - s.m1;
        if (m2_pi > 0) {
          record('Perfect inelastic → m2', 'm2 = pt/vf − m1', 'm2', m2_pi,
            function(st) { return 'm2 = ' + fmtSci(st.pt) + '/' + fvN(st.vf) + ' − ' + fvN(st.m1) + ' = ' + fmtSci(st.m2) + ' kg'; });
          changed = true;
        }
      }
    }

    /* B2. ELASTIC: standard closed-form solution
       v1f = [(m1-m2)v1i + 2m2*v2i] / (m1+m2)
       v2f = [(m2-m1)v2i + 2m1*v1i] / (m1+m2)  */
    if (type === 'elastic') {
      if (ok(s.m1, s.m2, s.v1i, s.v2i)) {
        var mt_e = s.m1 + s.m2;
        if (Math.abs(mt_e) > K) {
          if (unknown('v1f')) {
            record('Elastic v1f', 'v1f = [(m1−m2)v1i + 2m2v2i] / (m1+m2)', 'v1f',
              ((s.m1 - s.m2) * s.v1i + 2 * s.m2 * s.v2i) / mt_e,
              function(st) { return 'v1f = [(' + fvN(st.m1) + '−' + fvN(st.m2) + ')×' + fvN(st.v1i) + ' + 2×' + fvN(st.m2) + '×' + fvN(st.v2i) + '] / ' + fvN(st.m1 + st.m2) + ' = ' + fmtSci(st.v1f) + ' m/s'; });
            changed = true;
          }
          if (unknown('v2f')) {
            record('Elastic v2f', 'v2f = [2m1v1i + (m2−m1)v2i] / (m1+m2)', 'v2f',
              (2 * s.m1 * s.v1i + (s.m2 - s.m1) * s.v2i) / mt_e,
              function(st) { return 'v2f = [2×' + fvN(st.m1) + '×' + fvN(st.v1i) + ' + (' + fvN(st.m2) + '−' + fvN(st.m1) + ')×' + fvN(st.v2i) + '] / ' + fvN(st.m1 + st.m2) + ' = ' + fmtSci(st.v2f) + ' m/s'; });
            changed = true;
          }
        }
      }

      /* Elastic: e = 1 always */
      if (unknown('e')) {
        record('Elastic restitution', 'e = 1 (elastic)', 'e', 1,
          function() { return 'e = 1 (kinetic energy conserved)'; });
        changed = true;
      }

      /* Elastic back-solve: given v1f, v2f → find m1 or m2
         From momentum: m1(v1i - v1f) = m2(v2f - v2i)
         → m1 = m2(v2f - v2i)/(v1i - v1f)  */
      if (unknown('m1') && ok(s.m2, s.v1i, s.v1f, s.v2i, s.v2f)) {
        var d1 = s.v1i - s.v1f;
        if (Math.abs(d1) > K) {
          var m1_e = s.m2 * (s.v2f - s.v2i) / d1;
          if (m1_e > 0) {
            record('Elastic → m1', 'm1 = m2(v2f−v2i)/(v1i−v1f)', 'm1', m1_e,
              function(st) { return 'm1 = ' + fvN(st.m2) + '×(' + fvN(st.v2f) + '−' + fvN(st.v2i) + ')/(' + fvN(st.v1i) + '−' + fvN(st.v1f) + ') = ' + fmtSci(st.m1) + ' kg'; });
            changed = true;
          }
        }
      }
      if (unknown('m2') && ok(s.m1, s.v1i, s.v1f, s.v2i, s.v2f)) {
        var d2_e = s.v2f - s.v2i;
        if (Math.abs(d2_e) > K) {
          var m2_e = s.m1 * (s.v1i - s.v1f) / d2_e;
          if (m2_e > 0) {
            record('Elastic → m2', 'm2 = m1(v1i−v1f)/(v2f−v2i)', 'm2', m2_e,
              function(st) { return 'm2 = ' + fvN(st.m1) + '×(' + fvN(st.v1i) + '−' + fvN(st.v1f) + ')/(' + fvN(st.v2f) + '−' + fvN(st.v2i) + ') = ' + fmtSci(st.m2) + ' kg'; });
            changed = true;
          }
        }
      }
    }

    /* B3. INELASTIC with restitution coefficient e
       System of 2 equations:
         (1) m1*v1f + m2*v2f = pt           (momentum)
         (2) v2f - v1f = e*(v1i - v2i)      (restitution)
       Solve simultaneously:
         rel = e*(v1i - v2i)
         v1f = (pt - m2*rel) / (m1+m2)  ... wait, substitute:
         from (2): v2f = v1f + rel
         sub into (1): m1*v1f + m2*(v1f + rel) = pt
         → v1f*(m1+m2) = pt - m2*rel
         → v1f = (pt - m2*rel) / (m1+m2)
         → v2f = v1f + rel
    */
    if (type === 'inelastic') {
      /* e from restitution definition */
      if (unknown('e') && ok(s.v1i, s.v2i, s.v1f, s.v2f)) {
        var denom_e = s.v1i - s.v2i;
        if (Math.abs(denom_e) > K) {
          var e_val = (s.v2f - s.v1f) / denom_e;
          if (e_val >= 0 && e_val <= 1 + 1e-9) {
            record('Restitution', 'e = (v2f−v1f) / (v1i−v2i)', 'e', Math.min(e_val, 1),
              function(st) { return 'e = (' + fvN(st.v2f) + '−' + fvN(st.v1f) + ') / (' + fvN(st.v1i) + '−' + fvN(st.v2i) + ') = ' + fmtSci(st.e); });
            changed = true;
          }
        }
      }

      /* Solve for both v1f and v2f simultaneously using momentum + restitution */
      if (unknown('v1f') && unknown('v2f') && ok(s.e, s.pt, s.m1, s.m2, s.v1i, s.v2i)) {
        var rel = s.e * (s.v1i - s.v2i);
        var mt_i = s.m1 + s.m2;
        if (Math.abs(mt_i) > K) {
          var v1f_i = (s.pt - s.m2 * rel) / mt_i;
          var v2f_i = v1f_i + rel;
          record('Momentum + restitution → v1f', 'v1f = (pt − m2·e·Δv) / (m1+m2)', 'v1f', v1f_i,
            function(st) { return 'v1f = (' + fmtSci(st.pt) + ' − ' + fvN(st.m2) + '×' + fmtSci(st.e) + '×' + fvN(st.v1i - st.v2i) + ') / ' + fvN(st.m1 + st.m2) + ' = ' + fmtSci(st.v1f) + ' m/s'; });
          record('Momentum + restitution → v2f', 'v2f = v1f + e·(v1i−v2i)', 'v2f', v2f_i,
            function(st) { return 'v2f = ' + fmtSci(st.v1f) + ' + ' + fmtSci(st.e) + '×(' + fvN(st.v1i) + '−' + fvN(st.v2i) + ') = ' + fmtSci(st.v2f) + ' m/s'; });
          changed = true;
        }
      }

      /* Solve v1f only (v2f known) using momentum */
      if (unknown('v1f') && known_('v2f') && ok(s.pt, s.m1, s.m2) && Math.abs(s.m1) > K) {
        record('Momentum → v1f', 'v1f = (pt − m2·v2f) / m1', 'v1f',
          (s.pt - s.m2 * s.v2f) / s.m1,
          function(st) { return 'v1f = (' + fmtSci(st.pt) + ' − ' + fvN(st.m2) + '×' + fvN(st.v2f) + ') / ' + fvN(st.m1) + ' = ' + fmtSci(st.v1f) + ' m/s'; });
        changed = true;
      }

      /* Solve v2f only (v1f known) using momentum */
      if (unknown('v2f') && known_('v1f') && ok(s.pt, s.m1, s.m2) && Math.abs(s.m2) > K) {
        record('Momentum → v2f', 'v2f = (pt − m1·v1f) / m2', 'v2f',
          (s.pt - s.m1 * s.v1f) / s.m2,
          function(st) { return 'v2f = (' + fmtSci(st.pt) + ' − ' + fvN(st.m1) + '×' + fvN(st.v1f) + ') / ' + fvN(st.m2) + ' = ' + fmtSci(st.v2f) + ' m/s'; });
        changed = true;
      }

      /* Back-solve e if both final velocities known */
      if (unknown('e') && ok(s.v1i, s.v2i, s.v1f, s.v2f)) {
        var drel = s.v1i - s.v2i;
        if (Math.abs(drel) > K) {
          var e2 = (s.v2f - s.v1f) / drel;
          if (e2 >= -K && e2 <= 1 + K) {
            record('Restitution coefficient', 'e = (v2f−v1f)/(v1i−v2i)', 'e',
              Math.max(0, Math.min(1, e2)),
              function(st) { return 'e = (' + fvN(st.v2f) + '−' + fvN(st.v1f) + ')/(' + fvN(st.v1i) + '−' + fvN(st.v2i) + ') = ' + fmtSci(st.e); });
            changed = true;
          }
        }
      }

      /* Back-solve m1 from post-collision velocities + momentum */
      if (unknown('m1') && ok(s.pt, s.m2, s.v1f, s.v2f)) {
        var dv1_i = s.v1f;
        if (Math.abs(dv1_i) > K && ok(s.v2i, s.m2)) {
          // momentum: m1*v1f + m2*v2f = pt => m1 = (pt - m2*v2f)/v1f
          var m1_i = (s.pt - s.m2 * s.v2f) / s.v1f;
          if (m1_i > 0) {
            record('Momentum → m1 (post)', 'm1 = (pt − m2·v2f) / v1f', 'm1', m1_i,
              function(st) { return 'm1 = (' + fmtSci(st.pt) + '−' + fvN(st.m2) + '×' + fvN(st.v2f) + ')/' + fvN(st.v1f) + ' = ' + fmtSci(st.m1) + ' kg'; });
            changed = true;
          }
        }
      }
      if (unknown('m2') && ok(s.pt, s.m1, s.v1f, s.v2f) && Math.abs(s.v2f) > K) {
        var m2_i = (s.pt - s.m1 * s.v1f) / s.v2f;
        if (m2_i > 0) {
          record('Momentum → m2 (post)', 'm2 = (pt − m1·v1f) / v2f', 'm2', m2_i,
            function(st) { return 'm2 = (' + fmtSci(st.pt) + '−' + fvN(st.m1) + '×' + fvN(st.v1f) + ')/' + fvN(st.v2f) + ' = ' + fmtSci(st.m2) + ' kg'; });
          changed = true;
        }
      }
    }

    /* B4. GENERIC: if v1f and v2f both known, momentum post-check */
    if (unknown('pt') && ok(s.m1, s.v1f, s.m2, s.v2f)) {
      // pt is conserved: pt = m1*v1f + m2*v2f
      var pt_post = s.m1 * s.v1f + s.m2 * s.v2f;
      // only use if we don't already have pt from initial side
      if (unknown('pt')) {
        record('Momentum (post)', 'pt = m1·v1f + m2·v2f', 'pt', pt_post,
          function(st) { return 'pt = ' + fvN(st.m1) + '×' + fvN(st.v1f) + ' + ' + fvN(st.m2) + '×' + fvN(st.v2f) + ' = ' + fmtSci(st.pt) + ' kg·m/s'; });
        changed = true;
      }
    }

  } /* end while */

  /* ---- Physical validation ---- */
  var warnings = [];
  if (ok(s.m1) && s.m1 <= 0) warnings.push('m1 must be positive');
  if (ok(s.m2) && s.m2 <= 0) warnings.push('m2 must be positive');
  if (type === 'inelastic' && ok(s.e) && (s.e < 0 || s.e > 1))
    warnings.push('e must be 0 ≤ e ≤ 1');
  if (ok(s.KEa, s.KEb) && s.KEa > s.KEb + 1e-6)
    warnings.push('KE after > KE before — energy created? Check inputs.');

  return { state: s, log: log, derived: derived, warnings: warnings };
}

/* ============================================================
   METADATA — all display variables
   ============================================================ */
var COL_META = {
  m1:     { label:'Mass 1 (m1)',    unit:'kg',     card:'crc-m1',     val:'col-r-m1'  },
  m2:     { label:'Mass 2 (m2)',    unit:'kg',     card:'crc-m2',     val:'col-r-m2'  },
  v1i:    { label:'v1 initial',     unit:'m/s',    card:'crc-v1i',    val:'col-r-v1i' },
  v2i:    { label:'v2 initial',     unit:'m/s',    card:'crc-v2i',    val:'col-r-v2i' },
  v1f:    { label:'v1 final',       unit:'m/s',    card:'crc-v1f',    val:'col-r-v1f' },
  v2f:    { label:'v2 final',       unit:'m/s',    card:'crc-v2f',    val:'col-r-v2f' },
  vf:     { label:'vf (stuck)',     unit:'m/s',    card:'crc-vf',     val:'col-r-vf'  },
  e:      { label:'Restitution (e)',unit:'',       card:'crc-e',      val:'col-r-e'   },
  pt:     { label:'p total',        unit:'kg·m/s', card:'crc-pt',     val:'col-r-pt'  },
  p1i:    { label:'p1 initial',     unit:'kg·m/s', card:'crc-p1i',    val:'col-r-p1i' },
  p2i:    { label:'p2 initial',     unit:'kg·m/s', card:'crc-p2i',    val:'col-r-p2i' },
  KEb:    { label:'KE before',      unit:'J',      card:'crc-KEb',    val:'col-r-keb' },
  KEa:    { label:'KE after',       unit:'J',      card:'crc-KEa',    val:'col-r-kea' },
  KElost: { label:'KE lost',        unit:'J',      card:'crc-KElost', val:'col-r-kel' }
};

/* ============================================================
   UPDATE — read inputs, run solver, update UI
   ============================================================ */
function updateCollision() {
  var type = document.getElementById('col-type').value;

  /* 1. Read all inputs */
  var known = {
    m1:     getNullable('col-m1'),
    m2:     getNullable('col-m2'),
    v1i:    getNullable('col-v1i'),
    v2i:    getNullable('col-v2i'),
    v1f:    getNullable('col-v1f'),
    v2f:    getNullable('col-v2f'),
    vf:     getNullable('col-vf'),
    e:      getNullable('col-e'),
    KEb:    getNullable('col-KEb'),
    KEa:    getNullable('col-KEa'),
    KElost: getNullable('col-KElost'),
    pt:     getNullable('col-pt'),
    p1i:    getNullable('col-p1i'),
    p2i:    getNullable('col-p2i')
  };

  /* For perfectly inelastic: v1f = v2f = vf — copy vf into both if given */
  if (type === 'perfect' && known.vf !== null) {
    if (known.v1f === null) known.v1f = known.vf;
    if (known.v2f === null) known.v2f = known.vf;
  }

  /* 2. Input validation */
  showInputError('col-m1', validateInput(known.m1, { positive: true, label: 'm1' }));
  showInputError('col-m2', validateInput(known.m2, { positive: true, label: 'm2' }));
  showInputError('col-e',  validateInput(known.e,  { min: 0, max: 1,  label: 'e'  }));

  /* Track user-provided keys for card colouring */
  var userKeys = {};
  Object.keys(known).forEach(function(k) { if (known[k] !== null) userKeys[k] = true; });

  var solveFor = document.getElementById('col-solveFor').value;

  /* 3. Run general solver */
  var result = solveCollision(known, type);
  var s = result.state;

  /* 4. Update result cards */
  Object.keys(COL_META).forEach(function(k) {
    var meta  = COL_META[k];
    var valEl = document.getElementById(meta.val);
    if (!valEl) return;
    var val = s[k];
    valEl.textContent = (val !== null && val !== undefined)
      ? fmtSci(val) + (meta.unit ? ' ' + meta.unit : '')
      : '--';
    var st = (k === solveFor) ? 'target'
           : (userKeys[k]    ? 'user'
           : (val !== null   ? 'derived' : ''));
    styleCard(meta.card, st);
  });

  /* 5. Solver steps */
  renderSolverSteps('col-solver-steps', result.log, result.derived, s, solveFor,
    'Enter any known variables — solver uses momentum conservation,\nrestitution, and KE equations to find unknowns.');

  /* 6. Warnings panel */
  var warnEl = document.getElementById('col-warnings');
  if (warnEl) {
    if (result.warnings.length > 0) {
      warnEl.style.display = 'block';
      warnEl.textContent = '⚠ ' + result.warnings.join(' | ');
    } else {
      warnEl.style.display = 'none';
    }
  }

  /* 7. Store for animation */
  col._m1   = s.m1  || 5;
  col._m2   = s.m2  || 3;
  col._v1i  = s.v1i !== null ? s.v1i : 8;
  col._v2i  = s.v2i !== null ? s.v2i : -3;
  col._v1f  = s.v1f;
  col._v2f  = s.v2f;
  col._type = type;

  if (!simRunning) {
    col.b1 = { x: 0.12, v: col._v1i * 0.05 };
    col.b2 = { x: 0.78, v: col._v2i * 0.05 };
    col.phase    = 'pre';
    col.collided = false;
    drawCollision();
  }
}

/* ============================================================
   STEP — animation physics
   ============================================================ */
function stepCollision(dt) {
  var m1   = col._m1, m2   = col._m2;
  var v1i  = col._v1i, v2i = col._v2i;
  var type = col._type || 'elastic';

  if (simTime <= dt + 0.001) {
    col.b1 = { x: 0.12, v: v1i * 0.05 };
    col.b2 = { x: 0.78, v: v2i * 0.05 };
    col.collided = false; col.phase = 'pre';
    col.data = [];
  }

  var r1 = 0.07, r2 = 0.07;

  if (!col.collided) {
    col.b1.x += col.b1.v * dt;
    col.b2.x += col.b2.v * dt;

    if (Math.abs(col.b1.x - col.b2.x) <= r1 + r2) {
      col.collided = true; col.phase = 'post';
      var v1 = col.b1.v / 0.05, v2 = col.b2.v / 0.05;
      var mt = m1 + m2;
      var v1f, v2f;

      if (type === 'elastic') {
        v1f = ((m1 - m2) * v1 + 2 * m2 * v2) / mt;
        v2f = (2 * m1 * v1 + (m2 - m1) * v2) / mt;
      } else if (type === 'perfect') {
        v1f = v2f = (m1 * v1 + m2 * v2) / mt;
      } else {
        /* inelastic: use solver's computed v1f/v2f if available,
           otherwise fall back to e=0.5 */
        var e = getNullable('col-e');
        if (e === null) e = 0.5;
        var rel = e * (v1 - v2);
        v1f = (m1 * v1 + m2 * v2 - m2 * rel) / mt;
        v2f = v1f + rel;
      }

      col.b1.v = v1f * 0.05;
      col.b2.v = v2f * 0.05;
    }
  } else {
    col.b1.x += col.b1.v * dt;
    col.b2.x += col.b2.v * dt;
  }

  if (col.b1.x < r1)     { col.b1.x = r1;     col.b1.v *= -0.85; }
  if (col.b2.x > 1 - r2) { col.b2.x = 1 - r2; col.b2.v *= -0.85; }

  var v1now = col.b1.v / 0.05, v2now = col.b2.v / 0.05;
  var KE = 0.5 * m1 * v1now * v1now + 0.5 * m2 * v2now * v2now;
  col.data.push([parseFloat(simTime.toFixed(3)),
    parseFloat(col.b1.x.toFixed(4)), parseFloat(v1now.toFixed(3)),
    parseFloat(col.b2.x.toFixed(4)), parseFloat(v2now.toFixed(3)),
    parseFloat(KE.toFixed(3))]);

  updateInfoBar(simTime, col.b1.x, 0, Math.abs(v1now));
}

/* ============================================================
   DRAW
   ============================================================ */
function drawCollision() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var m1   = col._m1 || 5,  m2   = col._m2 || 3;
  var v1i  = col._v1i || 8, v2i  = col._v2i || -3;
  var type = col._type || 'elastic';

  var surfY  = H * 0.60;
  var margin = 60;
  var trackW = W - margin * 2;

  /* Surface */
  ctx.strokeStyle = 'rgba(0,212,255,0.4)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(margin, surfY); ctx.lineTo(W - margin, surfY); ctx.stroke();

  /* Mass-scaled radii */
  var r1px = Math.min(Math.max(Math.sqrt(m1) * 14, 12), 45);
  var r2px = Math.min(Math.max(Math.sqrt(m2) * 14, 12), 45);

  var b1x = margin + col.b1.x * trackW;
  var b2x = margin + col.b2.x * trackW;

  function drawBall(cx, cy, r, color, label) {
    ctx.save();
    ctx.shadowColor = color; ctx.shadowBlur = 22;
    var g = ctx.createRadialGradient(cx - r * 0.3, cy - r * 0.3, 0, cx, cy, r);
    g.addColorStop(0, color + 'ee'); g.addColorStop(1, color + '44');
    ctx.fillStyle = g;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#000'; ctx.font = 'bold 11px DM Sans';
    ctx.textAlign = 'center'; ctx.fillText(label, cx, cy + 4);
    ctx.restore();
  }

  drawBall(b1x, surfY - r1px, r1px, '#00d4ff', m1 + 'kg');
  drawBall(b2x, surfY - r2px, r2px, '#ff6b35', m2 + 'kg');

  /* Velocity arrows */
  var v1now = col.b1.v / 0.05, v2now = col.b2.v / 0.05;
  var sc = 4;
  if (Math.abs(v1now) > 0.05)
    drawArrow(ctx, b1x, surfY - r1px, b1x + v1now * sc, surfY - r1px, '#00d4ff', fmtSci(v1now) + ' m/s', 2);
  if (Math.abs(v2now) > 0.05)
    drawArrow(ctx, b2x, surfY - r2px, b2x + v2now * sc, surfY - r2px, '#ff6b35', fmtSci(v2now) + ' m/s', 2);

  /* Momentum arrows (below balls) */
  var p1 = m1 * v1now, p2 = m2 * v2now;
  var psc = 0.5;
  if (Math.abs(p1) > 0.05)
    drawArrow(ctx, b1x, surfY + 18, b1x + p1 * psc, surfY + 18, 'rgba(0,212,255,0.45)', 'p1', 1.5);
  if (Math.abs(p2) > 0.05)
    drawArrow(ctx, b2x, surfY + 18, b2x + p2 * psc, surfY + 18, 'rgba(255,107,53,0.45)', 'p2', 1.5);

  /* Phase / type label */
  var typeLabel = type === 'perfect' ? 'PERFECTLY INELASTIC' :
                  type === 'elastic' ? 'ELASTIC' : 'INELASTIC';
  ctx.fillStyle = col.phase === 'post' ? '#a8ff3e' : '#6b7a99';
  ctx.font = 'bold 11px Space Mono';
  ctx.textAlign = 'left';
  ctx.fillText(typeLabel + ' — ' + col.phase.toUpperCase(), margin, surfY + 40);

  /* KE comparison bars */
  var KEnow  = 0.5 * m1 * v1now * v1now + 0.5 * m2 * v2now * v2now;
  var KEinit = 0.5 * m1 * v1i  * v1i   + 0.5 * m2 * v2i  * v2i;
  var barY = H * 0.77, bh = 14, maxW = trackW / 2 - 24;
  var KEmax = Math.max(KEnow, KEinit, 1);

  ctx.fillStyle = 'rgba(10,13,20,0.75)'; ctx.fillRect(margin, barY - 18, trackW, bh + 26);
  ctx.fillStyle = 'rgba(107,122,153,0.55)'; ctx.font = '10px Space Mono';
  ctx.fillText('KE before: ' + fmtSci(KEinit) + ' J', margin + 4, barY - 4);
  ctx.fillText('KE now: '   + fmtSci(KEnow)  + ' J', margin + maxW + 28, barY - 4);

  ctx.fillStyle = 'rgba(30,42,66,0.6)'; ctx.fillRect(margin, barY, maxW, bh);
  ctx.fillStyle = 'rgba(0,212,255,0.5)';
  ctx.fillRect(margin, barY, Math.min(KEinit / KEmax * maxW, maxW), bh);

  ctx.fillStyle = 'rgba(30,42,66,0.6)'; ctx.fillRect(margin + maxW + 24, barY, maxW, bh);
  ctx.fillStyle = col.phase === 'post' ? 'rgba(168,255,62,0.5)' : 'rgba(0,212,255,0.5)';
  ctx.fillRect(margin + maxW + 24, barY, Math.min(KEnow / KEmax * maxW, maxW), bh);

  /* e label if inelastic */
  if (type === 'inelastic') {
    var eVal = getNullable('col-e');
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono';
    ctx.fillText('e = ' + (eVal !== null ? eVal : '?'), margin + trackW - 80, barY - 4);
  }
}

/* ============================================================
   CSV EXPORT
   ============================================================ */
function exportCollisionCSV() {
  if (!col.data || col.data.length === 0) {
    alert('Run the simulation first to generate data.'); return;
  }
  exportCSV('collision_data.csv',
    ['time_s','x1_norm','v1_ms','x2_norm','v2_ms','KE_J'],
    col.data);
}
