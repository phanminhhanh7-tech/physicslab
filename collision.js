/* ============================================================
   collision.js — True Algebraic Collision Solver v4
   
   DESIGN PHILOSOPHY:
   This solver does NOT just evaluate forward formulas.
   Each physics law is represented as a RELATION that can be
   rearranged to solve for ANY variable it contains.
   
   When a variable appears on BOTH sides of an equation
   (e.g. m1 in momentum), the solver groups like terms and
   isolates it algebraically before computing.
   
   Physics laws implemented:
   (L1) Momentum:   m1*v1i + m2*v2i = m1*v1f + m2*v2f
   (L2) Restitution: e = (v2f - v1f) / (v1i - v2i)
   (L3) Elastic KE:  ½m1*v1i² + ½m2*v2i² = ½m1*v1f² + ½m2*v2f²
   (L4) Perfect:     vf = (m1*v1i + m2*v2i) / (m1 + m2)
   
   Collision types:
     elastic    → L1 + L3 (or equivalently L1 + L2 with e=1)
     inelastic  → L1 + L2 (user supplies e)
     perfect    → L4 (v1f = v2f = vf)
   ============================================================ */

'use strict';

/* ============================================================
   ANIMATION STATE
   ============================================================ */
var col = {
  b1: { x: 0.12, v: 0 },
  b2: { x: 0.78, v: 0 },
  collided: false,
  phase: 'pre',
  data: [],
  _m1: 5, _m2: 3, _v1i: 8, _v2i: -3,
  _v1f: null, _v2f: null, _type: 'elastic'
};

function resetCollision() {
  var prev = { m1: col._m1, m2: col._m2, v1i: col._v1i, v2i: col._v2i };
  col = {
    b1: { x: 0.12, v: 0 }, b2: { x: 0.78, v: 0 },
    collided: false, phase: 'pre', data: [],
    _m1: prev.m1, _m2: prev.m2, _v1i: prev.v1i, _v2i: prev.v2i,
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
   ALGEBRAIC SOLVER ENGINE
   
   Core idea: each law is encoded as a set of EXPLICIT ALGEBRAIC
   REARRANGEMENTS — one for each variable it can solve for.
   
   For laws where the target variable appears on both sides
   (e.g. momentum solving for m1), we derive the rearrangement
   analytically before coding it, showing the algebra in comments.
   ============================================================ */

/**
 * solveCollision — true algebraic solver
 *
 * @param {object} K    - known values dict, null = unknown
 * @param {string} type - 'elastic' | 'inelastic' | 'perfect'
 * @returns {{ state, log, derived, warnings }}
 */
function solveCollision(K, type) {

  /* Working state — clone so we never mutate user inputs */
  var S = {};
  var ALL_VARS = ['m1','m2','v1i','v2i','v1f','v2f','vf','e',
                  'KEb','KEa','KElost','pt','p1i','p2i'];
  ALL_VARS.forEach(function(k) {
    S[k] = (K[k] !== undefined && K[k] !== null) ? K[k] : null;
  });

  var log     = [];   /* Step records shown in solver panel */
  var derived = {};   /* Keys filled by solver (not user) */
  var EPS     = 1e-12; /* Zero guard */

  /* ---- Helper: record a solved step ---- */
  function set(key, val, eqName, formula, subFn) {
    if (!isFinite(val)) return; /* Reject NaN / Infinity */
    S[key] = val;
    derived[key] = true;
    log.push({
      eqName:    eqName,
      formula:   formula,
      varSolved: key,
      value:     val,
      buildSub:  subFn
    });
  }

  /* ---- Shorthand checks ---- */
  function has(k) { return S[k] !== null && S[k] !== undefined; }
  function miss(k){ return !has(k); }
  function all()  {
    for (var i = 0; i < arguments.length; i++) if (!has(arguments[i])) return false;
    return true;
  }

  /* ============================================================
     PASS LOOP — repeat until no new values are found
     Up to 40 passes ensures multi-step chains always complete.
     ============================================================ */
  var changed = true, pass = 0;
  while (changed && pass < 40) {
    changed = false;
    pass++;

    /* ----------------------------------------------------------
       MODULE 1 — MOMENTUM CONSERVATION
       
       Law: m1*v1i + m2*v2i = m1*v1f + m2*v2f
       Rewrite as: m1*(v1i - v1f) = m2*(v2f - v2i)   ... (*)
       
       This single rewrite lets us isolate ANY variable:
         m1  = m2*(v2f - v2i) / (v1i - v1f)
         m2  = m1*(v1i - v1f) / (v2f - v2i)
         v1i = [m1*v1f + m2*v2f - m2*v2i] / m1
             = v1f + m2*(v2f - v2i)/m1
         v2i = [m1*v1i + m2*v2i_term]... use pt form:
             = (pt - m1*v1f - m2*v2f + m2*v2i... better via pt
         v1f = v1i - m2*(v2f - v2i)/m1
         v2f = v2i + m1*(v1i - v1f)/m2
       ---------------------------------------------------------- */

    /* 1a. Total momentum pt = m1*v1i + m2*v2i */
    if (miss('pt') && all('m1','v1i','m2','v2i')) {
      set('pt', S.m1*S.v1i + S.m2*S.v2i,
        'Momentum (before)',
        'pt = m1·v1i + m2·v2i',
        function(s){ return 'pt = '+fvN(s.m1)+'×'+fvN(s.v1i)+' + '+fvN(s.m2)+'×'+fvN(s.v2i)+' = '+fmtSci(s.pt)+' kg·m/s'; });
      changed = true;
    }

    /* 1b. pt also = m1*v1f + m2*v2f (momentum conserved) */
    if (miss('pt') && all('m1','v1f','m2','v2f')) {
      set('pt', S.m1*S.v1f + S.m2*S.v2f,
        'Momentum (after, conserved)',
        'pt = m1·v1f + m2·v2f',
        function(s){ return 'pt = '+fvN(s.m1)+'×'+fvN(s.v1f)+' + '+fvN(s.m2)+'×'+fvN(s.v2f)+' = '+fmtSci(s.pt)+' kg·m/s'; });
      changed = true;
    }

    /* 1c. p1i = m1*v1i */
    if (miss('p1i') && all('m1','v1i')) {
      set('p1i', S.m1*S.v1i, 'p1 initial', 'p1i = m1·v1i',
        function(s){ return 'p1i = '+fvN(s.m1)+'×'+fvN(s.v1i)+' = '+fmtSci(s.p1i)+' kg·m/s'; });
      changed = true;
    }

    /* 1d. p2i = m2*v2i */
    if (miss('p2i') && all('m2','v2i')) {
      set('p2i', S.m2*S.v2i, 'p2 initial', 'p2i = m2·v2i',
        function(s){ return 'p2i = '+fvN(s.m2)+'×'+fvN(s.v2i)+' = '+fmtSci(s.p2i)+' kg·m/s'; });
      changed = true;
    }

    /* 1e. pt from p1i + p2i */
    if (miss('pt') && all('p1i','p2i')) {
      set('pt', S.p1i+S.p2i, 'Total momentum', 'pt = p1i + p2i',
        function(s){ return 'pt = '+fmtSci(s.p1i)+' + '+fmtSci(s.p2i)+' = '+fmtSci(s.pt)+' kg·m/s'; });
      changed = true;
    }

    /* 1f. Solve v1i from pt, m1, m2, v2i
       From: pt = m1*v1i + m2*v2i
       → v1i = (pt - m2*v2i) / m1                            */
    if (miss('v1i') && all('pt','m1','m2','v2i') && Math.abs(S.m1)>EPS) {
      set('v1i', (S.pt - S.m2*S.v2i)/S.m1,
        'Momentum → v1i', 'v1i = (pt − m2·v2i) / m1',
        function(s){ return 'v1i = ('+fmtSci(s.pt)+' − '+fvN(s.m2)+'×'+fvN(s.v2i)+') / '+fvN(s.m1)+' = '+fmtSci(s.v1i)+' m/s'; });
      changed = true;
    }

    /* 1g. Solve v2i from pt, m1, m2, v1i */
    if (miss('v2i') && all('pt','m1','m2','v1i') && Math.abs(S.m2)>EPS) {
      set('v2i', (S.pt - S.m1*S.v1i)/S.m2,
        'Momentum → v2i', 'v2i = (pt − m1·v1i) / m2',
        function(s){ return 'v2i = ('+fmtSci(s.pt)+' − '+fvN(s.m1)+'×'+fvN(s.v1i)+') / '+fvN(s.m2)+' = '+fmtSci(s.v2i)+' m/s'; });
      changed = true;
    }

    /* 1h. Solve v1f from pt, m1, m2, v2f
       From: pt = m1*v1f + m2*v2f
       → v1f = (pt - m2*v2f) / m1                            */
    if (miss('v1f') && all('pt','m1','m2','v2f') && Math.abs(S.m1)>EPS) {
      set('v1f', (S.pt - S.m2*S.v2f)/S.m1,
        'Momentum → v1f', 'v1f = (pt − m2·v2f) / m1',
        function(s){ return 'v1f = ('+fmtSci(s.pt)+' − '+fvN(s.m2)+'×'+fvN(s.v2f)+') / '+fvN(s.m1)+' = '+fmtSci(s.v1f)+' m/s'; });
      changed = true;
    }

    /* 1i. Solve v2f from pt, m1, m2, v1f */
    if (miss('v2f') && all('pt','m1','m2','v1f') && Math.abs(S.m2)>EPS) {
      set('v2f', (S.pt - S.m1*S.v1f)/S.m2,
        'Momentum → v2f', 'v2f = (pt − m1·v1f) / m2',
        function(s){ return 'v2f = ('+fmtSci(s.pt)+' − '+fvN(s.m1)+'×'+fvN(s.v1f)+') / '+fvN(s.m2)+' = '+fmtSci(s.v2f)+' m/s'; });
      changed = true;
    }

    /* 1j. Solve m1 using the regrouped form (*):
       m1*(v1i - v1f) = m2*(v2f - v2i)
       → m1 = m2*(v2f - v2i) / (v1i - v1f)
       
       Algebra: start with m1*v1i + m2*v2i = m1*v1f + m2*v2f
         m1*v1i - m1*v1f = m2*v2f - m2*v2i
         m1*(v1i - v1f)  = m2*(v2f - v2i)
         m1 = m2*(v2f - v2i)/(v1i - v1f)               */
    if (miss('m1') && all('m2','v1i','v1f','v2i','v2f')) {
      var d_m1 = S.v1i - S.v1f;
      if (Math.abs(d_m1) > EPS) {
        var m1_val = S.m2*(S.v2f - S.v2i) / d_m1;
        if (m1_val > 0) {
          set('m1', m1_val,
            'Momentum regrouped → m1',
            'm1·(v1i−v1f) = m2·(v2f−v2i)  →  m1 = m2(v2f−v2i)/(v1i−v1f)',
            function(s){ return 'm1 = '+fvN(s.m2)+'×('+fvN(s.v2f)+'−'+fvN(s.v2i)+')/('+fvN(s.v1i)+'−'+fvN(s.v1f)+') = '+fmtSci(s.m1)+' kg'; });
          changed = true;
        }
      }
    }

    /* 1k. Solve m2 by symmetry:
       m2*(v2f - v2i) = m1*(v1i - v1f)
       → m2 = m1*(v1i - v1f) / (v2f - v2i)              */
    if (miss('m2') && all('m1','v1i','v1f','v2i','v2f')) {
      var d_m2 = S.v2f - S.v2i;
      if (Math.abs(d_m2) > EPS) {
        var m2_val = S.m1*(S.v1i - S.v1f) / d_m2;
        if (m2_val > 0) {
          set('m2', m2_val,
            'Momentum regrouped → m2',
            'm2·(v2f−v2i) = m1·(v1i−v1f)  →  m2 = m1(v1i−v1f)/(v2f−v2i)',
            function(s){ return 'm2 = '+fvN(s.m1)+'×('+fvN(s.v1i)+'−'+fvN(s.v1f)+')/('+fvN(s.v2f)+'−'+fvN(s.v2i)+') = '+fmtSci(s.m2)+' kg'; });
          changed = true;
        }
      }
    }

    /* 1l. m1 from pt and v1i when m2 and v2i known (indirect)
       pt = m1*v1i + m2*v2i  →  m1 = (pt - m2*v2i)/v1i      */
    if (miss('m1') && all('pt','v1i','m2','v2i') && Math.abs(S.v1i)>EPS) {
      var m1_p = (S.pt - S.m2*S.v2i)/S.v1i;
      if (m1_p > 0) {
        set('m1', m1_p, 'Momentum → m1', 'm1 = (pt − m2·v2i) / v1i',
          function(s){ return 'm1 = ('+fmtSci(s.pt)+'−'+fvN(s.m2)+'×'+fvN(s.v2i)+')/'+fvN(s.v1i)+' = '+fmtSci(s.m1)+' kg'; });
        changed = true;
      }
    }

    /* 1m. m2 from pt and v2i when m1 and v1i known */
    if (miss('m2') && all('pt','v2i','m1','v1i') && Math.abs(S.v2i)>EPS) {
      var m2_p = (S.pt - S.m1*S.v1i)/S.v2i;
      if (m2_p > 0) {
        set('m2', m2_p, 'Momentum → m2', 'm2 = (pt − m1·v1i) / v2i',
          function(s){ return 'm2 = ('+fmtSci(s.pt)+'−'+fvN(s.m1)+'×'+fvN(s.v1i)+')/'+fvN(s.v2i)+' = '+fmtSci(s.m2)+' kg'; });
        changed = true;
      }
    }

    /* ----------------------------------------------------------
       MODULE 2 — COEFFICIENT OF RESTITUTION
       
       Law: e = (v2f - v1f) / (v1i - v2i)
       
       Rearrangements:
         v2f - v1f = e*(v1i - v2i)      ... define rel = e*(v1i-v2i)
         v2f = v1f + e*(v1i - v2i)
         v1f = v2f - e*(v1i - v2i)
         v1i - v2i = (v2f - v1f)/e      (if e ≠ 0)
         v1i = v2i + (v2f - v1f)/e
         v2i = v1i - (v2f - v1f)/e
       ---------------------------------------------------------- */

    /* 2a. Compute e from all four velocities */
    if (miss('e') && all('v1i','v2i','v1f','v2f')) {
      var dvi = S.v1i - S.v2i;
      if (Math.abs(dvi) > EPS) {
        var e_val = (S.v2f - S.v1f) / dvi;
        /* e must be 0–1 physically */
        if (e_val >= -EPS && e_val <= 1+EPS) {
          set('e', Math.max(0, Math.min(1, e_val)),
            'Restitution definition', 'e = (v2f − v1f) / (v1i − v2i)',
            function(s){ return 'e = ('+fvN(s.v2f)+'−'+fvN(s.v1f)+')/('+fvN(s.v1i)+'−'+fvN(s.v2i)+') = '+fmtSci(s.e); });
          changed = true;
        }
      }
    }

    /* 2b. e = 1 for elastic (always) */
    if (type === 'elastic' && miss('e')) {
      set('e', 1, 'Elastic collision', 'e = 1 (elastic definition)',
        function(){ return 'e = 1 (kinetic energy is conserved in elastic collisions)'; });
      changed = true;
    }

    /* 2c. e = 0 for perfectly inelastic */
    if (type === 'perfect' && miss('e')) {
      set('e', 0, 'Perfectly inelastic', 'e = 0 (objects stick together)',
        function(){ return 'e = 0 (no relative velocity after collision)'; });
      changed = true;
    }

    /* 2d. v2f from v1f, e, v1i, v2i:
       v2f = v1f + e*(v1i - v2i)                          */
    if (miss('v2f') && all('v1f','e','v1i','v2i')) {
      set('v2f', S.v1f + S.e*(S.v1i - S.v2i),
        'Restitution → v2f', 'v2f = v1f + e·(v1i − v2i)',
        function(s){ return 'v2f = '+fvN(s.v1f)+' + '+fvN(s.e,3)+'×('+fvN(s.v1i)+'−'+fvN(s.v2i)+') = '+fmtSci(s.v2f)+' m/s'; });
      changed = true;
    }

    /* 2e. v1f from v2f, e, v1i, v2i:
       v1f = v2f - e*(v1i - v2i)                          */
    if (miss('v1f') && all('v2f','e','v1i','v2i')) {
      set('v1f', S.v2f - S.e*(S.v1i - S.v2i),
        'Restitution → v1f', 'v1f = v2f − e·(v1i − v2i)',
        function(s){ return 'v1f = '+fvN(s.v2f)+' − '+fvN(s.e,3)+'×('+fvN(s.v1i)+'−'+fvN(s.v2i)+') = '+fmtSci(s.v1f)+' m/s'; });
      changed = true;
    }

    /* 2f. v1i from e, v1f, v2f, v2i:
       e*(v1i - v2i) = v2f - v1f
       v1i - v2i = (v2f - v1f)/e
       v1i = v2i + (v2f - v1f)/e                         */
    if (miss('v1i') && all('e','v1f','v2f','v2i') && Math.abs(S.e)>EPS) {
      set('v1i', S.v2i + (S.v2f - S.v1f)/S.e,
        'Restitution → v1i', 'v1i = v2i + (v2f − v1f) / e',
        function(s){ return 'v1i = '+fvN(s.v2i)+' + ('+fvN(s.v2f)+'−'+fvN(s.v1f)+')/'+fvN(s.e,3)+' = '+fmtSci(s.v1i)+' m/s'; });
      changed = true;
    }

    /* 2g. v2i from e, v1f, v2f, v1i:
       v2i = v1i - (v2f - v1f)/e                         */
    if (miss('v2i') && all('e','v1f','v2f','v1i') && Math.abs(S.e)>EPS) {
      set('v2i', S.v1i - (S.v2f - S.v1f)/S.e,
        'Restitution → v2i', 'v2i = v1i − (v2f − v1f) / e',
        function(s){ return 'v2i = '+fvN(s.v1i)+' − ('+fvN(s.v2f)+'−'+fvN(s.v1f)+')/'+fvN(s.e,3)+' = '+fmtSci(s.v2i)+' m/s'; });
      changed = true;
    }

    /* 2h. SIMULTANEOUS: solve v1i when v1f, e, m1, m2, v2i are known
       From momentum + restitution (eliminating v2f):
         v2f = v1f + e*(v1i - v2i)   (restitution)
         m1*v1i + m2*v2i = m1*v1f + m2*v2f   (momentum)
       Substitute v2f:
         m1*v1i + m2*v2i = m1*v1f + m2*(v1f + e*(v1i-v2i))
         m1*v1i + m2*v2i = (m1+m2)*v1f + m2*e*v1i - m2*e*v2i
         v1i*(m1 - m2*e) = (m1+m2)*v1f - m2*v2i*(1+e)
         v1i = [(m1+m2)*v1f - m2*v2i*(1+e)] / (m1 - m2*e)         */
    if (miss('v1i') && all('v1f','e','m1','m2','v2i')) {
      var den_2h = S.m1 - S.m2 * S.e;
      if (Math.abs(den_2h) > EPS) {
        var v1i_2h = ((S.m1+S.m2)*S.v1f - S.m2*S.v2i*(1+S.e)) / den_2h;
        set('v1i', v1i_2h,
          'Momentum + restitution → v1i',
          'v1i = [(m1+m2)v1f − m2·v2i·(1+e)] / (m1 − m2·e)',
          function(s){ return 'v1i = [('+fvN(s.m1)+'+'+fvN(s.m2)+')×'+fvN(s.v1f)+' − '+fvN(s.m2)+'×'+fvN(s.v2i)+'×(1+'+fvN(s.e,2)+')] / ('+fvN(s.m1)+'−'+fvN(s.m2)+'×'+fvN(s.e,2)+') = '+fmtSci(s.v1i)+' m/s'; });
        changed = true;
      }
    }

    /* 2i. SIMULTANEOUS: solve v2i when v2f, e, m1, m2, v1i are known
       By symmetry of the above derivation:
         v1f = v2f - e*(v1i - v2i)   (restitution)
         m1*v1i + m2*v2i = m1*v1f + m2*v2f
       Substitute v1f:
         m1*v1i + m2*v2i = m1*(v2f-e*(v1i-v2i)) + m2*v2f
         m1*v1i + m2*v2i = (m1+m2)*v2f - m1*e*v1i + m1*e*v2i
         v2i*(m2 - m1*e) = (m1+m2)*v2f - m1*v1i*(1+e)
         v2i = [(m1+m2)*v2f - m1*v1i*(1+e)] / (m2 - m1*e)
       
       But if m2 = m1*e exactly (degenerate), use the ALTERNATE form:
       From momentum: m1*v1i + m2*v2i = m1*v1f + m2*v2f  — rearrange for v2i:
         m2*(v2i - v2f) = m1*(v1f - v1i)
         We also know v1f = v2f - e*(v1i - v2i) from restitution.
         Substituting & collecting gives the same degeneracy.
       Alternate: given v1f is derivable from v2f+e+v1i once v2i is known,
         use e*(v1i-v2i) = v2f-v1f combined with momentum in symmetric form:
         v2i = [m1*(1+e)*v1i - (m1+m2)*v2f] / (m1*e - m2)           */
    if (miss('v2i') && all('v2f','e','m1','m2','v1i')) {
      var den_2i = S.m2 - S.m1 * S.e;
      var den_2i_alt = S.m1 * S.e - S.m2; /* same magnitude, opposite sign */
      if (Math.abs(den_2i) > EPS) {
        var v2i_2i = ((S.m1+S.m2)*S.v2f - S.m1*S.v1i*(1+S.e)) / den_2i;
        set('v2i', v2i_2i,
          'Momentum + restitution → v2i',
          'v2i = [(m1+m2)v2f − m1·v1i·(1+e)] / (m2 − m1·e)',
          function(s){ return 'v2i = [('+fvN(s.m1)+'+'+fvN(s.m2)+')×'+fvN(s.v2f)+' − '+fvN(s.m1)+'×'+fvN(s.v1i)+'×(1+'+fvN(s.e,2)+')] / ('+fvN(s.m2)+'−'+fvN(s.m1)+'×'+fvN(s.e,2)+') = '+fmtSci(s.v2i)+' m/s'; });
        changed = true;
      } else if (Math.abs(den_2i_alt) > EPS) {
        /* alternate sign form: v2i = [m1*(1+e)*v1i - (m1+m2)*v2f] / (m1*e - m2) */
        var v2i_alt = (S.m1*(1+S.e)*S.v1i - (S.m1+S.m2)*S.v2f) / den_2i_alt;
        set('v2i', v2i_alt,
          'Momentum + restitution → v2i (alt)',
          'v2i = [m1(1+e)v1i − (m1+m2)v2f] / (m1·e − m2)',
          function(s){ return 'v2i = ['+fvN(s.m1)+'×(1+'+fvN(s.e,2)+')×'+fvN(s.v1i)+' − ('+fvN(s.m1)+'+'+fvN(s.m2)+')×'+fvN(s.v2f)+'] / ('+fvN(s.m1)+'×'+fvN(s.e,2)+'−'+fvN(s.m2)+') = '+fmtSci(s.v2i)+' m/s'; });
        changed = true;
      }
    }

    /* 2j. SIMULTANEOUS: solve v1i when v1f, e, m1, m2, v2i known
       Using: m1*v1i*(1+e) - (m1+m2)*v1f = m2*v2i*(1+e) - (m1+m2)*v1f... 
       From earlier derivation:
         v1i*(m1 - m2*e) = (m1+m2)*v1f - m2*v2i*(1+e)
         v1i = [(m1+m2)*v1f - m2*v2i*(1+e)] / (m1 - m2*e)
       Alternate form when m1 = m2*e:
         v1i = [(m1+m2)*v1f - m2*v2i*(1+e)] / (m1 - m2*e)
         → expand to remove degeneracy via the other form:
         v1i*(m1 - m2*e) = (m1+m2)*v1f - m2*(1+e)*v2i
       When denom=0: m1=m2*e, substitute back—system is underdetermined
       unless we also use the restitution direction:
         Additional route: v1i from restitution (2f) if v2i known.                */
    if (miss('v1i') && all('v1f','e','m1','m2','v2i')) {
      var den_2j = S.m1 - S.m2 * S.e;
      if (Math.abs(den_2j) > EPS) {
        var v1i_2j = ((S.m1+S.m2)*S.v1f - S.m2*S.v2i*(1+S.e)) / den_2j;
        set('v1i', v1i_2j,
          'Momentum + restitution → v1i',
          'v1i = [(m1+m2)v1f − m2·v2i·(1+e)] / (m1 − m2·e)',
          function(s){ return 'v1i = [('+fvN(s.m1)+'+'+fvN(s.m2)+')×'+fvN(s.v1f)+' − '+fvN(s.m2)+'×'+fvN(s.v2i)+'×(1+'+fvN(s.e,2)+')] / ('+fvN(s.m1)+'−'+fvN(s.m2)+'×'+fvN(s.e,2)+') = '+fmtSci(s.v1i)+' m/s'; });
        changed = true;
      }
    }

    /* 2k. Solve v2i given v1i, e, v1f, v2f — purely from restitution, no mass needed
       e*(v1i - v2i) = v2f - v1f
       v2i = v1i - (v2f - v1f)/e     [already in 2g but keep as fallback]       */
    /* Already covered by 2g above */

    /* 2l. Degenerate inelastic: when m1*e = m2, use KE equation to find v2i
       At this degeneracy point (e.g. e=0.6, m1=5, m2=3),
       both v1f and v2f are uniquely determined — but v2i is ambiguous
       unless an additional constraint (KEb or pt) is given.
       Rule: if pt is known, use: v2i = (pt - m1*v1i) / m2               */
    if (miss('v2i') && all('pt','m1','v1i','m2') && Math.abs(S.m2) > EPS) {
      set('v2i', (S.pt - S.m1*S.v1i)/S.m2,
        'Momentum → v2i', 'v2i = (pt − m1·v1i) / m2',
        function(s){ return 'v2i = ('+fmtSci(s.pt)+' − '+fvN(s.m1)+'×'+fvN(s.v1i)+') / '+fvN(s.m2)+' = '+fmtSci(s.v2i)+' m/s'; });
      changed = true;
    }
    if (miss('v1i') && all('pt','m2','v2i','m1') && Math.abs(S.m1) > EPS) {
      set('v1i', (S.pt - S.m2*S.v2i)/S.m1,
        'Momentum → v1i (from pt)', 'v1i = (pt − m2·v2i) / m1',
        function(s){ return 'v1i = ('+fmtSci(s.pt)+' − '+fvN(s.m2)+'×'+fvN(s.v2i)+') / '+fvN(s.m1)+' = '+fmtSci(s.v1i)+' m/s'; });
      changed = true;
    }

    /* ----------------------------------------------------------
       MODULE 3 — SIMULTANEOUS SYSTEM: MOMENTUM + RESTITUTION
       
       When both v1f AND v2f are unknown, we need to solve:
         (I)  m1*v1f + m2*v2f = pt
         (II) v2f - v1f = e*(v1i - v2i)   [let rel = e*(v1i-v2i)]
       
       Substitution method:
         From (II): v2f = v1f + rel
         Sub into (I): m1*v1f + m2*(v1f + rel) = pt
                       v1f*(m1 + m2) = pt - m2*rel
                       v1f = (pt - m2*rel) / (m1 + m2)
                       v2f = v1f + rel
       ---------------------------------------------------------- */
    if (miss('v1f') && miss('v2f') && all('pt','m1','m2','e','v1i','v2i')) {
      var mt  = S.m1 + S.m2;
      if (Math.abs(mt) > EPS) {
        var rel  = S.e * (S.v1i - S.v2i);
        var v1f_ = (S.pt - S.m2*rel) / mt;
        var v2f_ = v1f_ + rel;
        set('v1f', v1f_,
          'Simultaneous (momentum + restitution) → v1f',
          'v1f = (pt − m2·e·Δv) / (m1+m2)',
          function(s){ return 'Let rel=e×(v1i−v2i)='+fmtSci(s.e*(s.v1i-s.v2i))+'. v1f=('+fmtSci(s.pt)+'−'+fvN(s.m2)+'×rel)/('+fvN(s.m1)+'+'+fvN(s.m2)+') = '+fmtSci(s.v1f)+' m/s'; });
        set('v2f', v2f_,
          'Simultaneous (momentum + restitution) → v2f',
          'v2f = v1f + e·(v1i − v2i)',
          function(s){ return 'v2f = '+fmtSci(s.v1f)+' + '+fmtSci(s.e*(s.v1i-s.v2i))+' = '+fmtSci(s.v2f)+' m/s'; });
        changed = true;
      }
    }

    /* ----------------------------------------------------------
       MODULE 4 — ELASTIC KE CONSERVATION
       
       Law: ½m1*v1i² + ½m2*v2i² = ½m1*v1f² + ½m2*v2f²
       
       Rearrangements:
         m1*(v1i²-v1f²) = m2*(v2f²-v2i²)    ... (factored form)
         → m1*(v1i-v1f)*(v1i+v1f) = m2*(v2f-v2i)*(v2f+v2i)
       
       Combined with the momentum regrouped form:
         m1*(v1i-v1f) = m2*(v2f-v2i)
       
       Dividing the KE equation by momentum equation:
         v1i+v1f = v2f+v2i
         → This is the RELATIVE VELOCITY equation for elastic:
           v1i - v2i = -(v1f - v2f)   (same as e=1)
       
       So for elastic we just use MODULE 3 with e=1.
       Below we add direct KE derivations for KEb / KEa.
       ---------------------------------------------------------- */
    if (type === 'elastic' || type === 'inelastic') {
      /* KEb */
      if (miss('KEb') && all('m1','v1i','m2','v2i')) {
        set('KEb', 0.5*S.m1*S.v1i*S.v1i + 0.5*S.m2*S.v2i*S.v2i,
          'KE before', 'KEb = ½m1v1i² + ½m2v2i²',
          function(s){ return 'KEb = ½×'+fvN(s.m1)+'×'+fvN(s.v1i)+'² + ½×'+fvN(s.m2)+'×'+fvN(s.v2i)+'² = '+fmtSci(s.KEb)+' J'; });
        changed = true;
      }
      /* KEa */
      if (miss('KEa') && all('m1','v1f','m2','v2f')) {
        set('KEa', 0.5*S.m1*S.v1f*S.v1f + 0.5*S.m2*S.v2f*S.v2f,
          'KE after', 'KEa = ½m1v1f² + ½m2v2f²',
          function(s){ return 'KEa = ½×'+fvN(s.m1)+'×'+fvN(s.v1f)+'² + ½×'+fvN(s.m2)+'×'+fvN(s.v2f)+'² = '+fmtSci(s.KEa)+' J'; });
        changed = true;
      }

      /* Solve v1i from KE (elastic only, when m1, v1f, m2, v2i, v2f known)
         From KE factored: m1*(v1i²-v1f²) = m2*(v2f²-v2i²)
         v1i² = v1f² + m2*(v2f²-v2i²)/m1                   */
      if (type === 'elastic' && miss('v1i') && all('m1','v1f','m2','v2i','v2f')) {
        var disc1 = S.v1f*S.v1f + S.m2*(S.v2f*S.v2f - S.v2i*S.v2i)/S.m1;
        if (disc1 >= 0) {
          /* Pick the physically meaningful root: v1i ≠ v1f (non-trivial solution) */
          var v1i_ke = Math.sqrt(disc1);
          /* Check both signs — use the one consistent with restitution e=1 */
          var cand = [v1i_ke, -v1i_ke];
          for (var ci = 0; ci < cand.length; ci++) {
            var dvi_check = cand[ci] - S.v2i;
            if (Math.abs(dvi_check) > EPS) {
              var e_check = (S.v2f - S.v1f) / dvi_check;
              if (Math.abs(e_check - 1) < 0.01) {
                set('v1i', cand[ci],
                  'KE (elastic) → v1i',
                  'v1i = √(v1f² + m2(v2f²−v2i²)/m1)',
                  function(s){ return 'v1i² = '+fvN(s.v1f)+'² + '+fvN(s.m2)+'/'+fvN(s.m1)+'×('+fvN(s.v2f)+'²−'+fvN(s.v2i)+'²) → v1i = '+fmtSci(s.v1i)+' m/s'; });
                changed = true;
                break;
              }
            }
          }
        }
      }

      /* Solve m1 from KE (elastic):
         m1*(v1i²-v1f²) = m2*(v2f²-v2i²)
         m1 = m2*(v2f²-v2i²)/(v1i²-v1f²)                   */
      if (type === 'elastic' && miss('m1') && all('m2','v1i','v1f','v2i','v2f')) {
        var dv1sq = S.v1i*S.v1i - S.v1f*S.v1f;
        if (Math.abs(dv1sq) > EPS) {
          var m1_ke = S.m2*(S.v2f*S.v2f - S.v2i*S.v2i) / dv1sq;
          if (m1_ke > 0) {
            set('m1', m1_ke,
              'KE (elastic) → m1',
              'm1 = m2·(v2f²−v2i²) / (v1i²−v1f²)',
              function(s){ return 'm1 = '+fvN(s.m2)+'×('+fvN(s.v2f)+'²−'+fvN(s.v2i)+'²)/('+fvN(s.v1i)+'²−'+fvN(s.v1f)+'²) = '+fmtSci(s.m1)+' kg'; });
            changed = true;
          }
        }
      }

      /* Solve m2 from KE (elastic):
         m2 = m1*(v1i²-v1f²) / (v2f²-v2i²)                 */
      if (type === 'elastic' && miss('m2') && all('m1','v1i','v1f','v2i','v2f')) {
        var dv2sq = S.v2f*S.v2f - S.v2i*S.v2i;
        if (Math.abs(dv2sq) > EPS) {
          var m2_ke = S.m1*(S.v1i*S.v1i - S.v1f*S.v1f) / dv2sq;
          if (m2_ke > 0) {
            set('m2', m2_ke,
              'KE (elastic) → m2',
              'm2 = m1·(v1i²−v1f²) / (v2f²−v2i²)',
              function(s){ return 'm2 = '+fvN(s.m1)+'×('+fvN(s.v1i)+'²−'+fvN(s.v1f)+'²)/('+fvN(s.v2f)+'²−'+fvN(s.v2i)+'²) = '+fmtSci(s.m2)+' kg'; });
            changed = true;
          }
        }
      }
    }

    /* ----------------------------------------------------------
       MODULE 5 — PERFECTLY INELASTIC
       
       Law: vf = (m1*v1i + m2*v2i) / (m1 + m2)  =  pt / (m1+m2)
       
       Rearrangements:
         vf*(m1+m2) = pt
         m1+m2      = pt/vf
         m1         = pt/vf - m2
         m2         = pt/vf - m1
         pt         = vf*(m1+m2)   (already covered by momentum)
         v1i (given m2,v2i,vf,m1):
           m1*v1i = vf*(m1+m2) - m2*v2i
           v1i = [vf*(m1+m2) - m2*v2i] / m1
         v2i by symmetry
       ---------------------------------------------------------- */
    if (type === 'perfect') {

      /* 5a. vf = pt/(m1+m2) */
      if (miss('vf') && all('pt','m1','m2')) {
        var mt5 = S.m1+S.m2;
        if (Math.abs(mt5) > EPS) {
          set('vf', S.pt/mt5,
            'Perfectly inelastic → vf', 'vf = pt / (m1+m2)',
            function(s){ return 'vf = '+fmtSci(s.pt)+' / ('+fvN(s.m1)+'+'+fvN(s.m2)+') = '+fmtSci(s.vf)+' m/s'; });
          changed = true;
        }
      }

      /* 5b. v1f = v2f = vf */
      if (miss('v1f') && has('vf')) {
        set('v1f', S.vf, 'Objects stick → v1f = vf', 'v1f = vf',
          function(s){ return 'v1f = vf = '+fmtSci(s.vf)+' m/s'; });
        changed = true;
      }
      if (miss('v2f') && has('vf')) {
        set('v2f', S.vf, 'Objects stick → v2f = vf', 'v2f = vf',
          function(s){ return 'v2f = vf = '+fmtSci(s.vf)+' m/s'; });
        changed = true;
      }

      /* 5c. Solve m1 from vf:
         vf*(m1+m2) = m1*v1i + m2*v2i
         vf*m1 + vf*m2 = m1*v1i + m2*v2i
         Group m1 terms: m1*vf - m1*v1i = m2*v2i - vf*m2
         m1*(vf - v1i) = m2*(v2i - vf)
         m1 = m2*(v2i - vf) / (vf - v1i)                   */
      if (miss('m1') && all('vf','m2','v1i','v2i')) {
        var d5m1 = S.vf - S.v1i;
        if (Math.abs(d5m1) > EPS) {
          var m1_5 = S.m2*(S.v2i - S.vf) / d5m1;
          if (m1_5 > 0) {
            set('m1', m1_5,
              'Perfect inelastic → m1',
              'm1·(vf−v1i) = m2·(v2i−vf)  →  m1 = m2(v2i−vf)/(vf−v1i)',
              function(s){ return 'm1 = '+fvN(s.m2)+'×('+fvN(s.v2i)+'−'+fmtSci(s.vf)+')/('+fmtSci(s.vf)+'−'+fvN(s.v1i)+') = '+fmtSci(s.m1)+' kg'; });
            changed = true;
          }
        }
      }

      /* 5d. Solve m2 by symmetry */
      if (miss('m2') && all('vf','m1','v1i','v2i')) {
        var d5m2 = S.vf - S.v2i;
        if (Math.abs(d5m2) > EPS) {
          var m2_5 = S.m1*(S.v1i - S.vf) / d5m2;
          if (m2_5 > 0) {
            set('m2', m2_5,
              'Perfect inelastic → m2',
              'm2·(vf−v2i) = m1·(v1i−vf)  →  m2 = m1(v1i−vf)/(vf−v2i)',
              function(s){ return 'm2 = '+fvN(s.m1)+'×('+fvN(s.v1i)+'−'+fmtSci(s.vf)+')/('+fmtSci(s.vf)+'−'+fvN(s.v2i)+') = '+fmtSci(s.m2)+' kg'; });
            changed = true;
          }
        }
      }

      /* 5e. Solve v1i:
         m1*v1i = vf*(m1+m2) - m2*v2i
         v1i = [vf*(m1+m2) - m2*v2i] / m1                  */
      if (miss('v1i') && all('vf','m1','m2','v2i') && Math.abs(S.m1)>EPS) {
        set('v1i', (S.vf*(S.m1+S.m2) - S.m2*S.v2i)/S.m1,
          'Perfect inelastic → v1i',
          'v1i = [vf·(m1+m2) − m2·v2i] / m1',
          function(s){ return 'v1i = ['+fmtSci(s.vf)+'×('+fvN(s.m1)+'+'+fvN(s.m2)+') − '+fvN(s.m2)+'×'+fvN(s.v2i)+'] / '+fvN(s.m1)+' = '+fmtSci(s.v1i)+' m/s'; });
        changed = true;
      }

      /* 5f. Solve v2i:
         v2i = [vf*(m1+m2) - m1*v1i] / m2                  */
      if (miss('v2i') && all('vf','m1','m2','v1i') && Math.abs(S.m2)>EPS) {
        set('v2i', (S.vf*(S.m1+S.m2) - S.m1*S.v1i)/S.m2,
          'Perfect inelastic → v2i',
          'v2i = [vf·(m1+m2) − m1·v1i] / m2',
          function(s){ return 'v2i = ['+fmtSci(s.vf)+'×'+fvN(s.m1+s.m2)+' − '+fvN(s.m1)+'×'+fvN(s.v1i)+'] / '+fvN(s.m2)+' = '+fmtSci(s.v2i)+' m/s'; });
        changed = true;
      }

      /* 5g. KEa uses the stuck vf */
      if (miss('KEa') && all('m1','m2','vf')) {
        set('KEa', 0.5*(S.m1+S.m2)*S.vf*S.vf,
          'KE after (perfectly inelastic)', 'KEa = ½(m1+m2)vf²',
          function(s){ return 'KEa = ½×'+fvN(s.m1+s.m2)+'×'+fmtSci(s.vf)+'² = '+fmtSci(s.KEa)+' J'; });
        changed = true;
      }

      /* 5h. KEb for perfect inelastic */
      if (miss('KEb') && all('m1','v1i','m2','v2i')) {
        set('KEb', 0.5*S.m1*S.v1i*S.v1i + 0.5*S.m2*S.v2i*S.v2i,
          'KE before', 'KEb = ½m1v1i² + ½m2v2i²',
          function(s){ return 'KEb = ½×'+fvN(s.m1)+'×'+fvN(s.v1i)+'² + ½×'+fvN(s.m2)+'×'+fvN(s.v2i)+'² = '+fmtSci(s.KEb)+' J'; });
        changed = true;
      }
    }

    /* ----------------------------------------------------------
       MODULE 6 — KE BOOKKEEPING (all types)
       ---------------------------------------------------------- */

    /* 6a. KElost = KEb - KEa */
    if (miss('KElost') && all('KEb','KEa')) {
      set('KElost', S.KEb-S.KEa, 'KE lost', 'KElost = KEb − KEa',
        function(s){ return 'KElost = '+fmtSci(s.KEb)+' − '+fmtSci(s.KEa)+' = '+fmtSci(s.KElost)+' J'; });
      changed = true;
    }
    /* 6b. KEb from KElost + KEa */
    if (miss('KEb') && all('KElost','KEa')) {
      set('KEb', S.KEa+S.KElost, 'KEb from KElost', 'KEb = KEa + KElost',
        function(s){ return 'KEb = '+fmtSci(s.KEa)+' + '+fmtSci(s.KElost)+' = '+fmtSci(s.KEb)+' J'; });
      changed = true;
    }
    /* 6c. KEa from KEb - KElost */
    if (miss('KEa') && all('KEb','KElost')) {
      set('KEa', S.KEb-S.KElost, 'KEa from KElost', 'KEa = KEb − KElost',
        function(s){ return 'KEa = '+fmtSci(s.KEb)+' − '+fmtSci(s.KElost)+' = '+fmtSci(s.KEa)+' J'; });
      changed = true;
    }

    /* 6d. Recover a velocity from KEb if only one mass/v is missing
       KEb = ½m1v1i² + ½m2v2i²
       → v1i = √[(KEb - ½m2v2i²) × 2/m1]                 */
    if (miss('v1i') && all('KEb','m1','m2','v2i') && Math.abs(S.m1)>EPS) {
      var rem1 = S.KEb - 0.5*S.m2*S.v2i*S.v2i;
      if (rem1 >= 0) {
        set('v1i', Math.sqrt(2*rem1/S.m1),
          'KE → v1i', 'v1i = √[2(KEb − ½m2v2i²)/m1]',
          function(s){ return 'v1i = √[2×('+fmtSci(s.KEb)+'−½×'+fvN(s.m2)+'×'+fvN(s.v2i)+'²)/'+fvN(s.m1)+'] = '+fmtSci(s.v1i)+' m/s'; });
        changed = true;
      }
    }
    if (miss('v2i') && all('KEb','m1','m2','v1i') && Math.abs(S.m2)>EPS) {
      var rem2 = S.KEb - 0.5*S.m1*S.v1i*S.v1i;
      if (rem2 >= 0) {
        set('v2i', Math.sqrt(2*rem2/S.m2),
          'KE → v2i', 'v2i = √[2(KEb − ½m1v1i²)/m2]',
          function(s){ return 'v2i = √[2×('+fmtSci(s.KEb)+'−½×'+fvN(s.m1)+'×'+fvN(s.v1i)+'²)/'+fvN(s.m2)+'] = '+fmtSci(s.v2i)+' m/s'; });
        changed = true;
      }
    }

    /* 6e. m1 from KEb:
       m1 = 2*(KEb - ½m2v2i²) / v1i²                      */
    if (miss('m1') && all('KEb','m2','v2i','v1i') && Math.abs(S.v1i)>EPS) {
      var m1_ke2 = 2*(S.KEb - 0.5*S.m2*S.v2i*S.v2i)/(S.v1i*S.v1i);
      if (m1_ke2 > 0) {
        set('m1', m1_ke2, 'KE → m1', 'm1 = 2(KEb − ½m2v2i²)/v1i²',
          function(s){ return 'm1 = 2×('+fmtSci(s.KEb)+'−½×'+fvN(s.m2)+'×'+fvN(s.v2i)+'²)/'+fvN(s.v1i)+'² = '+fmtSci(s.m1)+' kg'; });
        changed = true;
      }
    }
    if (miss('m2') && all('KEb','m1','v1i','v2i') && Math.abs(S.v2i)>EPS) {
      var m2_ke2 = 2*(S.KEb - 0.5*S.m1*S.v1i*S.v1i)/(S.v2i*S.v2i);
      if (m2_ke2 > 0) {
        set('m2', m2_ke2, 'KE → m2', 'm2 = 2(KEb − ½m1v1i²)/v2i²',
          function(s){ return 'm2 = 2×('+fmtSci(s.KEb)+'−½×'+fvN(s.m1)+'×'+fvN(s.v1i)+'²)/'+fvN(s.v2i)+'² = '+fmtSci(s.m2)+' kg'; });
        changed = true;
      }
    }

  } /* end while loop */

  /* ----------------------------------------------------------
     PHYSICAL VALIDATION
     ---------------------------------------------------------- */
  var warnings = [];
  if (has('m1') && S.m1 <= 0) warnings.push('m1 must be > 0');
  if (has('m2') && S.m2 <= 0) warnings.push('m2 must be > 0');
  if (has('e')  && (S.e < -EPS || S.e > 1+EPS))
    warnings.push('e must be 0 ≤ e ≤ 1');
  if (has('KEa') && has('KEb') && S.KEa > S.KEb + 0.01)
    warnings.push('KE after > KE before — check your inputs');
  if (type === 'perfect' && has('v1f') && has('v2f') && Math.abs(S.v1f-S.v2f) > 0.01)
    warnings.push('Perfectly inelastic requires v1f = v2f');

  return { state: S, log: log, derived: derived, warnings: warnings };
}

/* ============================================================
   METADATA
   ============================================================ */
var COL_META = {
  m1:     { label:'m1',            unit:'kg',     card:'crc-m1',     val:'col-r-m1'  },
  m2:     { label:'m2',            unit:'kg',     card:'crc-m2',     val:'col-r-m2'  },
  v1i:    { label:'v1 initial',    unit:'m/s',    card:'crc-v1i',    val:'col-r-v1i' },
  v2i:    { label:'v2 initial',    unit:'m/s',    card:'crc-v2i',    val:'col-r-v2i' },
  v1f:    { label:'v1 final',      unit:'m/s',    card:'crc-v1f',    val:'col-r-v1f' },
  v2f:    { label:'v2 final',      unit:'m/s',    card:'crc-v2f',    val:'col-r-v2f' },
  vf:     { label:'vf (stuck)',    unit:'m/s',    card:'crc-vf',     val:'col-r-vf'  },
  e:      { label:'Restitution e', unit:'',       card:'crc-e',      val:'col-r-e'   },
  pt:     { label:'p total',       unit:'kg·m/s', card:'crc-pt',     val:'col-r-pt'  },
  p1i:    { label:'p1 initial',    unit:'kg·m/s', card:'crc-p1i',    val:'col-r-p1i' },
  p2i:    { label:'p2 initial',    unit:'kg·m/s', card:'crc-p2i',    val:'col-r-p2i' },
  KEb:    { label:'KE before',     unit:'J',      card:'crc-KEb',    val:'col-r-keb' },
  KEa:    { label:'KE after',      unit:'J',      card:'crc-KEa',    val:'col-r-kea' },
  KElost: { label:'KE lost',       unit:'J',      card:'crc-KElost', val:'col-r-kel' }
};

/* ============================================================
   UPDATE — read inputs, run solver, update UI
   ============================================================ */
function updateCollision() {
  var type = document.getElementById('col-type').value;

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

  /* For perfectly inelastic: propagate vf → v1f, v2f */
  if (type === 'perfect' && known.vf !== null) {
    if (known.v1f === null) known.v1f = known.vf;
    if (known.v2f === null) known.v2f = known.vf;
  }

  /* Validation */
  showInputError('col-m1', validateInput(known.m1, { positive: true, label: 'm1' }));
  showInputError('col-m2', validateInput(known.m2, { positive: true, label: 'm2' }));
  showInputError('col-e',  validateInput(known.e,  { min: 0, max: 1, label: 'e'  }));

  var userKeys = {};
  Object.keys(known).forEach(function(k) { if (known[k] !== null) userKeys[k] = true; });

  var solveFor = document.getElementById('col-solveFor').value;

  /* Run the algebraic solver */
  var result = solveCollision(known, type);
  var S = result.state;

  /* Update result cards */
  Object.keys(COL_META).forEach(function(k) {
    var meta  = COL_META[k];
    var valEl = document.getElementById(meta.val);
    if (!valEl) return;
    var val = S[k];
    valEl.textContent = (val !== null && val !== undefined)
      ? fmtSci(val) + (meta.unit ? ' ' + meta.unit : '') : '--';
    var st = (k === solveFor) ? 'target'
           : (userKeys[k] ? 'user' : (val !== null ? 'derived' : ''));
    styleCard(meta.card, st);
  });

  /* Solver steps panel */
  renderSolverSteps('col-solver-steps', result.log, result.derived, S, solveFor,
    'Enter any known variables.\nSolver rearranges physics equations algebraically.');

  /* Warnings */
  var warnEl = document.getElementById('col-warnings');
  if (warnEl) {
    warnEl.style.display = result.warnings.length > 0 ? 'block' : 'none';
    warnEl.textContent = result.warnings.length > 0 ? '⚠ ' + result.warnings.join(' | ') : '';
  }

  /* Cache for animation */
  col._m1   = S.m1  || 5;
  col._m2   = S.m2  || 3;
  col._v1i  = S.v1i !== null ? S.v1i : 8;
  col._v2i  = S.v2i !== null ? S.v2i : -3;
  col._v1f  = S.v1f;
  col._v2f  = S.v2f;
  col._type = type;

  if (!simRunning) {
    col.b1 = { x: 0.12, v: col._v1i * 0.05 };
    col.b2 = { x: 0.78, v: col._v2i * 0.05 };
    col.phase = 'pre'; col.collided = false;
    drawCollision();
  }
}

/* ============================================================
   PHYSICS ENGINE — updateObject + handleWallCollision
   
   Design:
   - Positions are stored in PIXEL space (px) not normalised 0-1,
     so radius comparisons are exact with no unit conversion bugs.
   - Each frame: move → wall resolve → object-object resolve → record
   - Wall resolution clamps position AND checks velocity direction
     before reversing (prevents double-reverse jitter).
   - Object-object collision uses a single-collision flag per
     approach, reset when objects separate, so re-collisions work.
   ============================================================ */

var WALL_RESTITUTION = 0.82; /* energy kept on wall bounce (0–1) */
var VEL_THRESHOLD    = 0.02; /* m/s — zero out crawling velocity  */

/**
 * updateObject — move a single ball by dt seconds.
 * Position is in pixels; velocity in pixels/second.
 * @param {{ x:number, vPx:number }} obj
 * @param {number} dt  delta-time in seconds
 */
function updateObject(obj, dt) {
  obj.x += obj.vPx * dt;
}

/**
 * handleWallCollision — resolve a ball against left/right walls.
 * 
 * Algorithm:
 *   1. Check if ball edge crossed the wall.
 *   2. If so, CLAMP position back to the wall surface (no clipping).
 *   3. Reverse velocity, apply restitution.
 *   4. Only reverse if ball is still moving INTO the wall (prevents
 *      double-reverse when a ball is nudged out by position correction).
 *   5. Kill crawl: if |v| < threshold after bounce, set v = 0.
 *
 * @param {{ x:number, vPx:number }} obj
 * @param {number} r    ball radius in pixels
 * @param {number} Wmin left boundary in pixels  (usually margin)
 * @param {number} Wmax right boundary in pixels (usually margin + trackW)
 */
function handleWallCollision(obj, r, Wmin, Wmax) {
  /* ---- Left wall ---- */
  if (obj.x - r < Wmin) {
    obj.x = Wmin + r;                /* clamp: push back inside     */
    if (obj.vPx < 0) {               /* only reverse if moving left */
      obj.vPx = -obj.vPx * WALL_RESTITUTION;
      if (Math.abs(obj.vPx) < VEL_THRESHOLD * 50) obj.vPx = 0; /* kill crawl */
    }
  }

  /* ---- Right wall ---- */
  if (obj.x + r > Wmax) {
    obj.x = Wmax - r;                /* clamp: push back inside     */
    if (obj.vPx > 0) {               /* only reverse if moving right*/
      obj.vPx = -obj.vPx * WALL_RESTITUTION;
      if (Math.abs(obj.vPx) < VEL_THRESHOLD * 50) obj.vPx = 0;
    }
  }
}

/* ============================================================
   ANIMATION STEP
   ============================================================ */
function stepCollision(dt) {
  var m1   = col._m1 || 5,  m2   = col._m2 || 3;
  var v1i  = col._v1i || 8, v2i  = col._v2i || -3;
  var type = col._type || 'elastic';

  /* Canvas geometry — computed each frame so resize is handled */
  var margin = 60;
  var trackW = canvas.width - margin * 2;
  var Wmin   = margin;
  var Wmax   = margin + trackW;

  /* Pixel radii — must match what drawCollision draws */
  var r1px = Math.min(Math.max(Math.sqrt(m1) * 14, 12), 45);
  var r2px = Math.min(Math.max(Math.sqrt(m2) * 14, 12), 45);

  /* ---- Initialise on first frame ---- */
  if (simTime <= dt + 0.001) {
    /* Convert physical velocities to pixel/s using a consistent scale:
       1 m/s in physics = 40 px/s on screen (same as old 0.05 factor: px_norm/s * trackW ≈ 40px/s) */
    var PX_PER_MS = 40;
    col.b1 = {
      x:    Wmin + 0.12 * trackW,   /* start at 12% of track */
      vPx:  v1i * PX_PER_MS
    };
    col.b2 = {
      x:    Wmin + 0.78 * trackW,   /* start at 78% of track */
      vPx:  v2i * PX_PER_MS
    };
    col.collided    = false;
    col.separating  = false;        /* tracks whether balls are moving apart */
    col.phase       = 'pre';
    col.data        = [];
  }

  /* ---- Step 1: Move both objects ---- */
  updateObject(col.b1, dt);
  updateObject(col.b2, dt);

  /* ---- Step 2: Resolve wall collisions ---- */
  handleWallCollision(col.b1, r1px, Wmin, Wmax);
  handleWallCollision(col.b2, r2px, Wmin, Wmax);

  /* ---- Step 3: Object-object collision ----
     
     Use surface-to-surface distance = |b2.x - b1.x| - (r1 + r2).
     Negative → overlapping.
     
     We allow RE-COLLISION: once balls separate (gap > 0) the flag resets,
     so if a ball bounces back from a wall and hits again, it collides again.
  */
  var gap = (col.b2.x - col.b1.x) - (r1px + r2px);

  /* Reset collision flag when balls have cleanly separated */
  if (gap > 2) {
    col.collided = false;
  }

  if (!col.collided && gap <= 0) {
    /* Check that balls are actually approaching (relative velocity negative) */
    var relVel = col.b1.vPx - col.b2.vPx; /* positive = b1 chasing b2 */

    if (relVel > 0) {
      col.collided = true;
      col.phase    = 'post';

      /* Convert px/s → m/s for physics calculation */
      var PX_PER_MS = 40;
      var v1 = col.b1.vPx / PX_PER_MS;
      var v2 = col.b2.vPx / PX_PER_MS;
      var mt = m1 + m2;
      var v1f, v2f;

      if (type === 'elastic') {
        v1f = ((m1 - m2) * v1 + 2 * m2 * v2) / mt;
        v2f = (2 * m1 * v1 + (m2 - m1) * v2) / mt;
      } else if (type === 'perfect') {
        v1f = v2f = (m1 * v1 + m2 * v2) / mt;
      } else {
        /* Inelastic: use user-supplied e, default 0.5 */
        var e   = getNullable('col-e');
        if (e === null || e < 0 || e > 1) e = 0.5;
        var rel = e * (v1 - v2);
        v1f = (m1 * v1 + m2 * v2 - m2 * rel) / mt;
        v2f = v1f + rel;
      }

      /* Apply new velocities */
      col.b1.vPx = v1f * PX_PER_MS;
      col.b2.vPx = v2f * PX_PER_MS;

      /* ---- Position correction: push balls apart so they don't overlap ----
         Split the overlap proportionally by mass (heavier ball moves less) */
      var overlap = -gap; /* positive amount of overlap */
      var push1   = overlap * (m2 / mt); /* b1 pushed left  */
      var push2   = overlap * (m1 / mt); /* b2 pushed right */
      col.b1.x   -= push1;
      col.b2.x   += push2;

      /* Re-run wall clamp after position correction */
      handleWallCollision(col.b1, r1px, Wmin, Wmax);
      handleWallCollision(col.b2, r2px, Wmin, Wmax);
    }
  }

  /* ---- Step 4: Kill crawl — stop near-zero velocities ----
     Threshold in px/s = VEL_THRESHOLD m/s * 40 px/m/s */
  var vThreshPx = VEL_THRESHOLD * 40;
  if (Math.abs(col.b1.vPx) < vThreshPx) col.b1.vPx = 0;
  if (Math.abs(col.b2.vPx) < vThreshPx) col.b2.vPx = 0;

  /* ---- Step 5: Record CSV data ---- */
  var PX_PER_MS = 40;
  var v1n = col.b1.vPx / PX_PER_MS;
  var v2n = col.b2.vPx / PX_PER_MS;
  var KE  = 0.5 * m1 * v1n * v1n + 0.5 * m2 * v2n * v2n;

  /* Convert pixel position back to normalised 0-1 for CSV */
  var x1norm = (col.b1.x - Wmin) / trackW;
  var x2norm = (col.b2.x - Wmin) / trackW;

  col.data.push([
    +simTime.toFixed(3),
    +x1norm.toFixed(4), +v1n.toFixed(3),
    +x2norm.toFixed(4), +v2n.toFixed(3),
    +KE.toFixed(3)
  ]);

  updateInfoBar(simTime, x1norm, 0, Math.abs(v1n));
}

/* ============================================================
   DRAW
   ============================================================ */
function drawCollision() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#080b12'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);

  var m1   = col._m1  || 5,  m2   = col._m2  || 3;
  var v1i  = col._v1i || 8,  v2i  = col._v2i || -3;
  var type = col._type || 'elastic';

  var surfY  = H * 0.60;
  var margin = 60;
  var trackW = W - margin * 2;
  var Wmin   = margin;
  var Wmax   = margin + trackW;

  /* ---- Track surface ---- */
  ctx.strokeStyle = 'rgba(0,212,255,0.4)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(Wmin, surfY); ctx.lineTo(Wmax, surfY); ctx.stroke();

  /* ---- Wall markers ---- */
  ctx.strokeStyle = 'rgba(0,212,255,0.2)'; ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(Wmin, surfY - 60); ctx.lineTo(Wmin, surfY + 10); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(Wmax, surfY - 60); ctx.lineTo(Wmax, surfY + 10); ctx.stroke();
  ctx.setLineDash([]);

  /* Pixel radii */
  var r1px = Math.min(Math.max(Math.sqrt(m1) * 14, 12), 45);
  var r2px = Math.min(Math.max(Math.sqrt(m2) * 14, 12), 45);

  /* Ball positions come directly from pixel state */
  var b1x = col.b1.x;
  var b2x = col.b2.x;

  /* ---- Draw balls ---- */
  function drawBall(cx, cy, r, color, label) {
    ctx.save();
    ctx.shadowColor = color; ctx.shadowBlur = 22;
    var g = ctx.createRadialGradient(cx - r * 0.3, cy - r * 0.3, 0, cx, cy, r);
    g.addColorStop(0, color + 'ee'); g.addColorStop(1, color + '44');
    ctx.fillStyle = g;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#000'; ctx.font = 'bold 10px DM Sans';
    ctx.textAlign = 'center'; ctx.fillText(label, cx, cy + 4);
    ctx.restore();
  }

  drawBall(b1x, surfY - r1px, r1px, '#00d4ff', m1 + 'kg');
  drawBall(b2x, surfY - r2px, r2px, '#ff6b35', m2 + 'kg');

  /* ---- Velocity arrows ---- */
  var PX_PER_MS = 40;
  var v1n = col.b1.vPx / PX_PER_MS;
  var v2n = col.b2.vPx / PX_PER_MS;
  var sc  = 4; /* pixels per m/s for arrow length */

  if (Math.abs(v1n) > 0.05)
    drawArrow(ctx, b1x, surfY - r1px, b1x + v1n * sc, surfY - r1px,
              '#00d4ff', fmtSci(v1n) + ' m/s', 2);
  if (Math.abs(v2n) > 0.05)
    drawArrow(ctx, b2x, surfY - r2px, b2x + v2n * sc, surfY - r2px,
              '#ff6b35', fmtSci(v2n) + ' m/s', 2);

  /* ---- Momentum arrows (below track) ---- */
  var p1 = m1 * v1n, p2 = m2 * v2n;
  if (Math.abs(p1) > 0.1)
    drawArrow(ctx, b1x, surfY + 18, b1x + p1 * 0.5, surfY + 18,
              'rgba(0,212,255,0.4)', 'p1', 1.5);
  if (Math.abs(p2) > 0.1)
    drawArrow(ctx, b2x, surfY + 18, b2x + p2 * 0.5, surfY + 18,
              'rgba(255,107,53,0.4)', 'p2', 1.5);

  /* ---- Type + phase label ---- */
  var tLabel = type === 'perfect' ? 'PERFECTLY INELASTIC'
             : type === 'elastic' ? 'ELASTIC' : 'INELASTIC';
  ctx.fillStyle  = col.phase === 'post' ? '#a8ff3e' : '#6b7a99';
  ctx.font       = 'bold 11px Space Mono';
  ctx.textAlign  = 'left';
  ctx.fillText(tLabel + ' — ' + col.phase.toUpperCase(), Wmin, surfY + 40);

  /* ---- KE comparison bars ---- */
  var KEnow  = 0.5 * m1 * v1n * v1n + 0.5 * m2 * v2n * v2n;
  var KEinit = 0.5 * m1 * v1i * v1i + 0.5 * m2 * v2i * v2i;
  var barY   = H * 0.78;
  var bh     = 13;
  var maxW   = trackW / 2 - 24;
  var KEmax  = Math.max(KEnow, KEinit, 1);

  ctx.fillStyle = 'rgba(10,13,20,0.75)';
  ctx.fillRect(Wmin, barY - 18, trackW, bh + 26);
  ctx.fillStyle = 'rgba(107,122,153,0.5)'; ctx.font = '10px Space Mono';
  ctx.fillText('KEi: '   + fmtSci(KEinit) + ' J', Wmin + 4, barY - 4);
  ctx.fillText('KE now: ' + fmtSci(KEnow)  + ' J', Wmin + maxW + 28, barY - 4);

  /* Initial KE bar */
  ctx.fillStyle = 'rgba(30,42,66,0.6)';
  ctx.fillRect(Wmin, barY, maxW, bh);
  ctx.fillStyle = 'rgba(0,212,255,0.5)';
  ctx.fillRect(Wmin, barY, Math.min(KEinit / KEmax * maxW, maxW), bh);

  /* Current KE bar */
  ctx.fillStyle = 'rgba(30,42,66,0.6)';
  ctx.fillRect(Wmin + maxW + 24, barY, maxW, bh);
  ctx.fillStyle = col.phase === 'post' ? 'rgba(168,255,62,0.5)' : 'rgba(0,212,255,0.5)';
  ctx.fillRect(Wmin + maxW + 24, barY, Math.min(KEnow / KEmax * maxW, maxW), bh);

  /* e label for inelastic */
  if (type === 'inelastic') {
    var eV = getNullable('col-e');
    ctx.fillStyle = '#a8ff3e'; ctx.font = '11px Space Mono';
    ctx.fillText('e = ' + (eV !== null ? eV : '?'), Wmin + trackW - 80, barY - 4);
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
    ['time_s', 'x1_norm', 'v1_ms', 'x2_norm', 'v2_ms', 'KE_J'], col.data);
}
