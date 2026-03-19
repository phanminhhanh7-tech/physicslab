/* ============================================================
   main.js — PhysicsLab Core Engine
   Handles: canvas setup, animation loop, tab switching,
   simulation control (play/pause/step/reset), keyboard shortcuts

   ANIMATION DESIGN:
   - requestAnimationFrame loop with real DOMHighResTimeStamp dt
   - dt clamped between MIN_DT (0.5ms) and MAX_DT (33ms)
   - First-frame skip prevents a zero-dt initialisation glitch
   - lastTime reset on every stop/pause so resuming is clean
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   CANVAS SETUP
   ------------------------------------------------------------ */
var canvas = document.getElementById('simCanvas');
var ctx    = canvas.getContext('2d');

function resizeCanvas() {
  var container = canvas.parentElement;
  canvas.width  = container.clientWidth;
  canvas.height = container.clientHeight;
}
window.addEventListener('resize', function() { resizeCanvas(); drawCurrentSim(); });

/* ------------------------------------------------------------
   GLOBAL SIMULATION STATE
   ------------------------------------------------------------ */
var currentSim = 'projectile';
var animFrame  = null;
var simRunning = false;
var simTime    = 0;
var lastTime   = null;

/* Fixed dt for manual step-through (1 frame @ 60 fps) */
var STEP_DT = 1 / 60;

/* Animation dt limits:
   MAX_DT: if a frame takes > 33 ms (tab hidden, slow device)
           skip rather than send a huge dt that tunnels objects.
   MIN_DT: skip the very first RAF tick where lastTime === ts. */
var MAX_DT = 0.033;
var MIN_DT = 0.0005;

/* ------------------------------------------------------------
   ANIMATION LOOP
   ------------------------------------------------------------ */

/**
 * RAF callback.
 * Order per frame:  compute dt → step physics → render → queue next.
 */
function simLoop(ts) {
  if (!simRunning) return;

  /* First frame after start: record time and skip stepping. */
  if (lastTime === null) {
    lastTime = ts;
    animFrame = requestAnimationFrame(simLoop);
    return;
  }

  var dt = (ts - lastTime) / 1000;
  lastTime = ts;

  /* Reject frames that are too tiny or suspiciously large. */
  if (dt < MIN_DT || dt > MAX_DT) {
    animFrame = requestAnimationFrame(simLoop);
    return;
  }

  simTime += dt;

  stepCurrentSim(dt);   /* physics update   */
  drawCurrentSim();     /* render           */

  animFrame = requestAnimationFrame(simLoop);
}

function stepCurrentSim(dt) {
  if (currentSim === 'projectile') stepProjectile(dt);
  if (currentSim === 'collision')  stepCollision(dt);
  if (currentSim === 'pendulum')   stepPendulum(dt);
  if (currentSim === 'work')       stepWork(dt);
}

function drawCurrentSim() {
  if (currentSim === 'projectile') drawProjectile();
  if (currentSim === 'collision')  drawCollision();
  if (currentSim === 'pendulum')   drawPendulum();
  if (currentSim === 'work')       drawWork();
}

/* ------------------------------------------------------------
   PLAYBACK CONTROLS
   ------------------------------------------------------------ */

function startSim() {
  simRunning = true;
  lastTime   = null;      /* skip first frame cleanly */
  setStatus('running');
  if (animFrame) cancelAnimationFrame(animFrame);
  animFrame = requestAnimationFrame(simLoop);
}

function pauseSim() {
  simRunning = false;
  lastTime   = null;      /* avoid stale timestamp on resume */
  setStatus('paused');
}

function stopSim() {
  simRunning = false;
  lastTime   = null;
  setStatus('stopped');
  if (animFrame) { cancelAnimationFrame(animFrame); animFrame = null; }
}

function resetSim() {
  stopSim();
  simTime  = 0;
  lastTime = null;
  if (currentSim === 'projectile') resetProjectile();
  if (currentSim === 'collision')  resetCollision();
  if (currentSim === 'pendulum')   resetPendulum();
  if (currentSim === 'work')       resetWork();
  updateInfoBar(0, 0, 0, 0);
  drawCurrentSim();
}

/**
 * Step one fixed frame forward — useful for frame-by-frame inspection.
 * Pauses running sim if called while running.
 */
function stepFrame() {
  if (simRunning) { pauseSim(); return; }
  if (simTime === 0) {
    if (currentSim === 'projectile') resetProjectile();
    if (currentSim === 'collision')  resetCollision();
    if (currentSim === 'pendulum')   resetPendulum();
    if (currentSim === 'work')       resetWork();
  }
  simTime += STEP_DT;
  stepCurrentSim(STEP_DT);
  drawCurrentSim();
  setStatus('paused');
}

function handleMainBtn() {
  if (simRunning) pauseSim();
  else            startSim();
}

/* ------------------------------------------------------------
   STATUS BADGE
   ------------------------------------------------------------ */
function setStatus(state) {
  var badge = document.getElementById('statusBadge');
  if (!badge) return;
  badge.className = 'status-badge ' + state;
  badge.innerHTML = '<span class="status-dot"></span> ' + state.toUpperCase();
  var btn = document.getElementById('mainBtn');
  if (!btn) return;
  if (state === 'running') {
    btn.textContent = '⏸ Pause';
    btn.className   = 'btn';
  } else {
    btn.textContent = '▶ Start';
    btn.className   = 'btn primary';
  }
}

/* ------------------------------------------------------------
   TAB SWITCHING
   ------------------------------------------------------------ */
function switchSim(name) {
  stopSim();
  currentSim = name;
  var names = ['projectile', 'collision', 'pendulum', 'work'];
  document.querySelectorAll('.sim-tab').forEach(function(t, i) {
    t.classList.toggle('active', names[i] === name);
  });
  names.forEach(function(n) {
    var el = document.getElementById('ctrl-' + n);
    if (el) el.classList.toggle('hidden', n !== name);
  });
  closeMobileSidebar();
  resetSim();
}

/* ------------------------------------------------------------
   CSV EXPORT DISPATCH
   ------------------------------------------------------------ */
function exportCurrentCSV() {
  if (currentSim === 'projectile') exportProjectileCSV();
  if (currentSim === 'collision')  exportCollisionCSV();
  if (currentSim === 'pendulum')   exportPendulumCSV();
  if (currentSim === 'work')       exportWorkCSV();
}

/* ------------------------------------------------------------
   MOBILE SIDEBAR
   ------------------------------------------------------------ */
function openMobileSidebar() {
  document.getElementById('sidebar').classList.add('open');
  document.getElementById('sidebarOverlay').classList.add('visible');
}
function closeMobileSidebar() {
  document.getElementById('sidebar').classList.remove('open');
  document.getElementById('sidebarOverlay').classList.remove('visible');
}

/* ------------------------------------------------------------
   KEYBOARD SHORTCUTS
   ------------------------------------------------------------ */
document.addEventListener('keydown', function(e) {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  switch (e.key) {
    case ' ':           e.preventDefault(); handleMainBtn(); break;
    case 'r': case 'R': resetSim(); break;
    case 'ArrowRight':  stepFrame(); break;
    case '1': switchSim('projectile'); break;
    case '2': switchSim('collision');  break;
    case '3': switchSim('pendulum');   break;
    case '4': switchSim('work');       break;
  }
});

/* ------------------------------------------------------------
   INITIALISE

   Each update/reset function calls its own draw at the end.
   We suppress those draws during init by temporarily patching
   simRunning = false + only calling drawCurrentSim once at the
   very end, after all state is set up.

   The key rule: currentSim = 'projectile' from the start,
   so only drawProjectile should fire at init time.
   ------------------------------------------------------------ */
resizeCanvas();

/* Silence all draw calls during reset/update — we'll draw once at the end */
var _initDone = false;
var _origDrawProjectile  = null; /* unused — projectile IS the active sim */

/* Block the inactive sims from painting during their reset */
function _noop() {}
var _realDrawCollision = drawCollision;
var _realDrawPendulum  = drawPendulum;
var _realDrawWork      = drawWork;
drawCollision = _noop;
drawPendulum  = _noop;
drawWork      = _noop;

resetProjectile();   /* active sim — its draw is still live  */
updateCollision();   /* sets solver state only               */
resetPendulum();     /* would draw pendulum — suppressed      */
updateWork();        /* would draw work — suppressed          */

/* Restore real draw functions */
drawCollision = _realDrawCollision;
drawPendulum  = _realDrawPendulum;
drawWork      = _realDrawWork;

/* Single clean draw of the active (projectile) tab */
drawCurrentSim();
initTooltips();
setStatus('stopped');
