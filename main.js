/* ============================================================
   main.js — PhysicsLab Core Engine
   Handles: canvas setup, animation loop, tab switching,
   simulation control (play/pause/step/reset), keyboard shortcuts
   ============================================================ */

'use strict';

/* ------------------------------------------------------------
   CANVAS SETUP
   ------------------------------------------------------------ */
var canvas = document.getElementById('simCanvas');
var ctx    = canvas.getContext('2d');

/** Resize canvas to fill its container (called on load + resize). */
function resizeCanvas() {
  var container = canvas.parentElement;
  canvas.width  = container.clientWidth;
  canvas.height = container.clientHeight;
}
window.addEventListener('resize', function() {
  resizeCanvas();
  drawCurrentSim();
});

/* ------------------------------------------------------------
   GLOBAL SIMULATION STATE
   ------------------------------------------------------------ */
var currentSim  = 'projectile'; // which simulation is active
var animFrame   = null;         // requestAnimationFrame handle
var simRunning  = false;        // is the simulation running?
var simTime     = 0;            // elapsed simulation time (s)
var lastTime    = null;         // previous RAF timestamp (ms)

/* Step-through mode: when stepping frame-by-frame */
var STEP_DT = 1 / 60;          // fixed delta-time for manual steps (s)

/* ------------------------------------------------------------
   SIMULATION LOOP
   ------------------------------------------------------------ */

/**
 * Main animation loop — called every frame by requestAnimationFrame.
 * @param {number} ts - timestamp in milliseconds
 */
function simLoop(ts) {
  if (!simRunning) return;

  // Compute delta-time, capped at 50 ms to prevent spiral-of-death
  // when tab is hidden or frame rate drops.
  if (!lastTime) lastTime = ts;
  var dt = Math.min((ts - lastTime) / 1000, 0.05);
  lastTime = ts;
  simTime += dt;

  // Step the active simulation
  stepCurrentSim(dt);

  // Render
  drawCurrentSim();

  // Schedule next frame
  animFrame = requestAnimationFrame(simLoop);
}

/**
 * Dispatch a physics step to the active simulation module.
 * @param {number} dt - delta time in seconds
 */
function stepCurrentSim(dt) {
  if (currentSim === 'projectile') stepProjectile(dt);
  if (currentSim === 'collision')  stepCollision(dt);
  if (currentSim === 'pendulum')   stepPendulum(dt);
  if (currentSim === 'work')       stepWork(dt);
}

/** Dispatch draw to the active simulation module. */
function drawCurrentSim() {
  if (currentSim === 'projectile') drawProjectile();
  if (currentSim === 'collision')  drawCollision();
  if (currentSim === 'pendulum')   drawPendulum();
  if (currentSim === 'work')       drawWork();
}

/* ------------------------------------------------------------
   PLAYBACK CONTROLS
   ------------------------------------------------------------ */

/** Start or resume the simulation. */
function startSim() {
  simRunning = true;
  lastTime   = null;
  setStatus('running');
  if (animFrame) cancelAnimationFrame(animFrame);
  animFrame = requestAnimationFrame(simLoop);
}

/** Pause without resetting. */
function pauseSim() {
  simRunning = false;
  setStatus('paused');
}

/** Stop and reset time. */
function stopSim() {
  simRunning = false;
  setStatus('stopped');
  if (animFrame) { cancelAnimationFrame(animFrame); animFrame = null; }
}

/**
 * Full reset: stop simulation, reset time, call the active
 * sim's reset function, redraw.
 */
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
 * Advance the simulation by exactly one fixed frame (STEP_DT).
 * This allows frame-by-frame inspection of the physics.
 * The simulation must be paused first.
 */
function stepFrame() {
  if (simRunning) { pauseSim(); return; }

  // If we're fully stopped, restart state first
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

/** Toggle play/pause — called by the main button. */
function handleMainBtn() {
  if (simRunning) pauseSim();
  else            startSim();
}

/* ------------------------------------------------------------
   STATUS BADGE
   ------------------------------------------------------------ */

/**
 * Update the status badge and main button label.
 * @param {'running'|'paused'|'stopped'} state
 */
function setStatus(state) {
  var badge = document.getElementById('statusBadge');
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

/**
 * Switch the active simulation tab.
 * Stops the current sim, shows the correct sidebar controls,
 * and resets.
 * @param {string} name - 'projectile' | 'collision' | 'pendulum' | 'work'
 */
function switchSim(name) {
  stopSim();
  currentSim = name;

  // Update tab button active state
  var names = ['projectile', 'collision', 'pendulum', 'work'];
  document.querySelectorAll('.sim-tab').forEach(function(t, i) {
    t.classList.toggle('active', names[i] === name);
  });

  // Show/hide the relevant sidebar section
  names.forEach(function(n) {
    var el = document.getElementById('ctrl-' + n);
    if (el) el.classList.toggle('hidden', n !== name);
  });

  // Close mobile sidebar if open
  closeMobileSidebar();

  resetSim();
}

/* ------------------------------------------------------------
   CSV EXPORT DISPATCH
   ------------------------------------------------------------ */

/** Export data for the currently active simulation. */
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
  // Don't fire when typing in an input
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

  switch (e.key) {
    case ' ':
      e.preventDefault();
      handleMainBtn();
      break;
    case 'r': case 'R':
      resetSim();
      break;
    case 'ArrowRight':
      // Step one frame forward
      stepFrame();
      break;
    case '1': switchSim('projectile'); break;
    case '2': switchSim('collision');  break;
    case '3': switchSim('pendulum');   break;
    case '4': switchSim('work');       break;
  }
});

/* ------------------------------------------------------------
   INITIALISE
   ------------------------------------------------------------ */
resizeCanvas();
resetProjectile();
updateCollision();
resetPendulum();
updateWork();
drawProjectile();
initTooltips();
setStatus('stopped');
