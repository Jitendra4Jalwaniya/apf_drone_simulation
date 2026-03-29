'use strict';

// ═══════════════════════════════════════════════════
//  SCENE BOOTSTRAP
// ═══════════════════════════════════════════════════
var engine = APF.initEngine('canvas-container');
var renderer = engine.renderer;
var scene = engine.scene;
var camera = engine.camera;

var cam = APF.initCameraControls(renderer, camera);
var speedCtrl = APF.initSpeedControl('speed-input', 'speed-val', 1.0);

// ═══════════════════════════════════════════════════
//  ENVIRONMENT
// ═══════════════════════════════════════════════════
APF.setupLighting(scene);
var HALF = 5;
var CUBE = HALF * 2;
APF.setupCube(scene, HALF);

// ═══════════════════════════════════════════════════
//  START & GOAL MARKERS
// ═══════════════════════════════════════════════════
var START = new THREE.Vector3(-HALF + 0.4, -HALF + 0.4, -HALF + 0.4);
var GOAL  = new THREE.Vector3( HALF - 0.4,  HALF - 0.4,  HALF - 0.4);
var DIAG  = START.distanceTo(GOAL);

var startMarker = APF.makeMarker(scene, START, 0x44ff88, 0x1a7a3a, 0.22);
var goalMarker  = APF.makeMarker(scene, GOAL, 0xff4444, 0x992222, 0.28);
var goalRings   = APF.createGoalRings(scene, GOAL);
APF.createDashedLine(scene, START, GOAL, 0x223a5a, 0.4);

// ═══════════════════════════════════════════════════
//  DRONE
// ═══════════════════════════════════════════════════
var droneCfg = {
  hullColor:  0xdde8f5,
  domeColor:  0x2288ff,
  ledColor:   0x00ffff,
  lightColor: 0x44ccff
};
var droneObj = APF.buildDrone(droneCfg);
var droneGroup = droneObj.group;
var rotorMeshes = droneObj.rotors;
droneGroup.position.copy(START);
scene.add(droneGroup);

// ═══════════════════════════════════════════════════
//  OBSTACLES
// ═══════════════════════════════════════════════════
var NUM_OBS = 15;
var obstacles = APF.createMovingObstacles(scene, NUM_OBS, HALF, [
  { pos: START, minDist: 2.5 },
  { pos: GOAL,  minDist: 2.5 }
]);

// ═══════════════════════════════════════════════════
//  TRAIL
// ═══════════════════════════════════════════════════
var MAX_TRAIL = 600;
var trailPoints = [];
var trailGeo = new THREE.BufferGeometry();
var trailLine = new THREE.Line(
  trailGeo,
  new THREE.LineBasicMaterial({ color: 0x33aaff, transparent: true, opacity: 0.75 })
);
scene.add(trailLine);

// ═══════════════════════════════════════════════════
//  APF PARAMETERS
// ═══════════════════════════════════════════════════
var SIM = {
  K_ATT:  2.5,
  D_STAR: 2.8,
  K_REP:  40.0,
  D0:     3.8,
  K_WALL: 14.0,
  D_WALL: 2.2,
  DT:     0.012,
  DAMP:   0.75,
  VMAX:   0.18,
};

// ═══════════════════════════════════════════════════
//  HPF PARAMETERS
//  Coulomb / Newtonian (1/r²) — fundamental solution
//  to Laplace's equation in 3D.  No hard cutoff:
//  every obstacle repels globally, 1/r² ensures fast
//  falloff so distant ones don't overwhelm attraction.
//  Attractive is constant-magnitude (log-potential
//  gradient) to guarantee goal-ward force everywhere.
// ═══════════════════════════════════════════════════
var HPF = {
  K_ATT:  6.0,    // constant magnitude attractive
  K_REP:  3.0,    // Coulomb 1/d² repulsion (no cutoff)
  K_WALL: 1.5,    // Coulomb 1/d² wall repulsion
  D_MIN:  0.3,    // min surface-dist (prevents singularity)
  DT:     0.012,
  DAMP:   0.75,
  VMAX:   0.18,
};

// ═══════════════════════════════════════════════════
//  FIELD MODE
// ═══════════════════════════════════════════════════
var fieldMode = 'APF';

window.setFieldMode = function(mode) {
  fieldMode = mode;
  var isHPF = mode === 'HPF';
  document.getElementById('btnAPF').className = 'field-btn' + (isHPF ? '' : ' active');
  document.getElementById('btnHPF').className = 'field-btn' + (isHPF ? ' hpf-active' : '');
  document.getElementById('hud-title').textContent = '\u9672 ' + mode + ' Moving Obstacles';
  document.getElementById('field-panel-title').textContent = mode + ' FORCES';
  trailLine.material.color.setHex(isHPF ? 0xcc44ff : 0x33aaff);
  document.getElementById('trail-dot').style.background =
    isHPF ? 'rgba(200,80,255,0.6)' : 'rgba(0,200,255,0.5)';
  document.getElementById('trail-label').textContent = mode + ' Trail';
};

// ═══════════════════════════════════════════════════
//  SIMULATION STATE
// ═══════════════════════════════════════════════════
var vel      = new THREE.Vector3();
var running  = false;
var reached  = false;
var pathLen  = 0;
var prevPos  = START.clone();
var stuckCtr = 0;
var frame    = 0;

var lastFatt = 0, lastFrep = 0, lastFnet = 0;

function computeAPF(pos) {
  var F = new THREE.Vector3();

  var toGoal = GOAL.clone().sub(pos);
  var dGoal = toGoal.length();
  var Fatt = dGoal < SIM.D_STAR
    ? toGoal.clone().multiplyScalar(SIM.K_ATT)
    : toGoal.clone().normalize().multiplyScalar(SIM.K_ATT * SIM.D_STAR);

  lastFatt = Fatt.length();
  F.add(Fatt);

  var Frep = new THREE.Vector3();
  for (var i = 0; i < obstacles.length; i++) {
    var obs = obstacles[i];
    var toObs = pos.clone().sub(obs.mesh.position);
    var raw = toObs.length();
    var d = Math.max(raw - obs.radius, 0.05);
    if (d < SIM.D0) {
      var mag = SIM.K_REP * (1/d - 1/SIM.D0) / (d * d);
      Frep.add(toObs.clone().normalize().multiplyScalar(mag));
    }
  }
  lastFrep = Frep.length();
  F.add(Frep);

  var bound = HALF - 0.25;
  var wallScale = Math.max(0.0, Math.min(1.0, (dGoal - 0.4) / 2.0));
  var axes = ['x', 'y', 'z'];
  for (var a = 0; a < axes.length; a++) {
    var ax = axes[a];
    var p = pos[ax];
    var dPos = bound - p;
    if (dPos < SIM.D_WALL && dPos > 0.001) {
      F[ax] -= SIM.K_WALL * (1/dPos - 1/SIM.D_WALL) / (dPos * dPos) * wallScale;
    }
    var dNeg = p + bound;
    if (dNeg < SIM.D_WALL && dNeg > 0.001) {
      F[ax] += SIM.K_WALL * (1/dNeg - 1/SIM.D_WALL) / (dNeg * dNeg) * wallScale;
    }
  }

  lastFnet = F.length();
  return F;
}

// Harmonic Potential Field — Coulomb (1/r²) formulation.
// Attractive:  constant K (gradient of linear potential K·r) —
//              guarantees goal-ward force at every position.
// Repulsive:   Coulomb 1/d² (gradient of Newtonian 1/r) —
//              no hard cutoff, global but fast-decaying.
function computeHPF(pos) {
  var F = new THREE.Vector3();

  var toGoal = GOAL.clone().sub(pos);
  var dGoal  = toGoal.length();
  // Constant attractive — always K_ATT toward goal regardless of distance
  var Fatt   = toGoal.clone().normalize().multiplyScalar(HPF.K_ATT);
  lastFatt   = Fatt.length();
  F.add(Fatt);

  // Coulomb 1/d² repulsion — no cutoff radius (every obstacle contributes)
  var Frep = new THREE.Vector3();
  for (var i = 0; i < obstacles.length; i++) {
    var obs   = obstacles[i];
    var toObs = pos.clone().sub(obs.mesh.position);
    var d     = Math.max(toObs.length() - obs.radius, HPF.D_MIN);
    Frep.add(toObs.clone().normalize().multiplyScalar(HPF.K_REP / (d * d)));
  }
  lastFrep = Frep.length();
  F.add(Frep);

  // Coulomb 1/d² wall repulsion — global, no D_WALL cutoff
  var bound = HALF - 0.25;
  var axes  = ['x', 'y', 'z'];
  for (var a = 0; a < axes.length; a++) {
    var ax   = axes[a];
    var p    = pos[ax];
    var dPos = Math.max(bound - p, 0.1);   // dist to + wall
    var dNeg = Math.max(p + bound, 0.1);   // dist to – wall
    F[ax] -= HPF.K_WALL / (dPos * dPos);
    F[ax] += HPF.K_WALL / (dNeg * dNeg);
  }

  lastFnet = F.length();
  return F;
}

// ═══════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════
function animate() {
  requestAnimationFrame(animate);
  frame++;

  var speedMul = speedCtrl.get();

  APF.animateGoalRings(goalRings, frame);
  APF.updateMovingObstacles(obstacles, HALF);

  if (running && !reached) {
    var dt = SIM.DT * speedMul;
    var pos = droneGroup.position;

    var force = fieldMode === 'HPF' ? computeHPF(pos) : computeAPF(pos);

    if (vel.length() < 0.004) {
      stuckCtr++;
      if (stuckCtr > 40) {
        force.add(new THREE.Vector3(
          (Math.random()-0.5)*0.8, (Math.random()-0.5)*0.8, (Math.random()-0.5)*0.8
        ));
        stuckCtr = 0;
      }
    } else { stuckCtr = 0; }

    vel.addScaledVector(force, dt);
    vel.multiplyScalar(SIM.DAMP);
    vel.clampLength(0, SIM.VMAX * speedMul);

    pos.add(vel);
    pos.clampScalar(-HALF + 0.18, HALF - 0.18);

    for (var i = 0; i < obstacles.length; i++) {
      var obs = obstacles[i];
      var diff = pos.clone().sub(obs.mesh.position);
      var minDist = obs.radius + 0.28;
      if (diff.length() < minDist) {
        pos.copy(obs.mesh.position).addScaledVector(diff.normalize(), minDist);
        vel.reflect(diff.normalize()).multiplyScalar(0.3);
      }
    }

    pathLen += pos.distanceTo(prevPos);
    prevPos.copy(pos.clone());
    trailPoints.push(pos.clone());
    if (trailPoints.length > MAX_TRAIL) trailPoints.shift();
    if (trailPoints.length >= 2) trailGeo.setFromPoints(trailPoints);

    APF.tiltDrone(droneGroup, vel);

    if (pos.distanceTo(GOAL) < 0.55) {
      reached = true;
      running = false;
      scene.remove(droneGroup);
      scene.remove(trailLine);
      onGoalReached();
    }

    // HUD
    var nearest = Infinity;
    for (var j = 0; j < obstacles.length; j++) {
      var d = pos.distanceTo(obstacles[j].mesh.position) - obstacles[j].radius;
      if (d < nearest) nearest = d;
    }

    document.getElementById('sPos').textContent =
      pos.x.toFixed(1) + ', ' + pos.y.toFixed(1) + ', ' + pos.z.toFixed(1);
    document.getElementById('sVel').textContent = (vel.length() * 100).toFixed(1) + ' u/s';
    document.getElementById('sPath').textContent = pathLen.toFixed(2);
    document.getElementById('sObs').textContent = nearest.toFixed(2) + ' u';
    document.getElementById('sDist').textContent = pos.distanceTo(GOAL).toFixed(2) + ' u';

    var progress = Math.max(0, 1 - pos.distanceTo(GOAL) / DIAG) * 100;
    document.getElementById('progress-bar').style.width = progress + '%';

    var maxF = 8;
    document.getElementById('fAtt').textContent = lastFatt.toFixed(2);
    document.getElementById('bAtt').style.width = Math.min(lastFatt/maxF*100, 100) + '%';
    document.getElementById('fRep').textContent = lastFrep.toFixed(2);
    document.getElementById('bRep').style.width = Math.min(lastFrep/maxF*100, 100) + '%';
    document.getElementById('fNet').textContent = lastFnet.toFixed(2);
    document.getElementById('bNet').style.width = Math.min(lastFnet/maxF*100, 100) + '%';
  }

  var spinSpeed = running ? 0.28 : 0.06;
  APF.spinRotors(rotorMeshes, spinSpeed);

  var sp = 1 + Math.sin(frame * 0.06) * 0.08;
  startMarker.scale.setScalar(sp);

  renderer.render(scene, camera);
}

// ═══════════════════════════════════════════════════
//  GOAL REACHED
// ═══════════════════════════════════════════════════
function onGoalReached() {
  document.getElementById('sStatus').textContent = 'ARRIVED';
  document.getElementById('sStatus').style.color = '#00ff88';
  document.getElementById('status-text').textContent = '\u2b21 MISSION COMPLETE';
  document.getElementById('status-text').className = 'arrived';
  document.getElementById('btnStart').textContent = '\u2713 DONE';
  document.getElementById('btnStart').disabled = true;
  document.getElementById('progress-bar').style.width = '100%';
  document.getElementById('goal-overlay').classList.add('visible');
  setTimeout(function() {
    document.getElementById('goal-overlay').classList.remove('visible');
  }, 5000);
}

// ═══════════════════════════════════════════════════
//  BUTTONS
// ═══════════════════════════════════════════════════
var btnStart = document.getElementById('btnStart');
var btnReset = document.getElementById('btnReset');

btnStart.addEventListener('click', function() {
  if (reached) return;
  running = !running;
  if (running) {
    btnStart.textContent = '\u23f8 PAUSE';
    btnStart.classList.add('active');
    document.getElementById('sStatus').textContent = 'NAVIGATING';
    document.getElementById('sStatus').style.color = '#ffcc00';
    document.getElementById('status-text').textContent = '\u2b21 NAVIGATING...';
    document.getElementById('status-text').className = 'navigating';
  } else {
    btnStart.textContent = '\u25b6 RESUME';
    btnStart.classList.remove('active');
    document.getElementById('sStatus').textContent = 'PAUSED';
    document.getElementById('sStatus').style.color = '#88a';
    document.getElementById('status-text').textContent = '\u2b21 PAUSED';
    document.getElementById('status-text').className = '';
  }
});

btnReset.addEventListener('click', function() {
  running = false;
  reached = false;
  vel.set(0, 0, 0);
  if (!scene.children.includes(droneGroup)) scene.add(droneGroup);
  if (!scene.children.includes(trailLine)) scene.add(trailLine);
  droneGroup.position.copy(START);
  droneGroup.rotation.set(0, 0, 0);
  trailPoints.length = 0;
  trailGeo = new THREE.BufferGeometry();
  trailLine.geometry = trailGeo;
  pathLen = 0;
  prevPos.copy(START);
  stuckCtr = 0;
  lastFatt = lastFrep = lastFnet = 0;

  btnStart.textContent = '\u25b6 LAUNCH';
  btnStart.classList.remove('active');
  btnStart.disabled = false;
  document.getElementById('sStatus').textContent = 'STANDBY';
  document.getElementById('sStatus').style.color = '#00ffe7';
  document.getElementById('status-text').textContent = '\u2b21 AWAITING LAUNCH';
  document.getElementById('status-text').className = '';
  document.getElementById('sPos').textContent = '\u2014';
  document.getElementById('sVel').textContent = '\u2014';
  document.getElementById('sPath').textContent = '0.00';
  document.getElementById('sObs').textContent = '\u2014';
  document.getElementById('sDist').textContent = '\u2014';
  document.getElementById('progress-bar').style.width = '0%';
  document.getElementById('bAtt').style.width = '0%';
  document.getElementById('bRep').style.width = '0%';
  document.getElementById('bNet').style.width = '0%';
  document.getElementById('fAtt').textContent = '0.00';
  document.getElementById('fRep').textContent = '0.00';
  document.getElementById('fNet').textContent = '0.00';
  document.getElementById('goal-overlay').classList.remove('visible');
});

// ═══════════════════════════════════════════════════
//  KICK OFF
// ═══════════════════════════════════════════════════
animate();
