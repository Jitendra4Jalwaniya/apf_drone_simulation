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
//  DRONE CONFIGURATIONS
//  Fixed arrival order: ALPHA -> BETA -> GAMMA -> DELTA
//  Enforced by decreasing K_ATT and VMAX
// ═══════════════════════════════════════════════════
var GOAL = new THREE.Vector3(HALF - 0.4, HALF - 0.4, HALF - 0.4);

var DRONE_CONFIGS = [
  {
    label: 'ALPHA',
    start:      new THREE.Vector3(-HALF + 0.4, -HALF + 0.4, -HALF + 0.4),
    hullColor:  0xdde8f5,
    domeColor:  0x44ddff,
    ledColor:   0x00ffff,
    lightColor: 0x44ccff,
    trailColor: 0x33aaff,
    dotColor:   '#44ddff',
    kAtt:       3.2,
    vmax:       0.22,
  },
  {
    label: 'BETA',
    start:      new THREE.Vector3( HALF - 0.4, -HALF + 0.4, -HALF + 0.4),
    hullColor:  0xf5e0d0,
    domeColor:  0xff8844,
    ledColor:   0xff4400,
    lightColor: 0xff6622,
    trailColor: 0xff6622,
    dotColor:   '#ff8844',
    kAtt:       2.6,
    vmax:       0.17,
  },
  {
    label: 'GAMMA',
    start:      new THREE.Vector3(-HALF + 0.4,  HALF - 0.4, -HALF + 0.4),
    hullColor:  0xd0f5d8,
    domeColor:  0x66ff66,
    ledColor:   0x00ff44,
    lightColor: 0x44ff88,
    trailColor: 0x44ff88,
    dotColor:   '#66ff66',
    kAtt:       2.0,
    vmax:       0.14,
  },
  {
    label: 'DELTA',
    start:      new THREE.Vector3(-HALF + 0.4, -HALF + 0.4,  HALF - 0.4),
    hullColor:  0xe8d0f5,
    domeColor:  0xcc66ff,
    ledColor:   0xaa00ff,
    lightColor: 0xaa44ff,
    trailColor: 0xaa44ff,
    dotColor:   '#cc66ff',
    kAtt:       1.5,
    vmax:       0.11,
  },
];

// ═══════════════════════════════════════════════════
//  GOAL MARKER
// ═══════════════════════════════════════════════════
var goalMarker = APF.makeMarker(scene, GOAL, 0xff4444, 0x992222, 0.28);
var goalRings  = APF.createGoalRings(scene, GOAL);

// Start markers + diagonal guide lines
var startMarkers = [];
DRONE_CONFIGS.forEach(function(cfg) {
  var m = APF.makeMarker(scene, cfg.start, cfg.domeColor, 0x001a10, 0.18);
  startMarkers.push(m);
  APF.createDashedLine(scene, cfg.start, GOAL, cfg.trailColor, 0.22);
});

// ═══════════════════════════════════════════════════
//  OBSTACLES
// ═══════════════════════════════════════════════════
var NUM_OBS = 15;
var exclusions = DRONE_CONFIGS.map(function(c) { return { pos: c.start, minDist: 2.0 }; });
exclusions.push({ pos: GOAL, minDist: 2.0 });
var obstacles = APF.createMovingObstacles(scene, NUM_OBS, HALF, exclusions);

// ═══════════════════════════════════════════════════
//  APF PARAMETERS
// ═══════════════════════════════════════════════════
var SIM = {
  D_STAR:      2.8,
  K_REP:       40.0,
  D0:          3.8,
  K_WALL:      14.0,
  D_WALL:      2.2,
  K_DRONE_REP: 28.0,
  D_DRONE:     2.2,
  DRONE_R:     0.32,
  DT:          0.012,
  DAMP:        0.75,
};

// ═══════════════════════════════════════════════════
//  HPF PARAMETERS
//  Coulomb (1/r²) — fundamental 3D harmonic solution.
//  Constant attractive (scales with per-drone kAtt to
//  preserve arrival order). 1/r² repulsion: globally
//  aware but fast-decaying, no hard cutoff radius.
// ═══════════════════════════════════════════════════
var HPF = {
  ATT_MULT:    2.0,    // constant attract = kAtt × ATT_MULT
  K_REP:       3.0,    // Coulomb 1/d² obstacle repulsion
  K_WALL:      1.5,    // Coulomb 1/d² wall repulsion
  K_DRONE_REP: 2.0,    // Coulomb 1/d² drone–drone repulsion
  DRONE_R:     0.32,
  D_MIN:       0.3,    // min surface dist (singularity guard)
  DT:          0.012,
  DAMP:        0.75,
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
  document.getElementById('hud-title').textContent = '\u9672 ' + mode + ' Drone Swarm';
  document.getElementById('field-panel-title').textContent = mode + ' FORCES \u00b7 ALPHA';
  // Update each drone trail colour
  var apfColors = [0x33aaff, 0xff6622, 0x44ff88, 0xaa44ff];
  var hpfColors = [0xcc44ff, 0xff44aa, 0xbb88ff, 0xff88cc];
  drones.forEach(function(drone, i) {
    drone.trail.line.material.color.setHex(isHPF ? hpfColors[i] : apfColors[i]);
  });
};

// ═══════════════════════════════════════════════════
//  SIMULATION STATE (one entry per drone)
// ═══════════════════════════════════════════════════
var MAX_TRAIL = 500;

var drones = DRONE_CONFIGS.map(function(cfg) {
  var droneObj = APF.buildDrone(cfg);
  droneObj.group.position.copy(cfg.start);
  scene.add(droneObj.group);

  var trailGeo = new THREE.BufferGeometry();
  var trailLine = new THREE.Line(
    trailGeo,
    new THREE.LineBasicMaterial({ color: cfg.trailColor, transparent: true, opacity: 0.75 })
  );
  scene.add(trailLine);

  return {
    cfg:         cfg,
    group:       droneObj.group,
    rotors:      droneObj.rotors,
    vel:         new THREE.Vector3(),
    prevPos:     cfg.start.clone(),
    pathLen:     0,
    stuckCtr:    0,
    reached:     false,
    arrivalRank: null,
    trail:       { points: [], geo: trailGeo, line: trailLine },
    lastFatt:    0,
    lastFrep:    0,
    lastFnet:    0,
  };
});

var running        = false;
var arrivalCounter = 0;
var allReached     = false;
var frame          = 0;

// ═══════════════════════════════════════════════════
//  APF COMPUTATION (per drone, includes drone-drone)
// ═══════════════════════════════════════════════════
function computeAPF(idx) {
  var drone = drones[idx];
  var pos   = drone.group.position;
  var F     = new THREE.Vector3();

  var toGoal = GOAL.clone().sub(pos);
  var dGoal  = toGoal.length();
  var Fatt   = dGoal < SIM.D_STAR
    ? toGoal.clone().multiplyScalar(drone.cfg.kAtt)
    : toGoal.clone().normalize().multiplyScalar(drone.cfg.kAtt * SIM.D_STAR);
  drone.lastFatt = Fatt.length();
  F.add(Fatt);

  // Goal capture zone
  if (dGoal < 1.5) {
    drone.lastFrep = 0;
    drone.lastFnet = F.length();
    return F;
  }

  var Frep = new THREE.Vector3();
  for (var i = 0; i < obstacles.length; i++) {
    var obs = obstacles[i];
    var toObs = pos.clone().sub(obs.mesh.position);
    var raw   = toObs.length();
    var d     = Math.max(raw - obs.radius, 0.05);
    if (d < SIM.D0) {
      var mag = SIM.K_REP * (1/d - 1/SIM.D0) / (d * d);
      Frep.add(toObs.clone().normalize().multiplyScalar(mag));
    }
  }

  for (var j = 0; j < drones.length; j++) {
    if (j === idx || drones[j].reached) continue;
    var toDrone = pos.clone().sub(drones[j].group.position);
    var rawD    = toDrone.length();
    var dd      = Math.max(rawD - SIM.DRONE_R * 2, 0.05);
    if (dd < SIM.D_DRONE) {
      var magD = SIM.K_DRONE_REP * (1/dd - 1/SIM.D_DRONE) / (dd * dd);
      Frep.add(toDrone.clone().normalize().multiplyScalar(magD));
    }
  }

  drone.lastFrep = Frep.length();
  F.add(Frep);

  var bound = HALF - 0.25;
  var axes = ['x', 'y', 'z'];
  for (var a = 0; a < axes.length; a++) {
    var ax = axes[a];
    var p    = pos[ax];
    var dPos = bound - p;
    if (dPos < SIM.D_WALL && dPos > 0.001)
      F[ax] -= SIM.K_WALL * (1/dPos - 1/SIM.D_WALL) / (dPos * dPos);
    var dNeg = p + bound;
    if (dNeg < SIM.D_WALL && dNeg > 0.001)
      F[ax] += SIM.K_WALL * (1/dNeg - 1/SIM.D_WALL) / (dNeg * dNeg);
  }

  drone.lastFnet = F.length();
  return F;
}

// Harmonic Potential Field per drone — Coulomb (1/r²).
// Constant attractive (kAtt × ATT_MULT) preserves arrival order.
// All repulsion uses 1/d²: globally aware, fast falloff.
function computeHPF(idx) {
  var drone = drones[idx];
  var pos   = drone.group.position;
  var F     = new THREE.Vector3();

  var toGoal = GOAL.clone().sub(pos);
  var dGoal  = toGoal.length();
  // Constant attractive scaled by per-drone kAtt (preserves ordering)
  var Fatt   = toGoal.clone().normalize().multiplyScalar(drone.cfg.kAtt * HPF.ATT_MULT);
  drone.lastFatt = Fatt.length();
  F.add(Fatt);

  // Goal capture zone — pure attraction only
  if (dGoal < 1.5) {
    drone.lastFrep = 0;
    drone.lastFnet = F.length();
    return F;
  }

  var Frep = new THREE.Vector3();

  // Coulomb 1/d² obstacle repulsion — no hard cutoff
  for (var i = 0; i < obstacles.length; i++) {
    var obs   = obstacles[i];
    var toObs = pos.clone().sub(obs.mesh.position);
    var d     = Math.max(toObs.length() - obs.radius, HPF.D_MIN);
    Frep.add(toObs.clone().normalize().multiplyScalar(HPF.K_REP / (d * d)));
  }

  // Coulomb 1/d² drone–drone repulsion
  for (var j = 0; j < drones.length; j++) {
    if (j === idx || drones[j].reached) continue;
    var toDrone = pos.clone().sub(drones[j].group.position);
    var dd      = Math.max(toDrone.length() - HPF.DRONE_R * 2, HPF.D_MIN);
    Frep.add(toDrone.clone().normalize().multiplyScalar(HPF.K_DRONE_REP / (dd * dd)));
  }

  drone.lastFrep = Frep.length();
  F.add(Frep);

  // Coulomb 1/d² wall repulsion — global, no D_WALL cutoff
  var bound = HALF - 0.25;
  var axes  = ['x', 'y', 'z'];
  for (var a = 0; a < axes.length; a++) {
    var ax   = axes[a];
    var p    = pos[ax];
    var dPos = Math.max(bound - p, 0.1);
    var dNeg = Math.max(p + bound, 0.1);
    F[ax] -= HPF.K_WALL / (dPos * dPos);
    F[ax] += HPF.K_WALL / (dNeg * dNeg);
  }

  drone.lastFnet = F.length();
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

  if (running && !allReached) {
    var dt = SIM.DT * speedMul;

    for (var idx = 0; idx < drones.length; idx++) {
      var drone = drones[idx];
      if (drone.reached) continue;

      var pos   = drone.group.position;
      var force = fieldMode === 'HPF' ? computeHPF(idx) : computeAPF(idx);

      if (drone.vel.length() < 0.004) {
        drone.stuckCtr++;
        if (drone.stuckCtr > 40) {
          force.add(new THREE.Vector3(
            (Math.random()-0.5)*0.9, (Math.random()-0.5)*0.9, (Math.random()-0.5)*0.9
          ));
          drone.stuckCtr = 0;
        }
      } else {
        drone.stuckCtr = 0;
      }

      drone.vel.addScaledVector(force, dt);
      drone.vel.multiplyScalar(SIM.DAMP);
      drone.vel.clampLength(0, drone.cfg.vmax * speedMul);

      pos.add(drone.vel);
      pos.clampScalar(-HALF + 0.18, HALF - 0.18);

      for (var i = 0; i < obstacles.length; i++) {
        var obs = obstacles[i];
        var diff = pos.clone().sub(obs.mesh.position);
        var minDist = obs.radius + 0.28;
        if (diff.length() < minDist) {
          pos.copy(obs.mesh.position).addScaledVector(diff.normalize(), minDist);
          drone.vel.reflect(diff.normalize()).multiplyScalar(0.3);
        }
      }

      for (var j = 0; j < drones.length; j++) {
        if (j === idx) continue;
        var diffD = pos.clone().sub(drones[j].group.position);
        var minDistD = SIM.DRONE_R * 2;
        if (diffD.length() > 0.001 && diffD.length() < minDistD) {
          pos.copy(drones[j].group.position).addScaledVector(diffD.normalize(), minDistD);
          drone.vel.reflect(diffD.normalize()).multiplyScalar(0.3);
        }
      }

      drone.pathLen += pos.distanceTo(drone.prevPos);
      drone.prevPos.copy(pos.clone());
      drone.trail.points.push(pos.clone());
      if (drone.trail.points.length > MAX_TRAIL) drone.trail.points.shift();
      if (drone.trail.points.length >= 2) drone.trail.geo.setFromPoints(drone.trail.points);

      APF.tiltDrone(drone.group, drone.vel);

      if (pos.distanceTo(GOAL) < 0.8) {
        drone.reached     = true;
        drone.vel.set(0, 0, 0);
        scene.remove(drone.group);
        scene.remove(drone.trail.line);
        arrivalCounter++;
        drone.arrivalRank = arrivalCounter;
        onDroneArrived(idx);
        if (arrivalCounter === drones.length) {
          allReached = true;
          running    = false;
          onAllReached();
        }
      }
    }

    // HUD update (ALPHA drone forces)
    var a    = drones[0];
    var maxF = 10;
    document.getElementById('fAtt').textContent = a.lastFatt.toFixed(2);
    document.getElementById('bAtt').style.width = Math.min(a.lastFatt / maxF * 100, 100) + '%';
    document.getElementById('fRep').textContent = a.lastFrep.toFixed(2);
    document.getElementById('bRep').style.width = Math.min(a.lastFrep / maxF * 100, 100) + '%';
    document.getElementById('fNet').textContent = a.lastFnet.toFixed(2);
    document.getElementById('bNet').style.width = Math.min(a.lastFnet / maxF * 100, 100) + '%';
  }

  var spinSpeed = running ? 0.28 : 0.06;
  for (var d = 0; d < drones.length; d++) {
    APF.spinRotors(drones[d].rotors, spinSpeed);
  }

  var sp = 1 + Math.sin(frame * 0.06) * 0.08;
  startMarkers.forEach(function(m) { m.scale.setScalar(sp); });

  renderer.render(scene, camera);
}

// ═══════════════════════════════════════════════════
//  ARRIVAL HANDLERS
// ═══════════════════════════════════════════════════
var RANK_LABELS = ['#1 ARRIVED', '#2 ARRIVED', '#3 ARRIVED', '#4 ARRIVED'];
var RANK_COLORS = ['#ffd700', '#c0c0c0', '#cd7f32', '#aa88ff'];

function onDroneArrived(idx) {
  var drone = drones[idx];
  var rank  = drone.arrivalRank - 1;
  var el    = document.getElementById('sDrone' + idx);
  el.textContent = RANK_LABELS[rank];
  el.style.color = RANK_COLORS[rank];
  document.getElementById('sArrived').textContent = arrivalCounter + ' / 4';
}

function onAllReached() {
  document.getElementById('sStatus').textContent = 'ARRIVED';
  document.getElementById('sStatus').style.color = '#00ff88';
  document.getElementById('status-text').textContent = '\u2b21 MISSION COMPLETE';
  document.getElementById('status-text').className = 'arrived';
  document.getElementById('btnStart').textContent = '\u2713 DONE';
  document.getElementById('btnStart').disabled = true;
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
  if (allReached) return;
  running = !running;
  if (running) {
    btnStart.textContent = '\u23f8 PAUSE';
    btnStart.classList.add('active');
    document.getElementById('sStatus').textContent = 'NAVIGATING';
    document.getElementById('sStatus').style.color = '#ffcc00';
    document.getElementById('status-text').textContent = '\u2b21 NAVIGATING...';
    document.getElementById('status-text').className = 'navigating';
    drones.forEach(function(d, i) {
      if (!d.reached) {
        var el = document.getElementById('sDrone' + i);
        el.textContent = 'FLYING';
        el.style.color = '#ffcc00';
      }
    });
  } else {
    btnStart.textContent = '\u25b6 RESUME';
    btnStart.classList.remove('active');
    document.getElementById('sStatus').textContent = 'PAUSED';
    document.getElementById('sStatus').style.color = '#88a';
    document.getElementById('status-text').textContent = '\u2b21 PAUSED';
    document.getElementById('status-text').className = '';
    drones.forEach(function(d, i) {
      if (!d.reached) {
        var el = document.getElementById('sDrone' + i);
        el.textContent = 'PAUSED';
        el.style.color = '#88a';
      }
    });
  }
});

btnReset.addEventListener('click', function() {
  running        = false;
  allReached     = false;
  arrivalCounter = 0;

  drones.forEach(function(drone, i) {
    drone.reached     = false;
    drone.arrivalRank = null;
    drone.vel.set(0, 0, 0);
    drone.group.position.copy(drone.cfg.start);
    drone.group.rotation.set(0, 0, 0);
    scene.add(drone.group);
    scene.add(drone.trail.line);
    drone.pathLen   = 0;
    drone.stuckCtr  = 0;
    drone.prevPos.copy(drone.cfg.start);
    drone.trail.points.length = 0;
    drone.trail.geo = new THREE.BufferGeometry();
    drone.trail.line.geometry = drone.trail.geo;
    drone.lastFatt  = drone.lastFrep = drone.lastFnet = 0;

    var el = document.getElementById('sDrone' + i);
    el.textContent = 'STANDBY';
    el.style.color = '#00ffe7';
  });

  btnStart.textContent = '\u25b6 LAUNCH';
  btnStart.classList.remove('active');
  btnStart.disabled = false;
  document.getElementById('sStatus').textContent = 'STANDBY';
  document.getElementById('sStatus').style.color = '#00ffe7';
  document.getElementById('status-text').textContent = '\u2b21 AWAITING LAUNCH';
  document.getElementById('status-text').className = '';
  document.getElementById('sArrived').textContent = '0 / 4';
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
