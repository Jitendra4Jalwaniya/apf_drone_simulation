'use strict';

// ═══════════════════════════════════════════════════
//  SCENE BOOTSTRAP
// ═══════════════════════════════════════════════════
var engine = APF.initEngine('canvas-container', { fov: 60 });
var renderer = engine.renderer;
var scene = engine.scene;
var camera = engine.camera;

var cam = APF.initCameraControls(renderer, camera, {
  theta: -Math.PI / 4,
  phi: Math.PI / 3.2,
  r: 24,
  sensitivity: 0.006,
  phiMin: 0.15,
  phiMax: Math.PI - 0.15
});

// ═══════════════════════════════════════════════════
//  LIGHTS
// ═══════════════════════════════════════════════════
var ambientLight = new THREE.AmbientLight(0x001122, 1.5);
scene.add(ambientLight);
var pointLight1 = new THREE.PointLight(0x00aaff, 2, 30);
scene.add(pointLight1);

// ═══════════════════════════════════════════════════
//  STARS
// ═══════════════════════════════════════════════════
(function makeStars() {
  var geo = new THREE.BufferGeometry();
  var count = 1800;
  var pos = new Float32Array(count * 3);
  for (var i = 0; i < count * 3; i++) pos[i] = (Math.random() - 0.5) * 140;
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  var mat = new THREE.PointsMaterial({ color: 0xffffff, size: 0.08, transparent: true, opacity: 0.5 });
  scene.add(new THREE.Points(geo, mat));
})();

// ═══════════════════════════════════════════════════
//  CUBE WIREFRAME
// ═══════════════════════════════════════════════════
var CUBE_SIZE = 10;
var HALF = CUBE_SIZE / 2;
var cubeGeo = new THREE.BoxGeometry(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
var cubeEdges = new THREE.EdgesGeometry(cubeGeo);
var cubeMat = new THREE.LineBasicMaterial({
  color: 0x00ffe7, transparent: true, opacity: 0.25, linewidth: 1
});
scene.add(new THREE.LineSegments(cubeEdges, cubeMat));

// Corner dots
var cornerGeo = new THREE.SphereGeometry(0.08, 8, 8);
var cornerMat = new THREE.MeshBasicMaterial({ color: 0x00ffe7, transparent: true, opacity: 0.5 });
var corners = [
  [-1,-1,-1],[1,-1,-1],[-1,1,-1],[1,1,-1],
  [-1,-1,1],[1,-1,1],[-1,1,1],[1,1,1]
];
corners.forEach(function(c) {
  var m = new THREE.Mesh(cornerGeo, cornerMat);
  m.position.set(c[0] * HALF, c[1] * HALF, c[2] * HALF);
  scene.add(m);
});

// Grid floor
var gridHelper = new THREE.GridHelper(CUBE_SIZE, 10, 0x001122, 0x001122);
gridHelper.position.y = -HALF;
gridHelper.material.transparent = true;
gridHelper.material.opacity = 0.4;
scene.add(gridHelper);

// ═══════════════════════════════════════════════════
//  START & GOAL
// ═══════════════════════════════════════════════════
var START_POS = new THREE.Vector3(-4.2, -4.2, -4.2);
var GOAL_POS = new THREE.Vector3(4.2, 4.2, 4.2);

var startGeo = new THREE.OctahedronGeometry(0.18, 0);
var startMat = new THREE.MeshBasicMaterial({ color: 0x4488ff, wireframe: true });
var startMarker = new THREE.Mesh(startGeo, startMat);
startMarker.position.copy(START_POS);
scene.add(startMarker);

var goalGeo = new THREE.SphereGeometry(0.28, 16, 16);
var goalMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.9 });
var goalMesh = new THREE.Mesh(goalGeo, goalMat);
goalMesh.position.copy(GOAL_POS);
scene.add(goalMesh);

var goalRingGeo = new THREE.TorusGeometry(0.55, 0.025, 8, 32);
var goalRingMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.5 });
var goalRing = new THREE.Mesh(goalRingGeo, goalRingMat);
goalRing.position.copy(GOAL_POS);
scene.add(goalRing);

// ═══════════════════════════════════════════════════
//  OBSTACLES
// ═══════════════════════════════════════════════════
var obstacles = [];
var obsGroup = new THREE.Group();
scene.add(obsGroup);

function placeObstacles() {
  while (obsGroup.children.length) obsGroup.remove(obsGroup.children[0]);
  obstacles.length = 0;

  var configs = [
    { t: 0.18, off: [1.2, -0.8, 0.5], r: 0.65 },
    { t: 0.28, off: [-1.0, 0.9, -0.6], r: 0.55 },
    { t: 0.38, off: [0.5, -1.2, 0.9], r: 0.70 },
    { t: 0.45, off: [-0.7, 0.6, 1.1], r: 0.50 },
    { t: 0.52, off: [1.1, -0.5, -0.8], r: 0.60 },
    { t: 0.58, off: [-0.9, 1.0, 0.4], r: 0.55 },
    { t: 0.65, off: [0.6, 0.8, -1.1], r: 0.65 },
    { t: 0.72, off: [-1.3, -0.6, 0.7], r: 0.58 },
    { t: 0.80, off: [0.8, -0.9, 0.6], r: 0.60 },
    { t: 0.88, off: [-0.5, 1.1, -0.8], r: 0.50 },
  ];

  configs.forEach(function(cfg) {
    var center = START_POS.clone().lerp(GOAL_POS, cfg.t);
    var obsPos = center.add(new THREE.Vector3(cfg.off[0], cfg.off[1], cfg.off[2]));
    obsPos.clampScalar(-HALF + cfg.r + 0.1, HALF - cfg.r - 0.1);

    var geo = new THREE.SphereGeometry(cfg.r, 20, 20);
    var mat = new THREE.MeshPhongMaterial({
      color: 0xff3333, emissive: 0x330000, transparent: true, opacity: 0.85, shininess: 80
    });
    var mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(obsPos);
    obsGroup.add(mesh);

    var wireGeo = new THREE.SphereGeometry(cfg.r * 1.02, 10, 10);
    var wireMat = new THREE.MeshBasicMaterial({ color: 0xff6666, wireframe: true, transparent: true, opacity: 0.3 });
    var wireMesh = new THREE.Mesh(wireGeo, wireMat);
    wireMesh.position.copy(obsPos);
    obsGroup.add(wireMesh);

    obstacles.push({ position: obsPos.clone(), radius: cfg.r });
  });

  document.getElementById('val-obs').textContent = obstacles.length;
}
placeObstacles();

// ═══════════════════════════════════════════════════
//  DRONE
// ═══════════════════════════════════════════════════
var droneGroup = new THREE.Group();
scene.add(droneGroup);

var bodyGeo = new THREE.OctahedronGeometry(0.22, 1);
var bodyMat = new THREE.MeshPhongMaterial({
  color: 0x2266ff, emissive: 0x0033aa, shininess: 120, transparent: true, opacity: 0.95
});
var bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
droneGroup.add(bodyMesh);

var armMat = new THREE.MeshBasicMaterial({ color: 0x88aaff, transparent: true, opacity: 0.7 });
var armOffsets = [[0.35, 0, 0.35], [-0.35, 0, -0.35], [0.35, 0, -0.35], [-0.35, 0, 0.35]];
armOffsets.forEach(function(off) {
  var armGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.5, 6);
  var arm = new THREE.Mesh(armGeo, armMat);
  arm.position.set(off[0] * 0.5, off[1], off[2] * 0.5);
  arm.lookAt(0, 0, 0);
  arm.rotateX(Math.PI / 2);
  droneGroup.add(arm);

  var rotorGeo = new THREE.TorusGeometry(0.12, 0.012, 6, 16);
  var rotorMat = new THREE.MeshBasicMaterial({ color: 0x00ddff, transparent: true, opacity: 0.7 });
  var rotor = new THREE.Mesh(rotorGeo, rotorMat);
  rotor.position.set(off[0], off[1], off[2]);
  rotor.rotation.x = Math.PI / 2;
  droneGroup.add(rotor);
});

var spriteMat = new THREE.SpriteMaterial({
  map: (function() {
    var c = document.createElement('canvas'); c.width = c.height = 64;
    var ctx = c.getContext('2d');
    var g = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
    g.addColorStop(0, 'rgba(100,180,255,0.9)');
    g.addColorStop(0.4, 'rgba(50,100,255,0.4)');
    g.addColorStop(1, 'rgba(0,0,255,0)');
    ctx.fillStyle = g; ctx.fillRect(0, 0, 64, 64);
    return new THREE.CanvasTexture(c);
  })(),
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending
});
var glowSprite = new THREE.Sprite(spriteMat);
glowSprite.scale.setScalar(1.4);
droneGroup.add(glowSprite);

droneGroup.position.copy(START_POS);

// ═══════════════════════════════════════════════════
//  TRAIL
// ═══════════════════════════════════════════════════
var MAX_TRAIL = 1200;
var trailPositions = new Float32Array(MAX_TRAIL * 3);
var trailColors = new Float32Array(MAX_TRAIL * 3);
var trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
trailGeo.setAttribute('color', new THREE.BufferAttribute(trailColors, 3));
var trailMat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.7, blending: THREE.AdditiveBlending, depthWrite: false });
var trailLine = new THREE.Line(trailGeo, trailMat);
scene.add(trailLine);

var trailPoints = [];

function updateTrail() {
  var n = trailPoints.length;
  var isHPF = fieldMode === 'HPF';
  for (var i = 0; i < Math.min(n, MAX_TRAIL); i++) {
    var p = trailPoints[i];
    trailPositions[i * 3]     = p.x;
    trailPositions[i * 3 + 1] = p.y;
    trailPositions[i * 3 + 2] = p.z;
    var t = i / Math.max(n - 1, 1);
    if (isHPF) {
      // Purple gradient for HPF
      trailColors[i * 3]     = 0.4 + t * 0.4;
      trailColors[i * 3 + 1] = 0.0 + t * 0.15;
      trailColors[i * 3 + 2] = 0.7 + t * 0.3;
    } else {
      // Cyan gradient for APF
      trailColors[i * 3]     = 0.1 + t * 0.1;
      trailColors[i * 3 + 1] = 0.5 + t * 0.5;
      trailColors[i * 3 + 2] = 0.8 + t * 0.2;
    }
  }
  trailGeo.setDrawRange(0, Math.min(n, MAX_TRAIL));
  trailGeo.attributes.position.needsUpdate = true;
  trailGeo.attributes.color.needsUpdate = true;
}

// ═══════════════════════════════════════════════════
//  FORCE ARROWS
// ═══════════════════════════════════════════════════
var arrowAtt = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1, 0x00ff88, 0.2, 0.12
);
var arrowRep = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1, 0xff4444, 0.2, 0.12
);
var arrowNet = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1, 0xffffff, 0.2, 0.12
);
scene.add(arrowAtt); scene.add(arrowRep); scene.add(arrowNet);

// ═══════════════════════════════════════════════════
//  APF PARAMETERS
// ═══════════════════════════════════════════════════
var K_ATT = 1.4;
var K_REP = 3.5;
var D0 = 2.2;
var WALL_K_REP = 1.2;
var WALL_D0 = 1.2;
var GOAL_THRESH = 0.35;
var BASE_SPEED = 0.038;

// ═══════════════════════════════════════════════════
//  HPF PARAMETERS
//  Coulomb / Newtonian potential — the fundamental
//  solution to Laplace's equation in 3D: φ = 1/r.
//  Forces decay as 1/r² (no cutoff, truly global).
//  Faster falloff than APF's (1/d−1/d₀)/d² ensures
//  distant obstacles don't overwhelm attraction.
// ═══════════════════════════════════════════════════
var HPF_K_ATT  = 5.0;   // attractive  (F = K / d,  1/r gradient)
var HPF_K_REP  = 1.5;   // repulsive   (F = K / d², Coulomb, no cutoff)
var HPF_K_WALL = 0.6;   // wall        (F = K / d², Coulomb)

// ═══════════════════════════════════════════════════
//  FIELD MODE
// ═══════════════════════════════════════════════════
var fieldMode = 'APF';

window.setFieldMode = function(mode) {
  fieldMode = mode;
  var isHPF = mode === 'HPF';
  document.getElementById('btnAPF').className = 'field-btn' + (isHPF ? '' : ' active');
  document.getElementById('btnHPF').className = 'field-btn' + (isHPF ? ' hpf-active' : '');
  document.getElementById('hud-title').textContent = '\u9672 ' + mode + ' Drone Nav';
  // Trail colour
  document.getElementById('trail-dot').style.background =
    isHPF ? 'rgba(200,80,255,0.6)' : 'rgba(0,200,255,0.5)';
  document.getElementById('trail-label').textContent = mode + ' Trail';
  // Arrow colours
  arrowAtt.setColor(isHPF ? 0xcc44ff : 0x00ff88);
  arrowRep.setColor(isHPF ? 0xff44aa : 0xff4444);
  // Clear trail so colour change is immediate
  trailPoints = [];
  updateTrail();
};

var dronePos = START_POS.clone();
var isRunning = true;
var arrived = false;
var stuckTimer = 0;
var stuckPos = new THREE.Vector3();

function computeAPF() {
  var pos = dronePos;
  var toGoal = GOAL_POS.clone().sub(pos);
  var distGoal = toGoal.length();

  var F_att = distGoal < 2.0
    ? toGoal.clone().multiplyScalar(K_ATT)
    : toGoal.clone().normalize().multiplyScalar(K_ATT * 2.0);

  var obsNorm = obstacles.map(function(o) { return { pos: o.position, radius: o.radius }; });
  var F_rep = APF.obstacleForceAPF(pos, obsNorm, { K_REP: K_REP, D0: D0 });
  F_rep.add(APF.wallForceAPF(pos, HALF, { K_WALL: WALL_K_REP, D_WALL: WALL_D0 }));

  return { F_att: F_att, F_rep: F_rep, total: F_att.clone().add(F_rep), distGoal: distGoal };
}

// Harmonic Potential Field — Coulomb (1/r²) formulation.
// Attractive:  F = K / d    (gradient of ln r — log potential)
// Repulsive:   F = K / d²   (gradient of 1/r — Newtonian, true 3D harmonic)
// No hard cutoff: every obstacle contributes globally,
// but 1/d² decays fast enough that distant ones are negligible.
function computeHPF() {
  var pos = dronePos;
  var toGoal = GOAL_POS.clone().sub(pos);
  var distGoal = toGoal.length();

  var attMag = HPF_K_ATT / Math.max(distGoal, 0.3);
  var F_att = toGoal.clone().normalize().multiplyScalar(attMag);

  var obsNorm = obstacles.map(function(o) { return { pos: o.position, radius: o.radius }; });
  var F_rep = APF.obstacleForceHPF(pos, obsNorm, { K_REP: HPF_K_REP, D_MIN: 0.25 });
  F_rep.add(APF.wallForceHPF(pos, HALF, { K_WALL: HPF_K_WALL }));

  return { F_att: F_att, F_rep: F_rep, total: F_att.clone().add(F_rep), distGoal: distGoal };
}

// ═══════════════════════════════════════════════════
//  SIMULATION STATE
// ═══════════════════════════════════════════════════
var simTime = 0;
var speedMult = 1.5;
document.getElementById('speed-input').addEventListener('input', function() {
  speedMult = parseFloat(this.value);
  document.getElementById('speed-val').textContent = speedMult.toFixed(1) + '\u00d7';
});

function togglePlay() {
  if (arrived) return;
  isRunning = !isRunning;
  var btn = document.getElementById('btn-play');
  btn.textContent = isRunning ? '\u23f8 PAUSE' : '\u25b6 PLAY';
  btn.classList.toggle('active', isRunning);
}
window.togglePlay = togglePlay;

function resetSim() {
  dronePos.copy(START_POS);
  droneGroup.position.copy(START_POS);
  trailPoints = [];
  updateTrail();
  arrived = false;
  isRunning = true;
  stuckTimer = 0;
  document.getElementById('btn-play').textContent = '\u23f8 PAUSE';
  document.getElementById('btn-play').classList.add('active');
  document.getElementById('status-text').textContent = '\u2b21 COMPUTING PATH...';
  document.getElementById('status-text').className = 'planning';
  document.getElementById('progress-bar').style.width = '0%';
  arrowAtt.visible = true; arrowRep.visible = true; arrowNet.visible = true;
}
window.resetSim = resetSim;

// ═══════════════════════════════════════════════════
//  ANIMATION LOOP
// ═══════════════════════════════════════════════════
var clock = new THREE.Clock();

function animate() {
  requestAnimationFrame(animate);
  var dt = clock.getDelta();
  simTime += dt;

  var speed = BASE_SPEED * speedMult;

  if (isRunning && !arrived) {
    var result = fieldMode === 'HPF' ? computeHPF() : computeAPF();
    var F_att = result.F_att;
    var F_rep = result.F_rep;
    var total = result.total;
    var distGoal = result.distGoal;
    var totalLen = total.length();

    if (dronePos.distanceTo(stuckPos) < 0.01) {
      stuckTimer++;
      if (stuckTimer > 60) {
        total.add(new THREE.Vector3(
          (Math.random() - 0.5) * 2.5,
          (Math.random() - 0.5) * 2.5,
          (Math.random() - 0.5) * 2.5
        ));
        stuckTimer = 0;
      }
    } else {
      stuckTimer = 0;
      stuckPos.copy(dronePos);
    }

    var step = total.clone().normalize().multiplyScalar(speed);
    dronePos.add(step);
    dronePos.clampScalar(-HALF + 0.25, HALF - 0.25);
    droneGroup.position.copy(dronePos);
    pointLight1.position.copy(dronePos);

    trailPoints.push(dronePos.clone());
    if (trailPoints.length > MAX_TRAIL) trailPoints.shift();
    updateTrail();

    bodyMesh.rotation.y += dt * 2.5;
    bodyMesh.rotation.z += dt * 1.2;

    var arrScale = 0.6;
    var attLen = Math.min(F_att.length() * arrScale, 3);
    var repLen = Math.min(F_rep.length() * arrScale, 3);
    var netLen = Math.min(total.length() * arrScale * 0.5, 3);
    arrowAtt.setLength(attLen, 0.25, 0.12); arrowAtt.setDirection(F_att.clone().normalize()); arrowAtt.position.copy(dronePos);
    arrowRep.setLength(Math.max(repLen, 0.01), 0.2, 0.1); if (F_rep.length() > 0.01) { arrowRep.setDirection(F_rep.clone().normalize()); } arrowRep.position.copy(dronePos);
    arrowNet.setLength(netLen, 0.22, 0.11); arrowNet.setDirection(total.clone().normalize()); arrowNet.position.copy(dronePos);

    document.getElementById('val-x').textContent = dronePos.x.toFixed(2);
    document.getElementById('val-y').textContent = dronePos.y.toFixed(2);
    document.getElementById('val-z').textContent = dronePos.z.toFixed(2);
    document.getElementById('val-dist').textContent = distGoal.toFixed(2);
    document.getElementById('val-force').textContent = totalLen.toFixed(2);
    var maxDist = START_POS.distanceTo(GOAL_POS);
    var pct = Math.min(100, ((maxDist - distGoal) / maxDist) * 100);
    document.getElementById('progress-bar').style.width = pct.toFixed(1) + '%';

    if (distGoal < GOAL_THRESH) {
      arrived = true;
      dronePos.copy(GOAL_POS);
      droneGroup.position.copy(GOAL_POS);
      arrowAtt.visible = false; arrowRep.visible = false; arrowNet.visible = false;
      document.getElementById('status-text').textContent = '\u2713 DESTINATION REACHED';
      document.getElementById('status-text').className = 'arrived';
      document.getElementById('progress-bar').style.width = '100%';
      document.getElementById('btn-play').textContent = '\u2713 ARRIVED';
      document.getElementById('btn-play').classList.remove('active');
    }
  }

  goalRing.rotation.x = simTime * 1.1;
  goalRing.rotation.y = simTime * 0.8;
  goalMesh.material.opacity = 0.6 + Math.sin(simTime * 4) * 0.3;
  goalMesh.scale.setScalar(1 + Math.sin(simTime * 4) * 0.12);
  glowSprite.material.opacity = 0.5 + Math.sin(simTime * 8) * 0.2;

  obsGroup.rotation.y = simTime * 0.05;

  renderer.render(scene, camera);
}

animate();
