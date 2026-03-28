window.APF = window.APF || {};

APF.makeMarker = function(scene, pos, color, emissive, size) {
  var m = new THREE.Mesh(
    new THREE.SphereGeometry(size, 20, 20),
    new THREE.MeshPhongMaterial({ color: color, emissive: emissive, shininess: 80 })
  );
  m.position.copy(pos);
  scene.add(m);
  return m;
};

APF.createGoalRings = function(scene, pos) {
  return [0.5, 0.75, 1.0].map(function(r) {
    var ring = new THREE.Mesh(
      new THREE.TorusGeometry(r, 0.03, 8, 32),
      new THREE.MeshBasicMaterial({ color: 0xff5533, transparent: true, opacity: 0.5 })
    );
    ring.position.copy(pos);
    scene.add(ring);
    return ring;
  });
};

APF.animateGoalRings = function(rings, frame) {
  for (var i = 0; i < rings.length; i++) {
    var t = frame * 0.012 + i * Math.PI * 0.6;
    rings[i].scale.setScalar(1 + Math.sin(t) * 0.12);
    rings[i].material.opacity = 0.25 + Math.sin(t) * 0.2;
    rings[i].rotation.x += 0.012 * (i + 1);
    rings[i].rotation.y += 0.008 * (i + 1);
  }
};

APF.createDashedLine = function(scene, start, end, color, opacity) {
  var lineGeo = new THREE.BufferGeometry().setFromPoints([start, end]);
  var lineMat = new THREE.LineDashedMaterial({
    color: color, dashSize: 0.25, gapSize: 0.35,
    transparent: true, opacity: opacity || 0.4
  });
  var line = new THREE.Line(lineGeo, lineMat);
  line.computeLineDistances();
  scene.add(line);
  return line;
};
