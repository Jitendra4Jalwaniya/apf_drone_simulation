window.APF = window.APF || {};

APF.createMovingObstacles = function(scene, count, half, exclusions) {
  var cube = half * 2;
  var obstacles = [];

  for (var i = 0; i < count; i++) {
    var rad = 0.35 + Math.random() * 0.32;
    var geo = new THREE.SphereGeometry(rad, 20, 20);
    var mat = new THREE.MeshPhongMaterial({
      color: 0xff3333, emissive: 0x330000, shininess: 80, transparent: true, opacity: 0.85
    });
    var mesh = new THREE.Mesh(geo, mat);

    var wireGeo = new THREE.SphereGeometry(rad * 1.02, 10, 10);
    var wireMat = new THREE.MeshBasicMaterial({ color: 0xff6666, wireframe: true, transparent: true, opacity: 0.3 });
    mesh.add(new THREE.Mesh(wireGeo, wireMat));

    var pos;
    var tries = 0;
    do {
      pos = new THREE.Vector3(
        (Math.random() - 0.5) * (cube - 1.8),
        (Math.random() - 0.5) * (cube - 1.8),
        (Math.random() - 0.5) * (cube - 1.8)
      );
      tries++;
      var tooClose = false;
      for (var j = 0; j < exclusions.length; j++) {
        if (pos.distanceTo(exclusions[j].pos) < exclusions[j].minDist) {
          tooClose = true;
          break;
        }
      }
      if (!tooClose) break;
    } while (tries < 60);

    mesh.position.copy(pos);
    scene.add(mesh);

    var speed = 0.012 + Math.random() * 0.02;
    var vel = new THREE.Vector3(
      Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5
    ).normalize().multiplyScalar(speed);

    obstacles.push({ mesh: mesh, vel: vel, radius: rad });
  }

  return obstacles;
};

APF.updateMovingObstacles = function(obstacles, half) {
  for (var i = 0; i < obstacles.length; i++) {
    var obs = obstacles[i];
    obs.mesh.position.add(obs.vel);
    obs.mesh.rotation.x += 0.008;
    obs.mesh.rotation.y += 0.013;
    var axes = ['x', 'y', 'z'];
    for (var a = 0; a < axes.length; a++) {
      var ax = axes[a];
      var b = half - obs.radius;
      if (obs.mesh.position[ax] > b)  { obs.vel[ax] *= -1; obs.mesh.position[ax] = b; }
      if (obs.mesh.position[ax] < -b) { obs.vel[ax] *= -1; obs.mesh.position[ax] = -b; }
    }
  }
};
