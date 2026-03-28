window.APF = window.APF || {};

APF.buildDrone = function(cfg) {
  var group = new THREE.Group();
  var rotors = [];

  var hull = new THREE.Mesh(
    new THREE.BoxGeometry(0.44, 0.10, 0.44),
    new THREE.MeshPhongMaterial({ color: cfg.hullColor, emissive: 0x0a1020, shininess: 120 })
  );
  group.add(hull);

  var dome = new THREE.Mesh(
    new THREE.SphereGeometry(0.09, 12, 8, 0, Math.PI * 2, 0, Math.PI / 2),
    new THREE.MeshPhongMaterial({ color: cfg.domeColor, emissive: 0x0a2266, shininess: 200 })
  );
  dome.position.y = 0.08;
  group.add(dome);

  var led = new THREE.Mesh(
    new THREE.SphereGeometry(0.05, 8, 8),
    new THREE.MeshBasicMaterial({ color: cfg.ledColor })
  );
  led.position.y = -0.08;
  group.add(led);

  var dl = new THREE.PointLight(cfg.lightColor, 1.5, 2.5);
  group.add(dl);

  var ARM_LEN = 0.55;
  var armMat = new THREE.MeshPhongMaterial({ color: 0x8899aa, emissive: 0x050a10 });
  var hubMat = new THREE.MeshPhongMaterial({ color: 0x667788 });
  var diskMat = new THREE.MeshPhongMaterial({ color: 0x1a2030, transparent: true, opacity: 0.75, side: THREE.DoubleSide });

  [45, 135, 225, 315].forEach(function(deg, i) {
    var rad = deg * Math.PI / 180;
    var dx = Math.cos(rad) * ARM_LEN;
    var dz = Math.sin(rad) * ARM_LEN;
    var dir = new THREE.Vector3(dx, 0, dz).normalize();

    var armCyl = new THREE.Mesh(
      new THREE.CylinderGeometry(0.022, 0.022, ARM_LEN * 1.95, 6), armMat
    );
    var quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    armCyl.quaternion.copy(quat);
    armCyl.position.set(dx * 0.5, 0, dz * 0.5);
    group.add(armCyl);

    var hub = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.07, 8), hubMat);
    hub.position.set(dx, 0.06, dz);
    group.add(hub);

    var disk = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.24, 0.012, 16), diskMat);
    disk.position.set(dx, 0.09, dz);
    group.add(disk);
    rotors.push({ mesh: disk, dir: i % 2 === 0 ? 1 : -1 });

    [0, Math.PI / 2].forEach(function(bRot) {
      var blade = new THREE.Mesh(
        new THREE.BoxGeometry(0.44, 0.008, 0.055),
        new THREE.MeshPhongMaterial({ color: 0x222830, transparent: true, opacity: 0.65 })
      );
      blade.position.set(dx, 0.095, dz);
      blade.rotation.y = bRot;
      group.add(blade);
      rotors.push({ mesh: blade, dir: i % 2 === 0 ? 1 : -1 });
    });
  });

  return { group: group, rotors: rotors };
};

APF.spinRotors = function(rotors, speed) {
  for (var i = 0; i < rotors.length; i++) {
    rotors[i].mesh.rotation.y += speed * rotors[i].dir;
  }
};

APF.tiltDrone = function(group, vel) {
  var tX = -vel.z * 4.5;
  var tZ = vel.x * 4.5;
  group.rotation.x += (tX - group.rotation.x) * 0.12;
  group.rotation.z += (tZ - group.rotation.z) * 0.12;
  if (vel.lengthSq() > 0.0001) {
    var yaw = Math.atan2(vel.x, vel.z);
    group.rotation.y += (yaw - group.rotation.y) * 0.08;
  }
};
