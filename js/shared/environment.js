window.APF = window.APF || {};

APF.setupLighting = function(scene) {
  scene.add(new THREE.AmbientLight(0x1a2a50, 1.2));

  var sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(8, 14, 6);
  sun.castShadow = true;
  scene.add(sun);

  var bluePoint = new THREE.PointLight(0x3366ff, 2, 18);
  bluePoint.position.set(-6, 5, -6);
  scene.add(bluePoint);

  var redPoint = new THREE.PointLight(0xff4433, 1, 12);
  redPoint.position.set(6, -5, 6);
  scene.add(redPoint);
};

APF.setupCube = function(scene, half) {
  var cube = half * 2;

  var edgeGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(cube, cube, cube));
  var edgeMat = new THREE.LineBasicMaterial({ color: 0x1a3a6a, transparent: true, opacity: 0.9 });
  scene.add(new THREE.LineSegments(edgeGeo, edgeMat));

  function makeGridFace(pos, rotX, rotY, color, opacity) {
    var g = new THREE.GridHelper(cube, 8, color, color);
    g.position.copy(pos);
    if (rotX) g.rotation.x = rotX;
    if (rotY) g.rotation.y = rotY;
    g.material.transparent = true;
    g.material.opacity = opacity;
    scene.add(g);
  }
  makeGridFace(new THREE.Vector3(0, -half, 0), 0, 0, 0x112244, 0.4);
  makeGridFace(new THREE.Vector3(0,  half, 0), 0, 0, 0x112244, 0.2);
  makeGridFace(new THREE.Vector3(-half, 0, 0), Math.PI / 2, Math.PI / 2, 0x0d1e38, 0.15);
  makeGridFace(new THREE.Vector3( half, 0, 0), Math.PI / 2, Math.PI / 2, 0x0d1e38, 0.15);
  makeGridFace(new THREE.Vector3(0, 0, -half), Math.PI / 2, 0, 0x0d1e38, 0.15);
  makeGridFace(new THREE.Vector3(0, 0,  half), Math.PI / 2, 0, 0x0d1e38, 0.15);

  var cornerMat = new THREE.MeshBasicMaterial({ color: 0x1a3870 });
  var cornerGeo = new THREE.SphereGeometry(0.07, 8, 8);
  for (var x of [-half, half])
    for (var y of [-half, half])
      for (var z of [-half, half]) {
        var c = new THREE.Mesh(cornerGeo, cornerMat);
        c.position.set(x, y, z);
        scene.add(c);
      }
};
