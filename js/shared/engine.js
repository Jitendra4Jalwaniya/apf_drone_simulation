window.APF = window.APF || {};

APF.initEngine = function(containerId, config) {
  config = config || {};
  var fov = config.fov || 58;
  var container = document.getElementById(containerId);
  var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  var scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000408);
  scene.fog = new THREE.FogExp2(0x000408, 0.018);

  var camera = new THREE.PerspectiveCamera(fov, window.innerWidth / window.innerHeight, 0.1, 200);

  window.addEventListener('resize', function() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  return { renderer, scene, camera };
};

APF.initCameraControls = function(renderer, camera, config) {
  config = config || {};
  var sens = config.sensitivity || 0.007;
  var phiMin = config.phiMin || 0.06;
  var phiMax = config.phiMax || Math.PI - 0.06;

  var camState = {
    theta: config.theta != null ? config.theta : Math.PI * 0.42,
    phi:   config.phi   != null ? config.phi   : Math.PI * 0.32,
    r:     config.r     != null ? config.r     : 26
  };

  function updateCamera() {
    camera.position.set(
      camState.r * Math.sin(camState.phi) * Math.sin(camState.theta),
      camState.r * Math.cos(camState.phi),
      camState.r * Math.sin(camState.phi) * Math.cos(camState.theta)
    );
    camera.lookAt(0, 0, 0);
  }
  updateCamera();

  var dragging = false, lastM = {x:0, y:0};
  var el = renderer.domElement;

  el.addEventListener('mousedown', function(e) { dragging = true; lastM = {x:e.clientX,y:e.clientY}; });
  window.addEventListener('mouseup', function() { dragging = false; });
  el.addEventListener('mouseleave', function() { dragging = false; });
  window.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    camState.theta -= (e.clientX - lastM.x) * sens;
    camState.phi = Math.max(phiMin, Math.min(phiMax, camState.phi - (e.clientY - lastM.y) * sens));
    lastM = {x:e.clientX, y:e.clientY};
    updateCamera();
  });
  el.addEventListener('wheel', function(e) {
    camState.r = Math.max(7, Math.min(55, camState.r + e.deltaY * 0.025));
    updateCamera();
    e.preventDefault();
  }, { passive: false });

  var lastTouch = null;
  el.addEventListener('touchstart', function(e) { lastTouch = e.touches[0]; });
  el.addEventListener('touchmove', function(e) {
    if (!lastTouch) return;
    var t = e.touches[0];
    camState.theta -= (t.clientX - lastTouch.clientX) * sens;
    camState.phi = Math.max(phiMin, Math.min(phiMax, camState.phi - (t.clientY - lastTouch.clientY) * sens));
    lastTouch = t;
    updateCamera();
    e.preventDefault();
  }, { passive: false });
  el.addEventListener('touchend', function() { lastTouch = null; });

  return { camState: camState, updateCamera: updateCamera };
};

APF.initSpeedControl = function(inputId, displayId, defaultVal) {
  var mul = defaultVal || 1.0;
  var input = document.getElementById(inputId);
  var display = document.getElementById(displayId);
  if (input) {
    input.value = mul;
    input.addEventListener('input', function() {
      mul = parseFloat(input.value);
      if (display) display.textContent = mul.toFixed(1) + '\u00d7';
    });
  }
  return { get: function() { return mul; } };
};
