/**
 * sim-swarm-large.js
 * 100-drone APF swarm simulation with WebGPU-accelerated physics.
 *
 * Physics are computed in a WGSL compute shader on the GPU (via gpu-compute.js).
 * Falls back to an equivalent JS CPU path when WebGPU is unavailable.
 *
 * Rendering uses THREE.InstancedMesh — all 100 drones in a single draw call.
 *
 * Drone state buffer layout (8 × f32 = 32 bytes per drone):
 *   [0,1,2] position xyz
 *   [3]     arrived  (0 = flying, 1 = arrived)
 *   [4,5,6] velocity xyz
 *   [7]     padding
 */

(function () {
    'use strict';

    // ─── Simulation constants ─────────────────────────────────────────────────

    var NUM_DRONES    = 100;
    var NUM_OBSTACLES = 20;
    var HALF          = 7;        // cube extends ±HALF on each axis

    var GOAL = { x: 0, y: 0, z: 0 };

    // APF physics parameters
    var P = {
        K_ATT:       2.5,   // attractive gain
        D_STAR:      3.5,   // parabolic/conic switch distance
        K_REP:       14.0,  // obstacle repulsion gain
        D0:          2.5,   // obstacle repulsion cutoff
        K_DRONE_REP: 2.0,   // inter-drone repulsion gain
        D_DRONE:     1.2,   // inter-drone repulsion range
        K_WALL:      9.0,   // wall repulsion gain
        D_WALL:      1.8,   // wall repulsion cutoff
        DT:          0.012, // time step  (matches sim-swarm)
        DAMP:        0.82,  // velocity damping per step
        VMAX:        0.08,  // maximum speed (units / step)
        GOAL_THRESH: 0.55,  // arrival distance
        NEAR_GOAL:   2.0,   // disable repulsion inside this radius of goal
    };

    // ─── Simulation state ─────────────────────────────────────────────────────

    var running      = false;
    var arrivedCount = 0;
    var frame        = 0;
    var speedMult    = 1.0;

    var gpuCompute   = null;   // GPUSwarmCompute instance, or null
    var gpuActive    = false;  // true when GPU path is live

    // Flat typed arrays shared between JS and GPU
    var droneData  = new Float32Array(NUM_DRONES * 8);
    var obsData    = new Float32Array(NUM_OBSTACLES * 4);
    var paramsData = new Float32Array(24);

    var obstacles   = [];   // [{mesh, vel, radius}]
    var droneColors = [];   // THREE.Color per drone (rainbow)
    var droneTrails = [];   // APF trail object per drone

    // Three.js handles
    var engine, scene, camera, renderer;
    var droneMesh;          // THREE.InstancedMesh
    var goalMarker, goalRings;

    var _dummy = new THREE.Object3D();   // reused for matrix construction
    var _gold  = new THREE.Color(1.0, 0.84, 0.0);

    // ─── Helpers ──────────────────────────────────────────────────────────────

    function hslColor(h) {
        // h in [0,1] → THREE.Color via HSL
        var c = new THREE.Color();
        c.setHSL(h, 1.0, 0.62);
        return c;
    }

    function sqDist3(ax, ay, az, bx, by, bz) {
        var dx = ax - bx, dy = ay - by, dz = az - bz;
        return dx*dx + dy*dy + dz*dz;
    }

    // ─── Drone initialisation ─────────────────────────────────────────────────

    function initDronePositions() {
        var minDistSq = 4.0 * 4.0;   // keep drones ≥ 4 units from goal on start
        for (var i = 0; i < NUM_DRONES; i++) {
            var base = i * 8;
            var px, py, pz, tries = 0;
            do {
                px = (Math.random() * 2.0 - 1.0) * (HALF - 0.8);
                py = (Math.random() * 2.0 - 1.0) * (HALF - 0.8);
                pz = (Math.random() * 2.0 - 1.0) * (HALF - 0.8);
                tries++;
            } while (tries < 30 && sqDist3(px, py, pz, GOAL.x, GOAL.y, GOAL.z) < minDistSq);

            droneData[base + 0] = px;
            droneData[base + 1] = py;
            droneData[base + 2] = pz;
            droneData[base + 3] = 0;   // not arrived
            droneData[base + 4] = 0;   // vx
            droneData[base + 5] = 0;   // vy
            droneData[base + 6] = 0;   // vz
            droneData[base + 7] = 0;   // pad
        }
    }

    function buildParamsData() {
        var dt   = P.DT   * speedMult;
        var vmax = P.VMAX * speedMult;

        paramsData[ 0] = GOAL.x; paramsData[ 1] = GOAL.y; paramsData[ 2] = GOAL.z; paramsData[ 3] = 0;
        paramsData[ 4] = NUM_DRONES; paramsData[ 5] = NUM_OBSTACLES; paramsData[ 6] = 0; paramsData[ 7] = 0;
        paramsData[ 8] = dt; paramsData[ 9] = P.DAMP; paramsData[10] = vmax; paramsData[11] = HALF;
        paramsData[12] = P.K_ATT; paramsData[13] = P.K_REP; paramsData[14] = P.D0; paramsData[15] = P.D_STAR;
        paramsData[16] = P.K_DRONE_REP; paramsData[17] = P.D_DRONE; paramsData[18] = P.K_WALL; paramsData[19] = P.D_WALL;
        paramsData[20] = P.GOAL_THRESH; paramsData[21] = P.NEAR_GOAL; paramsData[22] = 0; paramsData[23] = 0;
    }

    function packObstacleData() {
        for (var i = 0; i < obstacles.length; i++) {
            var b = i * 4;
            var p = obstacles[i].mesh.position;
            obsData[b + 0] = p.x;
            obsData[b + 1] = p.y;
            obsData[b + 2] = p.z;
            obsData[b + 3] = obstacles[i].radius;
        }
    }

    // ─── CPU physics fallback ─────────────────────────────────────────────────
    // Mirrors the WGSL shader exactly so CPU and GPU produce identical results.

    function wallRepulse(d, D0, K) {
        if (d > 0.001 && d < D0) return K * (1.0 / d - 1.0 / D0) / (d * d);
        return 0.0;
    }

    function cpuStep() {
        var dt   = P.DT   * speedMult;
        var vmax = P.VMAX * speedMult;
        var lo   = -(HALF - 0.15);
        var hi   =   HALF - 0.15;

        for (var i = 0; i < NUM_DRONES; i++) {
            var base = i * 8;
            if (droneData[base + 3] > 0.5) continue;

            var px = droneData[base + 0];
            var py = droneData[base + 1];
            var pz = droneData[base + 2];
            var vx = droneData[base + 4];
            var vy = droneData[base + 5];
            var vz = droneData[base + 6];

            // ── Attractive force ──────────────────────────────────────
            var gx   = GOAL.x - px;
            var gy   = GOAL.y - py;
            var gz   = GOAL.z - pz;
            var dist = Math.sqrt(gx*gx + gy*gy + gz*gz);

            var Fx = 0, Fy = 0, Fz = 0;

            if (dist > P.GOAL_THRESH) {
                if (dist <= P.D_STAR) {
                    Fx += gx * P.K_ATT;
                    Fy += gy * P.K_ATT;
                    Fz += gz * P.K_ATT;
                } else {
                    var scale = P.K_ATT * P.D_STAR / dist;
                    Fx += gx * scale;
                    Fy += gy * scale;
                    Fz += gz * scale;
                }
            }

            if (dist >= P.NEAR_GOAL) {
                // ── Obstacle repulsion ────────────────────────────────
                for (var j = 0; j < obstacles.length; j++) {
                    var ob   = j * 4;
                    var odx  = px - obsData[ob + 0];
                    var ody  = py - obsData[ob + 1];
                    var odz  = pz - obsData[ob + 2];
                    var olen = Math.sqrt(odx*odx + ody*ody + odz*odz);
                    var d    = Math.max(olen - obsData[ob + 3], 0.001);
                    if (d < P.D0) {
                        var rep  = P.K_REP * (1.0/d - 1.0/P.D0) / (d * d);
                        var norm = Math.max(olen, 0.0001);
                        Fx += (odx / norm) * rep;
                        Fy += (ody / norm) * rep;
                        Fz += (odz / norm) * rep;
                    }
                }

                // ── Inter-drone repulsion ─────────────────────────────
                for (var j = 0; j < NUM_DRONES; j++) {
                    if (j === i) continue;
                    var jb = j * 8;
                    if (droneData[jb + 3] > 0.5) continue;
                    var ddx = px - droneData[jb + 0];
                    var ddy = py - droneData[jb + 1];
                    var ddz = pz - droneData[jb + 2];
                    var dd  = Math.sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
                    if (dd > 0.001 && dd < P.D_DRONE) {
                        var rep = P.K_DRONE_REP * (1.0/dd - 1.0/P.D_DRONE) / (dd * dd);
                        Fx += (ddx / dd) * rep;
                        Fy += (ddy / dd) * rep;
                        Fz += (ddz / dd) * rep;
                    }
                }

                // ── Wall repulsion ────────────────────────────────────
                Fx += wallRepulse(px + HALF,  P.D_WALL, P.K_WALL) - wallRepulse(HALF - px, P.D_WALL, P.K_WALL);
                Fy += wallRepulse(py + HALF,  P.D_WALL, P.K_WALL) - wallRepulse(HALF - py, P.D_WALL, P.K_WALL);
                Fz += wallRepulse(pz + HALF,  P.D_WALL, P.K_WALL) - wallRepulse(HALF - pz, P.D_WALL, P.K_WALL);
            }

            // ── Integrate velocity ────────────────────────────────────
            vx = (vx + Fx * dt) * P.DAMP;
            vy = (vy + Fy * dt) * P.DAMP;
            vz = (vz + Fz * dt) * P.DAMP;

            var spd = Math.sqrt(vx*vx + vy*vy + vz*vz);
            if (spd > vmax) { var s = vmax / spd; vx *= s; vy *= s; vz *= s; }

            var npx = px + vx;
            var npy = py + vy;
            var npz = pz + vz;

            // Bounce off walls
            if (npx < lo) { npx = lo; vx =  Math.abs(vx) * 0.5; }
            if (npx > hi) { npx = hi; vx = -Math.abs(vx) * 0.5; }
            if (npy < lo) { npy = lo; vy =  Math.abs(vy) * 0.5; }
            if (npy > hi) { npy = hi; vy = -Math.abs(vy) * 0.5; }
            if (npz < lo) { npz = lo; vz =  Math.abs(vz) * 0.5; }
            if (npz > hi) { npz = hi; vz = -Math.abs(vz) * 0.5; }

            var nd = Math.sqrt(sqDist3(npx, npy, npz, GOAL.x, GOAL.y, GOAL.z));
            var arrived = nd < P.GOAL_THRESH ? 1.0 : 0.0;
            if (arrived) { vx = 0; vy = 0; vz = 0; }

            droneData[base + 0] = npx;
            droneData[base + 1] = npy;
            droneData[base + 2] = npz;
            droneData[base + 3] = arrived;
            droneData[base + 4] = vx;
            droneData[base + 5] = vy;
            droneData[base + 6] = vz;
        }
    }

    // ─── Three.js scene setup ─────────────────────────────────────────────────

    function setupScene() {
        engine   = APF.initEngine('canvas-container', { fov: 55 });
        scene    = engine.scene;
        camera   = engine.camera;
        renderer = engine.renderer;

        APF.initCameraControls(renderer, camera, {
            rotSpeed: 0.004,
            zoomSpeed: 0.12,
            minDist:  4,
            maxDist:  35,
            initPos:  { x: 2, y: 8, z: 22 },
        });

        APF.setupLighting(scene);
        APF.setupCube(scene, HALF);

        // Goal marker
        var gv       = new THREE.Vector3(GOAL.x, GOAL.y, GOAL.z);
        goalMarker   = APF.makeMarker(scene, gv, 0xff4444, 0x220000, 0.35);
        goalRings    = APF.createGoalRings(scene, gv);

        // Instanced mesh — one OctahedronGeometry per drone, single draw call
        var geo   = new THREE.OctahedronGeometry(0.18);
        var mat   = new THREE.MeshPhongMaterial({ shininess: 90 });
        droneMesh = new THREE.InstancedMesh(geo, mat, NUM_DRONES);
        droneMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        scene.add(droneMesh);

        // Rainbow colours + per-drone trails
        for (var i = 0; i < NUM_DRONES; i++) {
            var col = hslColor(i / NUM_DRONES);
            droneColors.push(col);
            droneTrails.push(APF.createTrail(scene, {
                color:     col,
                maxPoints: 100,
                opacity:   0.55,
            }));
        }

        updateDroneInstances();

        // Moving obstacles
        obstacles = APF.createMovingObstacles(scene, NUM_OBSTACLES, HALF, []);
    }

    function updateDroneInstances() {
        for (var i = 0; i < NUM_DRONES; i++) {
            var b        = i * 8;
            var isArrived = droneData[b + 3] > 0.5;

            _dummy.position.set(droneData[b], droneData[b + 1], droneData[b + 2]);
            // Yaw toward velocity direction
            _dummy.rotation.y = Math.atan2(droneData[b + 4], droneData[b + 6]);
            // Arrived drones pulse slightly larger
            _dummy.scale.setScalar(isArrived ? 1.4 : 1.0);
            _dummy.updateMatrix();

            droneMesh.setMatrixAt(i, _dummy.matrix);
            droneMesh.setColorAt(i, isArrived ? _gold : droneColors[i]);
        }
        droneMesh.instanceMatrix.needsUpdate = true;
        if (droneMesh.instanceColor) droneMesh.instanceColor.needsUpdate = true;
    }

    var _trailPos = new THREE.Vector3();  // reused each frame to avoid GC

    function updateDroneTrails() {
        for (var i = 0; i < NUM_DRONES; i++) {
            var b = i * 8;
            if (droneData[b + 3] > 0.5) continue;  // skip arrived drones
            _trailPos.set(droneData[b], droneData[b + 1], droneData[b + 2]);
            APF.pushTrailPoint(droneTrails[i], _trailPos);
        }
    }

    function countArrived() {
        var n = 0;
        for (var i = 0; i < NUM_DRONES; i++) {
            if (droneData[i * 8 + 3] > 0.5) n++;
        }
        return n;
    }

    // ─── GPU initialisation ───────────────────────────────────────────────────

    async function initGPU() {
        var statusEl  = document.getElementById('sGPUStatus');
        var backendEl = document.getElementById('sGPUBackend');

        // Respect the kill switch set on the home page
        if (localStorage.getItem('apf_gpu_enabled') === 'false') {
            gpuActive = false;
            gpuCompute = null;
            if (statusEl)  { statusEl.textContent = 'CPU FALLBACK'; statusEl.style.color = '#ff8844'; }
            if (backendEl) { backendEl.textContent = 'GPU disabled via home page toggle'; backendEl.style.color = '#888'; }
            return;
        }

        gpuCompute = new APF.GPUSwarmCompute(NUM_DRONES, NUM_OBSTACLES);
        var ok = await gpuCompute.init();

        if (!ok) {
            var reason = gpuCompute._reason || 'WebGPU unavailable';
            console.warn('[gpu] Falling back to CPU:', reason);
            gpuActive  = false;
            gpuCompute = null;
            if (statusEl)  { statusEl.textContent  = 'CPU FALLBACK'; statusEl.style.color  = '#ff8844'; }
            if (backendEl) { backendEl.textContent  = reason;         backendEl.style.color = '#888';    }
            return;
        }

        buildParamsData();
        gpuCompute.uploadDrones(droneData);
        gpuCompute.uploadParams(paramsData);
        packObstacleData();
        gpuCompute.uploadObstacles(obsData);

        gpuActive = true;
        var label = gpuCompute.adapterInfo || 'WebGPU';
        if (statusEl)  { statusEl.textContent  = 'GPU ACTIVE \u2736'; statusEl.style.color  = '#00ffe7'; }
        if (backendEl) { backendEl.textContent  = label;              backendEl.style.color = '#00ffe7'; }
    }

    // ─── FPS tracking ─────────────────────────────────────────────────────────

    var _fpsLast  = 0;
    var _fpsCnt   = 0;
    var _fps      = 0;

    function trackFPS(now) {
        _fpsCnt++;
        if (now - _fpsLast >= 1000) {
            _fps    = Math.round(_fpsCnt * 1000 / (now - _fpsLast));
            _fpsCnt = 0;
            _fpsLast = now;
        }
    }

    // ─── Animation loop ───────────────────────────────────────────────────────

    async function animate() {
        requestAnimationFrame(animate);

        var now = performance.now();
        trackFPS(now);

        if (running && arrivedCount < NUM_DRONES) {
            APF.updateMovingObstacles(obstacles, HALF);
            packObstacleData();

            if (gpuActive && gpuCompute) {
                // Re-upload params each frame (speed multiplier may change)
                buildParamsData();
                gpuCompute.uploadParams(paramsData);
                gpuCompute.uploadObstacles(obsData);

                var result;
                try {
                    result = await gpuCompute.step();
                    droneData.set(result);
                } catch (e) {
                    console.error('[gpu] step() failed, switching to CPU:', e);
                    gpuActive = false;
                    gpuCompute = null;
                    var sEl = document.getElementById('sGPUStatus');
                    if (sEl) { sEl.textContent = 'CPU FALLBACK'; sEl.style.color = '#ff8844'; }
                    cpuStep();
                }
            } else {
                cpuStep();
            }

            arrivedCount = countArrived();
            updateDroneInstances();
            updateDroneTrails();
            frame++;

            if (arrivedCount >= NUM_DRONES) {
                document.getElementById('goal-overlay').style.display = 'flex';
            }
        }

        APF.animateGoalRings(goalRings, frame);

        if (frame % 4 === 0) updateHUD();

        renderer.render(scene, camera);
    }

    function updateHUD() {
        var el;
        if ((el = document.getElementById('sArrived'))) el.textContent = arrivedCount + ' / ' + NUM_DRONES;
        if ((el = document.getElementById('sFPS')))     el.textContent = _fps + ' fps';
    }

    // ─── Controls ─────────────────────────────────────────────────────────────

    window.togglePlay = function () {
        running = !running;
        var btn = document.getElementById('btnStart');
        if (btn) btn.textContent = running ? '\u23F8 PAUSE' : '\u25B6 LAUNCH';
        var status = document.getElementById('sStatus');
        if (status) status.textContent = running ? 'ACTIVE' : 'PAUSED';
    };

    window.resetSim = function () {
        running      = false;
        arrivedCount = 0;
        frame        = 0;

        document.getElementById('goal-overlay').style.display = 'none';
        var btn = document.getElementById('btnStart');
        if (btn) btn.textContent = '\u25B6 LAUNCH';
        var status = document.getElementById('sStatus');
        if (status) status.textContent = 'STANDBY';

        // Remove old obstacle meshes from scene, then recreate
        obstacles.forEach(function (obs) { scene.remove(obs.mesh); });
        obstacles = APF.createMovingObstacles(scene, NUM_OBSTACLES, HALF, []);

        initDronePositions();

        if (gpuActive && gpuCompute) {
            gpuCompute.ping = 0;
            buildParamsData();
            gpuCompute.uploadDrones(droneData);
            gpuCompute.uploadParams(paramsData);
            packObstacleData();
            gpuCompute.uploadObstacles(obsData);
        }

        droneTrails.forEach(function (t) { APF.clearTrail(t); });
        updateDroneInstances();
        updateHUD();
    };

    // ─── Boot ─────────────────────────────────────────────────────────────────

    document.addEventListener('DOMContentLoaded', async function () {
        initDronePositions();
        setupScene();
        await initGPU();

        _fpsLast = performance.now();
        animate();

        var speedInput = document.getElementById('speed-input');
        if (speedInput) {
            speedInput.addEventListener('input', function () {
                speedMult = parseFloat(this.value);
                var lbl = document.getElementById('speed-val');
                if (lbl) lbl.textContent = speedMult.toFixed(1) + '\u00D7';
            });
        }
    });

})();
