/**
 * gpu-compute.js
 * WebGPU compute shader for APF swarm physics.
 * Offloads force integration for all drones to the GPU in parallel.
 * Falls back gracefully when WebGPU is unavailable.
 *
 * Buffer layout (per drone, 32 bytes = 2 × vec4<f32>):
 *   pos: vec4<f32>  — xyz = position, w = arrived (0 or 1)
 *   vel: vec4<f32>  — xyz = velocity, w = unused
 *
 * Params buffer layout (96 bytes = 6 × vec4<f32>):
 *   [0]  goal      — xyz = goal position
 *   [1]  config    — x = numDrones (as f32), y = numObstacles (as f32)
 *   [2]  physics   — x = dt, y = DAMP, z = VMAX, w = bound (HALF)
 *   [3]  forces    — x = K_ATT, y = K_REP, z = D0, w = D_STAR
 *   [4]  repulsion — x = K_DRONE_REP, y = D_DRONE, z = K_WALL, w = D_WALL
 *   [5]  misc      — x = GOAL_THRESH, y = NEAR_GOAL_DIST
 */

(function (APF) {
    'use strict';

    // ─── WGSL Compute Shader ──────────────────────────────────────────────────

    var WGSL_SHADER = /* wgsl */`

struct DroneState {
    pos : vec4<f32>,   // xyz = position,  w = arrived (0 or 1)
    vel : vec4<f32>,   // xyz = velocity,  w = unused
}

struct Obstacle {
    posRadius : vec4<f32>,  // xyz = position, w = radius
}

struct Params {
    goal      : vec4<f32>,  // xyz = goal, w = unused
    config    : vec4<f32>,  // x = numDrones, y = numObstacles (stored as f32)
    physics   : vec4<f32>,  // x = dt, y = DAMP, z = VMAX, w = bound
    forces    : vec4<f32>,  // x = K_ATT, y = K_REP, z = D0, w = D_STAR
    repulsion : vec4<f32>,  // x = K_DRONE_REP, y = D_DRONE, z = K_WALL, w = D_WALL
    misc      : vec4<f32>,  // x = GOAL_THRESH, y = NEAR_GOAL_DIST
}

@group(0) @binding(0) var<storage, read>       dronesIn  : array<DroneState>;
@group(0) @binding(1) var<storage, read_write> dronesOut : array<DroneState>;
@group(0) @binding(2) var<storage, read>       obstacles : array<Obstacle>;
@group(0) @binding(3) var<uniform>             params    : Params;

// APF wall repulsion: (1/d - 1/D0) / d^2
fn wallRepulse(d : f32, D0 : f32, K : f32) -> f32 {
    if (d > 0.001 && d < D0) {
        return K * (1.0 / d - 1.0 / D0) / (d * d);
    }
    return 0.0;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid : vec3<u32>) {
    let i          = gid.x;
    let numDrones  = u32(params.config.x);
    let numObs     = u32(params.config.y);

    if (i >= numDrones) { return; }

    // Always propagate current state (handles already-arrived drones)
    dronesOut[i] = dronesIn[i];
    if (dronesIn[i].pos.w > 0.5) { return; }

    let pos  = dronesIn[i].pos.xyz;
    var vel  = dronesIn[i].vel.xyz;

    let goal         = params.goal.xyz;
    let dt           = params.physics.x;
    let DAMP         = params.physics.y;
    let VMAX         = params.physics.z;
    let bound        = params.physics.w;
    let K_ATT        = params.forces.x;
    let K_REP        = params.forces.y;
    let D0           = params.forces.z;
    let D_STAR       = params.forces.w;
    let K_DRONE_REP  = params.repulsion.x;
    let D_DRONE      = params.repulsion.y;
    let K_WALL       = params.repulsion.z;
    let D_WALL       = params.repulsion.w;
    let GOAL_THRESH  = params.misc.x;
    let NEAR_GOAL    = params.misc.y;

    var F = vec3<f32>(0.0, 0.0, 0.0);

    // ── Attractive force (conic-parabolic APF) ────────────────────────
    let toGoal = goal - pos;
    let dist   = length(toGoal);

    if (dist > GOAL_THRESH) {
        if (dist <= D_STAR) {
            // Parabolic region: proportional to displacement
            F += toGoal * K_ATT;
        } else {
            // Conic region: constant magnitude
            F += (toGoal / dist) * (K_ATT * D_STAR);
        }
    }

    // ── Repulsive forces (disabled inside capture zone near goal) ─────
    if (dist >= NEAR_GOAL) {

        // Obstacle repulsion
        for (var j = 0u; j < numObs; j++) {
            let op   = obstacles[j].posRadius.xyz;
            let or_  = obstacles[j].posRadius.w;
            let diff = pos - op;
            let dlen = max(length(diff) - or_, 0.001);
            if (dlen < D0) {
                let rep   = K_REP * (1.0 / dlen - 1.0 / D0) / (dlen * dlen);
                let denom = max(length(diff), 0.0001);
                F += (diff / denom) * rep;
            }
        }

        // Inter-drone repulsion (reads pre-step positions from dronesIn — no data race)
        for (var j = 0u; j < numDrones; j++) {
            if (j == i || dronesIn[j].pos.w > 0.5) { continue; }
            let diff = pos - dronesIn[j].pos.xyz;
            let dd   = length(diff);
            if (dd > 0.001 && dd < D_DRONE) {
                let rep = K_DRONE_REP * (1.0 / dd - 1.0 / D_DRONE) / (dd * dd);
                F += (diff / dd) * rep;
            }
        }

        // Wall repulsion (per axis, APF formula)
        F.x += wallRepulse(pos.x + bound, D_WALL, K_WALL);
        F.x -= wallRepulse(bound - pos.x, D_WALL, K_WALL);
        F.y += wallRepulse(pos.y + bound, D_WALL, K_WALL);
        F.y -= wallRepulse(bound - pos.y, D_WALL, K_WALL);
        F.z += wallRepulse(pos.z + bound, D_WALL, K_WALL);
        F.z -= wallRepulse(bound - pos.z, D_WALL, K_WALL);
    }

    // ── Velocity integration ──────────────────────────────────────────
    vel  += F * dt;
    vel  *= DAMP;
    let spd = length(vel);
    if (spd > VMAX) { vel = (vel / spd) * VMAX; }

    var newPos = pos + vel;

    // Bounce off walls
    let lo = -(bound - 0.15);
    let hi =   bound - 0.15;
    if (newPos.x < lo) { newPos.x = lo; vel.x =  abs(vel.x) * 0.5; }
    if (newPos.x > hi) { newPos.x = hi; vel.x = -abs(vel.x) * 0.5; }
    if (newPos.y < lo) { newPos.y = lo; vel.y =  abs(vel.y) * 0.5; }
    if (newPos.y > hi) { newPos.y = hi; vel.y = -abs(vel.y) * 0.5; }
    if (newPos.z < lo) { newPos.z = lo; vel.z =  abs(vel.z) * 0.5; }
    if (newPos.z > hi) { newPos.z = hi; vel.z = -abs(vel.z) * 0.5; }

    // Goal arrival check
    let newDist = length(newPos - goal);
    var arrived = 0.0;
    if (newDist < GOAL_THRESH) {
        arrived = 1.0;
        vel     = vec3<f32>(0.0, 0.0, 0.0);
    }

    dronesOut[i] = DroneState(
        vec4<f32>(newPos, arrived),
        vec4<f32>(vel,    0.0)
    );
}
`;

    // ─── GPUSwarmCompute Class ────────────────────────────────────────────────

    /**
     * Manages WebGPU buffers and a compute pipeline for swarm APF physics.
     * Uses ping-pong double buffering so dronesIn always holds the
     * previous frame's state (no intra-dispatch data races).
     */
    function GPUSwarmCompute(numDrones, numObstacles) {
        this.numDrones    = numDrones;
        this.numObstacles = numObstacles;
        this.DRONE_BYTES  = numDrones    * 32;           // 2 × vec4<f32>
        this.OBS_BYTES    = Math.max(numObstacles * 16, 16); // 1 × vec4<f32>, min 16
        this.PARAMS_BYTES = 96;                          // 6 × vec4<f32>
        this.ping         = 0;
        this.ready        = false;
        this._reason      = '';

        this.device       = null;
        this.droneBuf     = null;
        this.obsBuffer    = null;
        this.paramsBuffer = null;
        this.stagingBuf   = null;
        this.pipeline     = null;
        this.bindGroups   = null;
    }

    GPUSwarmCompute.prototype.init = async function () {
        if (!navigator.gpu) {
            this._reason = 'WebGPU API unavailable — use Chrome 113+ or Safari 18+';
            return false;
        }

        var adapter;
        try {
            adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
        } catch (e) {
            this._reason = 'requestAdapter failed: ' + e.message;
            return false;
        }
        if (!adapter) {
            this._reason = 'No WebGPU adapter found on this system';
            return false;
        }

        // Store adapter info for display
        if (adapter.info) {
            this.adapterInfo = (adapter.info.vendor || '') + ' / ' + (adapter.info.architecture || '');
        } else {
            this.adapterInfo = 'WebGPU';
        }

        try {
            this.device = await adapter.requestDevice();
        } catch (e) {
            this._reason = 'requestDevice failed: ' + e.message;
            return false;
        }

        var self = this;
        this.device.lost.then(function (info) {
            console.warn('[gpu-compute] Device lost:', info.message);
            self.ready = false;
        });

        // ── Create GPU buffers ────────────────────────────────────────
        var droneUsage = GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST | GPUBufferUsage.COPY_SRC;
        this.droneBuf = [
            this.device.createBuffer({ size: this.DRONE_BYTES, usage: droneUsage }),
            this.device.createBuffer({ size: this.DRONE_BYTES, usage: droneUsage }),
        ];

        this.obsBuffer = this.device.createBuffer({
            size:  this.OBS_BYTES,
            usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST,
        });

        this.paramsBuffer = this.device.createBuffer({
            size:  this.PARAMS_BYTES,
            usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
        });

        this.stagingBuf = this.device.createBuffer({
            size:  this.DRONE_BYTES,
            usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST,
        });

        // ── Compile shader ────────────────────────────────────────────
        var shaderModule = this.device.createShaderModule({ code: WGSL_SHADER });

        // Check for WGSL compilation errors (async, not all browsers expose this)
        if (typeof shaderModule.getCompilationInfo === 'function') {
            var info = await shaderModule.getCompilationInfo();
            var errs = info.messages.filter(function (m) { return m.type === 'error'; });
            if (errs.length > 0) {
                this._reason = 'WGSL compile error: ' + errs[0].message;
                console.error('[gpu-compute] WGSL errors:', errs);
                return false;
            }
        }

        // ── Build pipeline ────────────────────────────────────────────
        var bgl = this.device.createBindGroupLayout({
            entries: [
                { binding: 0, visibility: GPUShaderStage.COMPUTE, buffer: { type: 'read-only-storage' } },
                { binding: 1, visibility: GPUShaderStage.COMPUTE, buffer: { type: 'storage'           } },
                { binding: 2, visibility: GPUShaderStage.COMPUTE, buffer: { type: 'read-only-storage' } },
                { binding: 3, visibility: GPUShaderStage.COMPUTE, buffer: { type: 'uniform'           } },
            ],
        });

        this.pipeline = this.device.createComputePipeline({
            layout:  this.device.createPipelineLayout({ bindGroupLayouts: [bgl] }),
            compute: { module: shaderModule, entryPoint: 'main' },
        });

        // ── Create two bind groups for ping-pong ──────────────────────
        var self = this;
        function makeBG(inBuf, outBuf) {
            return self.device.createBindGroup({
                layout:  bgl,
                entries: [
                    { binding: 0, resource: { buffer: inBuf            } },
                    { binding: 1, resource: { buffer: outBuf           } },
                    { binding: 2, resource: { buffer: self.obsBuffer   } },
                    { binding: 3, resource: { buffer: self.paramsBuffer} },
                ],
            });
        }

        this.bindGroups = [
            makeBG(this.droneBuf[0], this.droneBuf[1]),  // ping=0: A→B
            makeBG(this.droneBuf[1], this.droneBuf[0]),  // ping=1: B→A
        ];

        this.ready = true;
        return true;
    };

    /** Upload initial drone states (must call before first step). */
    GPUSwarmCompute.prototype.uploadDrones = function (data) {
        // data: Float32Array [px,py,pz,arrived, vx,vy,vz,pad, ...]
        this.device.queue.writeBuffer(this.droneBuf[this.ping], 0, data);
    };

    /** Upload physics params (call once or when params change). */
    GPUSwarmCompute.prototype.uploadParams = function (data) {
        // data: Float32Array of 24 f32 values (6 vec4s)
        this.device.queue.writeBuffer(this.paramsBuffer, 0, data);
    };

    /** Upload current obstacle positions/radii (call each frame). */
    GPUSwarmCompute.prototype.uploadObstacles = function (data) {
        // data: Float32Array [ox,oy,oz,radius, ...]
        this.device.queue.writeBuffer(this.obsBuffer, 0, data);
    };

    /**
     * Dispatch one physics step on the GPU.
     * Returns a Float32Array with the new drone states (same layout as uploadDrones).
     * Internally does a readback via a staging buffer.
     */
    GPUSwarmCompute.prototype.step = async function () {
        var p      = this.ping;
        var outBuf = this.droneBuf[1 - p];

        var encoder = this.device.createCommandEncoder();

        var pass = encoder.beginComputePass();
        pass.setPipeline(this.pipeline);
        pass.setBindGroup(0, this.bindGroups[p]);
        pass.dispatchWorkgroups(Math.ceil(this.numDrones / 64));
        pass.end();

        // Copy compute output → staging buffer for CPU readback
        encoder.copyBufferToBuffer(outBuf, 0, this.stagingBuf, 0, this.DRONE_BYTES);

        this.device.queue.submit([encoder.finish()]);

        // Map staging buffer and read results
        await this.stagingBuf.mapAsync(GPUMapMode.READ);
        var result = new Float32Array(this.stagingBuf.getMappedRange().slice(0));
        this.stagingBuf.unmap();

        // Swap ping-pong: this frame's output is next frame's input
        this.ping = 1 - p;

        return result;
    };

    GPUSwarmCompute.prototype.destroy = function () {
        if (this.droneBuf)     { this.droneBuf.forEach(function (b) { b.destroy(); }); }
        if (this.obsBuffer)    { this.obsBuffer.destroy(); }
        if (this.paramsBuffer) { this.paramsBuffer.destroy(); }
        if (this.stagingBuf)   { this.stagingBuf.destroy(); }
    };

    // ─── Public API ───────────────────────────────────────────────────────────

    APF.GPUSwarmCompute = GPUSwarmCompute;

    /**
     * Quick WebGPU capability check.
     * Returns { ok: bool, msg: string }
     */
    APF.checkWebGPUSupport = async function () {
        if (!navigator.gpu) {
            return { ok: false, msg: 'navigator.gpu not available — Chrome 113+ or Safari 18+ required' };
        }
        var adapter;
        try { adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' }); }
        catch (e) { return { ok: false, msg: 'requestAdapter error: ' + e.message }; }
        if (!adapter) {
            return { ok: false, msg: 'No WebGPU adapter (check browser GPU flags)' };
        }
        var info = adapter.info || {};
        var label = [info.vendor, info.architecture].filter(Boolean).join(' / ') || 'WebGPU';
        return { ok: true, msg: label };
    };

})(window.APF = window.APF || {});
