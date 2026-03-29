/**
 * trails.js — Shared trail / path-trace utilities
 *
 * All simulations use APF.createTrail / APF.pushTrailPoint so trail
 * rendering logic lives in one place.
 *
 * Gradient functions  gradFn(t) -> [r, g, b]
 *   t = 0  →  oldest point  (dim end)
 *   t = 1  →  newest point  (bright end)
 *
 * Typical usage:
 *   var trail = APF.createTrail(scene, { color: 0x33aaff, maxPoints: 500 });
 *   // each frame:
 *   APF.pushTrailPoint(trail, dronePosition);
 *   // on reset:
 *   APF.clearTrail(trail);
 *   // on mode switch (APF ↔ HPF):
 *   APF.setTrailGradient(trail, APF.trailGradients.hpf);
 */

(function (APF) {
    'use strict';

    // ─── Built-in gradient factories ─────────────────────────────────────────

    APF.trailGradients = {

        /** Cyan/teal gradient — APF mode (matches original sim-static palette). */
        apf: function (t) {
            return [0.1 + t * 0.1, 0.5 + t * 0.5, 0.8 + t * 0.2];
        },

        /** Purple gradient — HPF mode (matches original sim-static palette). */
        hpf: function (t) {
            return [0.4 + t * 0.4, 0.0 + t * 0.15, 0.7 + t * 0.3];
        },

        /**
         * Fade a solid colour from 20 % brightness (oldest) to 100 % (newest).
         * colorOrHex: THREE.Color or hex number (e.g. 0x33aaff)
         */
        fromColor: function (colorOrHex) {
            var c = new THREE.Color(colorOrHex);
            return function (t) {
                var s = 0.2 + t * 0.8;
                return [c.r * s, c.g * s, c.b * s];
            };
        },

        /** Constant colour regardless of age. */
        solid: function (colorOrHex) {
            var c = new THREE.Color(colorOrHex);
            return function () { return [c.r, c.g, c.b]; };
        },
    };

    // ─── Core API ─────────────────────────────────────────────────────────────

    /**
     * Create a trail and add its line to the scene.
     *
     * opts:
     *   maxPoints  {number}          default 500
     *   opacity    {number}          default 0.75
     *   blending   {THREE.Blending}  default NormalBlending
     *   depthWrite {boolean}         default true
     *   gradientFn {function}        gradFn(t) -> [r,g,b]  (overrides color)
     *   color      {hex number}      shorthand — creates a fromColor gradient
     *
     * Returns a trail object (pass to pushTrailPoint / clearTrail etc.).
     */
    APF.createTrail = function (scene, opts) {
        opts = opts || {};

        var maxPoints  = opts.maxPoints  || 500;
        var opacity    = opts.opacity    != null ? opts.opacity    : 0.75;
        var blending   = opts.blending   || THREE.NormalBlending;
        var depthWrite = opts.depthWrite != null ? opts.depthWrite : true;
        var gradFn     = opts.gradientFn
            || (opts.color != null
                    ? APF.trailGradients.fromColor(opts.color)
                    : APF.trailGradients.apf);

        // Pre-allocate typed arrays — no GC churn per frame
        var positions = new Float32Array(maxPoints * 3);
        var colors    = new Float32Array(maxPoints * 3);

        var geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geo.setAttribute('color',    new THREE.BufferAttribute(colors,    3));
        geo.setDrawRange(0, 0);

        var mat = new THREE.LineBasicMaterial({
            vertexColors: true,
            transparent:  true,
            opacity:      opacity,
            blending:     blending,
            depthWrite:   depthWrite,
        });

        var line = new THREE.Line(geo, mat);
        if (scene) scene.add(line);

        return {
            line:      line,
            geo:       geo,
            positions: positions,
            colors:    colors,
            maxPoints: maxPoints,
            points:    [],   // rolling window of THREE.Vector3
            gradFn:    gradFn,
        };
    };

    /**
     * Append a new position to the trail and refresh GPU vertex buffers.
     * Call once per frame per drone with the current position.
     * v3: THREE.Vector3
     */
    APF.pushTrailPoint = function (trail, v3) {
        trail.points.push(v3.clone());
        if (trail.points.length > trail.maxPoints) trail.points.shift();

        var n = trail.points.length;
        if (n < 2) { trail.geo.setDrawRange(0, 0); return; }

        var gradFn = trail.gradFn;
        var last   = n - 1;

        for (var i = 0; i < n; i++) {
            var p   = trail.points[i];
            var idx = i * 3;
            trail.positions[idx]     = p.x;
            trail.positions[idx + 1] = p.y;
            trail.positions[idx + 2] = p.z;

            var t   = last === 0 ? 1 : i / last;
            var rgb = gradFn(t);
            trail.colors[idx]     = rgb[0];
            trail.colors[idx + 1] = rgb[1];
            trail.colors[idx + 2] = rgb[2];
        }

        trail.geo.setDrawRange(0, n);
        trail.geo.attributes.position.needsUpdate = true;
        trail.geo.attributes.color.needsUpdate    = true;
    };

    /**
     * Clear all recorded positions without destroying the trail object.
     * Use on simulation reset or teleport.
     */
    APF.clearTrail = function (trail) {
        trail.points.length = 0;
        trail.geo.setDrawRange(0, 0);
    };

    /**
     * Swap the gradient function (e.g. when the user toggles APF ↔ HPF).
     * The new gradient takes effect on the next pushTrailPoint call.
     */
    APF.setTrailGradient = function (trail, gradFn) {
        trail.gradFn = gradFn;
    };

    /** Add a trail's line back into the scene (e.g. after removing on arrival). */
    APF.addTrail = function (scene, trail) {
        if (trail && trail.line && !scene.children.includes(trail.line)) {
            scene.add(trail.line);
        }
    };

    /** Remove a trail's line from the scene (e.g. when drone reaches goal). */
    APF.removeTrail = function (scene, trail) {
        if (trail && trail.line) scene.remove(trail.line);
    };

})(window.APF = window.APF || {});
