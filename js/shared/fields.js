'use strict';

window.APF = window.APF || {};

/**
 * APF obstacle repulsion force.
 * Formula: K_REP * (1/d - 1/D0) / d²  — zero outside D0 (hard cutoff).
 *
 * @param {THREE.Vector3} pos       drone position
 * @param {Array}         obstacles [{ pos: THREE.Vector3, radius: Number }]
 * @param {Object}        p         { K_REP, D0 }
 * @returns {THREE.Vector3}
 */
APF.obstacleForceAPF = function(pos, obstacles, p) {
    var F = new THREE.Vector3();
    for (var i = 0; i < obstacles.length; i++) {
        var obs  = obstacles[i];
        var diff = pos.clone().sub(obs.pos);
        var d    = Math.max(diff.length() - obs.radius, 0.05);
        if (d < p.D0) {
            F.add(diff.normalize().multiplyScalar(p.K_REP * (1/d - 1/p.D0) / (d * d)));
        }
    }
    return F;
};

/**
 * HPF obstacle repulsion force.
 * Formula: K_REP / d²  (Coulomb, no hard cutoff — globally aware).
 *
 * @param {THREE.Vector3} pos       drone position
 * @param {Array}         obstacles [{ pos: THREE.Vector3, radius: Number }]
 * @param {Object}        p         { K_REP, D_MIN }
 * @returns {THREE.Vector3}
 */
APF.obstacleForceHPF = function(pos, obstacles, p) {
    var F = new THREE.Vector3();
    for (var i = 0; i < obstacles.length; i++) {
        var obs  = obstacles[i];
        var diff = pos.clone().sub(obs.pos);
        var d    = Math.max(diff.length() - obs.radius, p.D_MIN);
        F.add(diff.normalize().multiplyScalar(p.K_REP / (d * d)));
    }
    return F;
};

/**
 * APF wall repulsion force.
 * Formula: K_WALL * (1/d - 1/D_WALL) / d²  — zero outside D_WALL.
 *
 * @param {THREE.Vector3} pos    drone position
 * @param {Number}        bound  half-extent of the bounding cube
 *                               (e.g. HALF for static, HALF-0.25 for moving/swarm)
 * @param {Object}        p      { K_WALL, D_WALL }
 * @param {Number}       [scale] optional force multiplier (default 1.0)
 * @returns {THREE.Vector3}
 */
APF.wallForceAPF = function(pos, bound, p, scale) {
    if (scale === undefined) scale = 1.0;
    var F    = new THREE.Vector3();
    var axes = ['x', 'y', 'z'];
    for (var a = 0; a < axes.length; a++) {
        var ax   = axes[a];
        var v    = pos[ax];
        var dPos = bound - v;          // dist to + wall
        if (dPos < p.D_WALL && dPos > 0.001)
            F[ax] -= p.K_WALL * (1/dPos - 1/p.D_WALL) / (dPos * dPos) * scale;
        var dNeg = v + bound;          // dist to − wall
        if (dNeg < p.D_WALL && dNeg > 0.001)
            F[ax] += p.K_WALL * (1/dNeg - 1/p.D_WALL) / (dNeg * dNeg) * scale;
    }
    return F;
};

/**
 * HPF wall repulsion force.
 * Formula: K_WALL / d²  (Coulomb, no hard cutoff).
 *
 * @param {THREE.Vector3} pos    drone position
 * @param {Number}        bound  half-extent of the bounding cube
 * @param {Object}        p      { K_WALL }
 * @returns {THREE.Vector3}
 */
APF.wallForceHPF = function(pos, bound, p) {
    var F    = new THREE.Vector3();
    var axes = ['x', 'y', 'z'];
    for (var a = 0; a < axes.length; a++) {
        var ax   = axes[a];
        var v    = pos[ax];
        var dPos = Math.max(bound - v, 0.1);   // dist to + wall
        var dNeg = Math.max(v + bound, 0.1);   // dist to − wall
        F[ax] -= p.K_WALL / (dPos * dPos);
        F[ax] += p.K_WALL / (dNeg * dNeg);
    }
    return F;
};
