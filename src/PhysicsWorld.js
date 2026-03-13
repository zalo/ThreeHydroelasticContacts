import * as THREE from '../node_modules/three/build/three.module.js';
import PhysXInit from '../dist/physx-js-webidl.mjs';

/**
 * PhysX WASM integration with hydroelastic contact forces.
 *
 * Uses Drake's compliant hydroelastic contact model:
 *   F_n = k * depth * (1 + d * v_n)           (Hunt-Crossley dissipation)
 *   F_t = -μ * |F_n| * v_slip / max(|v_slip|, v_stiction)  (regularized Coulomb)
 */
export default class PhysicsWorld {

    constructor() {
        this.ready = false;
        this.bodies = [];           // { pxActor, threeMesh, isKinematic }
        this.hydroelasticPairs = []; // { bodyA, bodyB } — pairs using hydroelastic contacts
        this.gripperFingers = [];    // { link, joint, targetAngle }
        this.useHydroelastic = true; // false = standard PhysX contact

        // For velocity-from-drag computation
        this._lastGripperPos = new THREE.Vector3();
        this._gripperVel = new THREE.Vector3();
        this._dragActive = false;

        // Drake hydroelastic parameters
        this.params = {
            stiffness: 5e4,       // Hydroelastic modulus (Pa/m) — rubber-like
            dissipation: 1.0,     // Hunt-Crossley dissipation (s/m)
            friction: 0.8,        // Coulomb friction coefficient μ
            stictionVel: 0.01,    // Regularization velocity for friction (m/s)
            gripForce: 1.5,       // Gripper joint drive target (radians)
            dt: 1 / 120,          // Physics timestep
            maxDepth: 0.15,       // Cap penetration depth (m) — prevents force explosion
            maxForcePerTri: 500,  // Cap force magnitude per triangle (N)
            maxTotalForce: 2000,  // Cap total force per contact pair (N)
        };

        // Temp vectors for force computation
        this._v0 = new THREE.Vector3();
        this._v1 = new THREE.Vector3();
        this._v2 = new THREE.Vector3();
        this._e1 = new THREE.Vector3();
        this._e2 = new THREE.Vector3();
        this._normal = new THREE.Vector3();
        this._relVel = new THREE.Vector3();
        this._tangent = new THREE.Vector3();
        this._force = new THREE.Vector3();
        this._centroid = new THREE.Vector3();
    }

    async init() {
        this.PhysX = await PhysXInit();
        const PX = this.PhysX;

        const version = PX.PHYSICS_VERSION;
        console.log('PhysX ' + ((version >> 24) & 0xff) + '.' + ((version >> 16) & 0xff) + '.' + ((version >> 8) & 0xff));

        const allocator = new PX.PxDefaultAllocator();
        const errorCb = new PX.PxDefaultErrorCallback();
        this.foundation = PX.CreateFoundation(version, allocator, errorCb);

        const tolerances = new PX.PxTolerancesScale();
        this.physics = PX.CreatePhysics(version, this.foundation, tolerances);

        // Scene
        const tmpVec = new PX.PxVec3(0, -9.81, 0);
        const sceneDesc = new PX.PxSceneDesc(tolerances);
        sceneDesc.set_gravity(tmpVec);
        sceneDesc.set_cpuDispatcher(PX.DefaultCpuDispatcherCreate(0));
        sceneDesc.set_filterShader(PX.DefaultFilterShader());
        this.scene = this.physics.createScene(sceneDesc);

        // Materials
        this.defaultMaterial = this.physics.createMaterial(0.5, 0.5, 0.3);
        this.highFrictionMaterial = this.physics.createMaterial(1.2, 1.0, 0.0);
        this.shapeFlags = new PX.PxShapeFlags(
            PX.PxShapeFlagEnum.eSCENE_QUERY_SHAPE |
            PX.PxShapeFlagEnum.eSIMULATION_SHAPE |
            PX.PxShapeFlagEnum.eVISUALIZATION
        );
        this.defaultFilterData = new PX.PxFilterData(1, 1, 0, 0);

        // Collision filtering:
        // word0 = "I am in group X", word1 = "I collide with groups Y"
        // Default filter shader: collide if (A.word0 & B.word1) && (B.word0 & A.word1)
        // Ground (group 1) collides with everything
        this.groundFilterData = new PX.PxFilterData(1, 0xffffffff, 0, 0);
        // Finger shapes (group 2) collide only with ground (group 1)
        this.fingerFilterData = new PX.PxFilterData(2, 1, 0, 0);
        // Object shapes (group 4) collide only with ground (group 1)
        this.objectFilterData = new PX.PxFilterData(4, 1, 0, 0);

        PX.destroy(tmpVec);
        PX.destroy(sceneDesc);
        PX.destroy(tolerances);

        this.tmpVec = new PX.PxVec3(0, 0, 0);
        this.tmpPose = new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity);

        this.ready = true;
        return this;
    }

    /** Create a static ground plane at y=0 */
    addGround() {
        const PX = this.PhysX;
        const groundGeo = new PX.PxBoxGeometry(10, 0.5, 10);
        const groundShape = this.physics.createShape(groundGeo, this.defaultMaterial, true, this.shapeFlags);
        groundShape.setSimulationFilterData(this.groundFilterData);

        this.tmpVec.set_x(0); this.tmpVec.set_y(-0.5); this.tmpVec.set_z(0);
        this.tmpPose.set_p(this.tmpVec);
        const ground = this.physics.createRigidStatic(this.tmpPose);
        ground.attachShape(groundShape);
        this.scene.addActor(ground);

        PX.destroy(groundGeo);
        this.tmpPose.set_p(new PX.PxVec3(0, 0, 0));
    }

    /**
     * Create a dynamic rigid body and link it to a three.js mesh.
     * @param {THREE.Mesh} mesh
     * @param {number} mass
     * @param {{x,y,z}} halfExtents - box half-extents for the PhysX shape
     * @param {boolean} isHydroelastic - if true, uses hydroelastic filter group
     */
    addDynamicBox(mesh, mass, halfExtents, isHydroelastic = false) {
        const PX = this.PhysX;
        const geo = new PX.PxBoxGeometry(halfExtents.x, halfExtents.y, halfExtents.z);
        const shape = this.physics.createShape(geo, this.defaultMaterial, true, this.shapeFlags);
        shape.setSimulationFilterData(isHydroelastic ? this.objectFilterData : this.defaultFilterData);

        const pos = mesh.position;
        this.tmpVec.set_x(pos.x); this.tmpVec.set_y(pos.y); this.tmpVec.set_z(pos.z);
        this.tmpPose.set_p(this.tmpVec);

        const body = this.physics.createRigidDynamic(this.tmpPose);
        body.attachShape(shape);
        PX.PxRigidBodyExt.prototype.updateMassAndInertia(body, mass / (8 * halfExtents.x * halfExtents.y * halfExtents.z));
        body.setMaxLinearVelocity(10.0);
        body.setMaxAngularVelocity(10.0);
        this.scene.addActor(body);

        const entry = { pxActor: body, threeMesh: mesh, isKinematic: false, isHydroelastic };
        this.bodies.push(entry);

        PX.destroy(geo);
        return entry;
    }

    /**
     * Create a dynamic convex-hull body from a three.js mesh geometry.
     */
    addDynamicConvex(mesh, mass, isHydroelastic = false) {
        const PX = this.PhysX;
        // Approximate with a bounding box for simplicity
        mesh.geometry.computeBoundingBox();
        const bb = mesh.geometry.boundingBox;
        const hx = (bb.max.x - bb.min.x) / 2;
        const hy = (bb.max.y - bb.min.y) / 2;
        const hz = (bb.max.z - bb.min.z) / 2;
        return this.addDynamicBox(mesh, mass, { x: hx, y: hy, z: hz }, isHydroelastic);
    }

    /**
     * Build a parallel-jaw gripper using a kinematic base + D6 joints.
     * The kinematic base can be smoothly repositioned via setKinematicTarget.
     * Each finger is a dynamic body connected to the base via a D6 joint.
     * Joint frames are rotated so the twist axis (X) aligns with world Z,
     * making the fingers swing inward/outward in the XY plane.
     *
     * @param {THREE.Group} gripperGroup - parent three.js group for visual meshes
     * @param {{x,y,z}} basePos - gripper base position in world
     */
    buildGripper(gripperGroup, basePos) {
        const PX = this.PhysX;

        // --- Kinematic base body ---
        const baseHalf = { x: 0.28, y: 0.05, z: 0.28 };
        const baseGeo = new PX.PxBoxGeometry(baseHalf.x, baseHalf.y, baseHalf.z);
        const baseShape = this.physics.createShape(baseGeo, this.highFrictionMaterial, true, this.shapeFlags);
        baseShape.setSimulationFilterData(this.fingerFilterData);

        this.tmpVec.set_x(basePos.x); this.tmpVec.set_y(basePos.y); this.tmpVec.set_z(basePos.z);
        this.tmpPose.set_p(this.tmpVec);
        const baseBody = this.physics.createRigidDynamic(this.tmpPose);
        baseBody.setRigidBodyFlag(PX.PxRigidBodyFlagEnum.eKINEMATIC, true);
        baseBody.attachShape(baseShape);
        this.scene.addActor(baseBody);

        const baseMesh = new THREE.Mesh(
            new THREE.BoxGeometry(baseHalf.x * 2, baseHalf.y * 2, baseHalf.z * 2),
            new THREE.MeshPhysicalMaterial({ color: 0x556677 })
        );
        gripperGroup.add(baseMesh);

        PX.destroy(baseGeo);

        // --- Two finger bodies connected via D6 joints ---
        const fingerHalfWidth = 0.08;
        const fingerHalfLength = 0.38;
        const fingerHalfDepth = 0.08;
        const fingerSpacing = 0.45;  // pivot distance from center

        // Quaternion to rotate joint frame so twist axis (X) → world Z axis
        // This is a -90° rotation around Y: (0, -sin(45°), 0, cos(45°))
        const s45 = Math.sin(Math.PI / 4);
        const c45 = Math.cos(Math.PI / 4);

        const fingers = [];
        const fingerMeshes = [];
        const fingerJoints = [];

        for (let side = 0; side < 2; side++) {
            const sign = side === 0 ? -1 : 1;

            // Finger dynamic body
            const fingerGeo = new PX.PxBoxGeometry(fingerHalfWidth, fingerHalfLength, fingerHalfDepth);
            const fingerShape = this.physics.createShape(fingerGeo, this.highFrictionMaterial, true, this.shapeFlags);
            fingerShape.setSimulationFilterData(this.fingerFilterData);

            // Initial position: hanging below the pivot point
            this.tmpVec.set_x(basePos.x + sign * fingerSpacing);
            this.tmpVec.set_y(basePos.y - baseHalf.y - fingerHalfLength);
            this.tmpVec.set_z(basePos.z);
            this.tmpPose.set_p(this.tmpVec);

            const fingerBody = this.physics.createRigidDynamic(this.tmpPose);
            fingerBody.attachShape(fingerShape);
            PX.PxRigidBodyExt.prototype.updateMassAndInertia(fingerBody, 2000);
            fingerBody.setLinearDamping(10.0);
            fingerBody.setAngularDamping(10.0);
            fingerBody.setMaxLinearVelocity(5.0);
            fingerBody.setMaxAngularVelocity(5.0);
            this.scene.addActor(fingerBody);

            // Joint frame rotation: twist axis (X) → world Z
            const jq = new PX.PxQuat(0, -s45, 0, c45);

            // D6 joint: base (actor0) ↔ finger (actor1)
            const baseFrame = new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity);
            const bf = new PX.PxVec3(sign * fingerSpacing, -baseHalf.y, 0);
            baseFrame.set_p(bf);
            baseFrame.set_q(jq);

            const fingerFrame = new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity);
            const ff = new PX.PxVec3(0, fingerHalfLength, 0);
            fingerFrame.set_p(ff);
            fingerFrame.set_q(jq);

            const joint = PX.D6JointCreate(this.physics, baseBody, baseFrame, fingerBody, fingerFrame);

            // Lock all DOFs except twist (now aligned with world Z → inward/outward swing)
            joint.setMotion(PX.PxD6AxisEnum.eX, PX.PxD6MotionEnum.eLOCKED);
            joint.setMotion(PX.PxD6AxisEnum.eY, PX.PxD6MotionEnum.eLOCKED);
            joint.setMotion(PX.PxD6AxisEnum.eZ, PX.PxD6MotionEnum.eLOCKED);
            joint.setMotion(PX.PxD6AxisEnum.eTWIST, PX.PxD6MotionEnum.eLIMITED);
            joint.setMotion(PX.PxD6AxisEnum.eSWING1, PX.PxD6MotionEnum.eLOCKED);
            joint.setMotion(PX.PxD6AxisEnum.eSWING2, PX.PxD6MotionEnum.eLOCKED);

            joint.setTwistLimit(new PX.PxJointAngularLimitPair(-0.8, 0.8, new PX.PxSpring(0, 0)));

            // Drive on twist axis to control open/close (high stiffness for strong grip)
            const drive = new PX.PxD6JointDrive(15000, 800, 50000, false);
            joint.setDrive(PX.PxD6DriveEnum.eTWIST, drive);
            joint.setDrivePosition(new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity));

            // Visual
            const fingerMesh = new THREE.Mesh(
                new THREE.BoxGeometry(fingerHalfWidth * 2, fingerHalfLength * 2, fingerHalfDepth * 2),
                new THREE.MeshPhysicalMaterial({ color: 0x889aab })
            );
            gripperGroup.add(fingerMesh);
            fingerMeshes.push(fingerMesh);

            fingers.push({
                link: fingerBody,
                d6joint: joint,
                mesh: fingerMesh,
                side: sign,
                halfExtents: { x: fingerHalfWidth, y: fingerHalfLength, z: fingerHalfDepth }
            });
            fingerJoints.push(joint);

            PX.destroy(fingerGeo);
            PX.destroy(bf);
            PX.destroy(ff);
            PX.destroy(jq);
        }

        this.baseBody = baseBody;
        this.gripperBase = { link: baseBody, mesh: baseMesh };
        this.gripperFingers = fingers;
        this.gripperJoints = fingerJoints;

        return { baseBody, baseMesh, fingers, fingerMeshes };
    }

    /** Set gripper drive target angle (radians). Positive = close. */
    setGripperTarget(angle) {
        const PX = this.PhysX;
        for (const finger of this.gripperFingers) {
            // Rotate around the twist axis (which is now aligned with world Z)
            // Opposite signs so fingers close toward each other
            const qa = finger.side * angle;
            const sinHalf = Math.sin(qa / 2);
            const cosHalf = Math.cos(qa / 2);
            const targetPose = new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity);
            const q = targetPose.get_q();
            q.set_x(sinHalf); q.set_y(0); q.set_z(0); q.set_w(cosHalf);
            targetPose.set_q(q);
            finger.d6joint.setDrivePosition(targetPose);
            PX.destroy(targetPose);
        }
    }

    /**
     * Move the gripper base to a new position. Uses setKinematicTarget
     * for smooth interpolation by PhysX. PhysX computes the kinematic
     * body's velocity internally from the delta between target poses,
     * which drives proper friction in the standard PhysX contact solver.
     */
    moveGripper(position) {
        const PX = this.PhysX;
        this.tmpVec.set_x(position.x); this.tmpVec.set_y(position.y); this.tmpVec.set_z(position.z);
        this.tmpPose.set_p(this.tmpVec);
        this.baseBody.setKinematicTarget(this.tmpPose);
    }

    /** Call when TransformControls drag starts */
    onDragStart(position) {
        this._lastGripperPos.copy(position);
        this._dragActive = true;
    }

    /** Call when TransformControls drag ends */
    onDragEnd() {
        this._dragActive = false;
        this._gripperVel.set(0, 0, 0);
    }

    /**
     * Update gripper velocity estimate from drag delta.
     * Call each frame during drag with the current gizmo position.
     */
    updateGripperVelocity(position, dt) {
        if (!this._dragActive || dt <= 0) return;
        this._gripperVel.subVectors(position, this._lastGripperPos).divideScalar(dt);
        this._lastGripperPos.copy(position);
    }

    /** Get the estimated gripper base velocity from dragging */
    getGripperVelocity() {
        return this._gripperVel;
    }

    /**
     * Switch between hydroelastic and standard PhysX contact.
     * Reconfigures collision filter data on finger and object shapes.
     * @param {boolean} useHydroelastic
     */
    setContactMode(useHydroelastic) {
        this.useHydroelastic = useHydroelastic;
        const PX = this.PhysX;

        if (useHydroelastic) {
            // Hydroelastic: fingers (group 2) and object (group 4) don't collide via PhysX
            this._setShapeFilter(this.baseBody, this.fingerFilterData);
            for (const finger of this.gripperFingers) {
                this._setShapeFilter(finger.link, this.fingerFilterData);
            }
            for (const body of this.bodies) {
                if (body.isHydroelastic) {
                    this._setShapeFilter(body.pxActor, this.objectFilterData);
                    body.pxActor.setSolverIterationCounts(4, 1);
                }
            }
        } else {
            // Standard PhysX: everything collides with everything, high solver iterations
            const allCollide = new PX.PxFilterData(0xff, 0xff, 0, 0);
            this._setShapeFilter(this.baseBody, allCollide);
            for (const finger of this.gripperFingers) {
                this._setShapeFilter(finger.link, allCollide);
                // 3x default solver iterations (default is 4 pos, 1 vel)
                finger.link.setSolverIterationCounts(12, 3);
            }
            for (const body of this.bodies) {
                if (body.isHydroelastic) {
                    this._setShapeFilter(body.pxActor, allCollide);
                    body.pxActor.setSolverIterationCounts(12, 3);
                }
            }
            PX.destroy(allCollide);
        }
    }

    /** Helper: set simulation filter data on all shapes of an actor */
    _setShapeFilter(actor, filterData) {
        const PX = this.PhysX;
        const numShapes = actor.getNbShapes();
        for (let i = 0; i < numShapes; i++) {
            const shape = PX.SupportFunctions.prototype.PxActor_getShape(actor, i);
            shape.setSimulationFilterData(filterData);
        }
    }

    /**
     * Register a pair of bodies for hydroelastic contact processing.
     * PhysX collision between them should be filtered out; we compute our own forces.
     */
    registerHydroelasticPair(bodyEntryA, bodyEntryB) {
        this.hydroelasticPairs.push({ bodyA: bodyEntryA, bodyB: bodyEntryB });
    }

    /**
     * Get the linear + angular velocity at a world-space point on a rigid body.
     * v = v_linear + ω × (point - COM)
     */
    getVelocityAtPoint(pxBody, worldPoint, out) {
        const lv = pxBody.getLinearVelocity();
        const av = pxBody.getAngularVelocity();
        const pose = pxBody.getGlobalPose();
        const com = pose.get_p();

        // r = worldPoint - COM
        const rx = worldPoint.x - com.get_x();
        const ry = worldPoint.y - com.get_y();
        const rz = worldPoint.z - com.get_z();

        // ω × r
        const wx = av.get_x(), wy = av.get_y(), wz = av.get_z();
        out.set(
            lv.get_x() + (wy * rz - wz * ry),
            lv.get_y() + (wz * rx - wx * rz),
            lv.get_z() + (wx * ry - wy * rx)
        );
        return out;
    }

    /**
     * Apply a force at a world-space position to a PhysX rigid body.
     */
    applyForceAtPoint(pxBody, force, point) {
        const PX = this.PhysX;
        this.tmpVec.set_x(force.x); this.tmpVec.set_y(force.y); this.tmpVec.set_z(force.z);
        const posVec = new PX.PxVec3(point.x, point.y, point.z);
        PX.PxRigidBodyExt.prototype.addForceAtPos(pxBody, this.tmpVec, posVec, PX.PxForceModeEnum.eFORCE, true);
        PX.destroy(posVec);
    }

    /**
     * Compute and apply Drake-style hydroelastic contact forces from a contact manifold.
     *
     * @param {Float32Array|number[]} vertices - triangle vertices (flat xyz)
     * @param {Float32Array|number[]} penetrationDepth - per-vertex penetration depth
     * @param {number} numTris - number of triangles
     * @param {object} bodyA - { pxActor } first body
     * @param {object} bodyB - { pxActor } second body
     * @returns {{ totalForceA: THREE.Vector3, totalForceB: THREE.Vector3 }}
     */
    applyHydroelasticForces(vertices, penetrationDepth, numTris, bodyA, bodyB) {
        if (numTris === 0) return { totalForceA: new THREE.Vector3(), totalForceB: new THREE.Vector3() };

        const { stiffness, dissipation, friction, stictionVel, maxDepth, maxForcePerTri, maxTotalForce } = this.params;
        const totalForceA = new THREE.Vector3();
        const totalForceB = new THREE.Vector3();

        const v0 = this._v0, v1 = this._v1, v2 = this._v2;
        const e1 = this._e1, e2 = this._e2;
        const normal = this._normal;
        const relVel = this._relVel;
        const tangent = this._tangent;
        const force = this._force;
        const centroid = this._centroid;

        const velA = new THREE.Vector3();
        const velB = new THREE.Vector3();

        const actorA = bodyA.pxActor;
        const actorB = bodyB.pxActor;

        // Accumulate forces first, then clamp total before applying
        const forceAccum = [];

        for (let t = 0; t < numTris; t++) {
            const base = t * 9;
            const dBase = t * 3;

            v0.set(vertices[base], vertices[base + 1], vertices[base + 2]);
            v1.set(vertices[base + 3], vertices[base + 4], vertices[base + 5]);
            v2.set(vertices[base + 6], vertices[base + 7], vertices[base + 8]);

            e1.subVectors(v1, v0);
            e2.subVectors(v2, v0);
            normal.crossVectors(e1, e2);

            const area = normal.length() * 0.5;
            if (area < 1e-10) continue;
            normal.normalize();

            // Average penetration depth, capped to prevent force explosion
            const d0 = penetrationDepth[dBase];
            const d1 = penetrationDepth[dBase + 1];
            const d2 = penetrationDepth[dBase + 2];
            const avgDepth = Math.min((d0 + d1 + d2) / 3, maxDepth);
            if (avgDepth <= 0) continue;

            // Triangle centroid
            centroid.set(
                (v0.x + v1.x + v2.x) / 3,
                (v0.y + v1.y + v2.y) / 3,
                (v0.z + v1.z + v2.z) / 3
            );

            // --- Relative velocity at centroid ---
            // vn_BqAq_W = velocity of A relative to B, projected onto normal
            // normal points from B into A (MC convention)
            // positive vn = separating, negative vn = approaching
            this.getVelocityAtPoint(actorA, centroid, velA);
            this.getVelocityAtPoint(actorB, centroid, velB);
            relVel.subVectors(velA, velB);
            const vn = relVel.dot(normal);

            // --- Normal force: Drake's Hunt-Crossley (Eq. 16 from Hunt 1975) ---
            // pressure = e * (1 - d_hc * vn), clamped >= 0
            // When approaching (vn < 0), -d*vn > 0 → increased pressure (damping)
            // When separating (vn > 0), -d*vn < 0 → decreased pressure (release)
            const e = stiffness * avgDepth;
            const c = dissipation * avgDepth;  // Drake: c = dissipation * e (pressure-proportional)
            const pressureN = Math.max(e - vn * stiffness * c, 0);
            const fn = pressureN * area;

            // Force on body A along +normal (repulsive: pushes A away from B)
            // Drake: traction_Aq_W = nhat_W * normal_traction
            force.copy(normal).multiplyScalar(fn);

            // --- Tangential force: Regularized Coulomb friction ---
            // Tangential velocity component
            tangent.copy(relVel).addScaledVector(normal, -vn);
            const vSlip = tangent.length();

            if (vSlip > 1e-8) {
                // Friction opposes relative sliding: -μ * fn * tangent_direction
                const ftMag = friction * fn * Math.min(vSlip / stictionVel, 1.0);
                tangent.normalize().multiplyScalar(-ftMag);
                force.add(tangent);
            }

            // Clamp per-triangle force magnitude
            const forceMag = force.length();
            if (forceMag > maxForcePerTri) {
                force.multiplyScalar(maxForcePerTri / forceMag);
            }

            forceAccum.push({
                fx: force.x, fy: force.y, fz: force.z,
                cx: centroid.x, cy: centroid.y, cz: centroid.z
            });
            totalForceA.add(force);
        }

        // Clamp total force magnitude across all triangles
        const totalMag = totalForceA.length();
        const scale = totalMag > maxTotalForce ? maxTotalForce / totalMag : 1.0;

        for (const f of forceAccum) {
            force.set(f.fx * scale, f.fy * scale, f.fz * scale);
            centroid.set(f.cx, f.cy, f.cz);

            this.applyForceAtPoint(actorA, force, centroid);

            force.negate();
            this.applyForceAtPoint(actorB, force, centroid);
        }

        totalForceA.multiplyScalar(scale);
        totalForceB.copy(totalForceA).negate();

        return { totalForceA, totalForceB };
    }

    /** Step the PhysX simulation */
    step(dt) {
        if (!this.ready) return;
        this.scene.simulate(dt || this.params.dt);
        this.scene.fetchResults(true);
    }

    /**
     * Sync PhysX body poses → three.js meshes
     */
    syncToThreeJS() {
        if (!this.ready) return;

        for (const entry of this.bodies) {
            if (entry.isKinematic) continue;
            const pose = entry.pxActor.getGlobalPose();
            const p = pose.get_p();
            const q = pose.get_q();
            entry.threeMesh.position.set(p.get_x(), p.get_y(), p.get_z());
            entry.threeMesh.quaternion.set(q.get_x(), q.get_y(), q.get_z(), q.get_w());
            entry.threeMesh.updateMatrix();
            entry.threeMesh.updateMatrixWorld(true);
        }

        // Sync gripper base (kinematic — driven by TransformControls, so read back from PhysX)
        if (this.gripperBase) {
            const bp = this.gripperBase.link.getGlobalPose();
            const bpp = bp.get_p();
            const bpq = bp.get_q();
            this.gripperBase.mesh.position.set(bpp.get_x(), bpp.get_y(), bpp.get_z());
            this.gripperBase.mesh.quaternion.set(bpq.get_x(), bpq.get_y(), bpq.get_z(), bpq.get_w());
            this.gripperBase.mesh.updateMatrix();
            this.gripperBase.mesh.updateMatrixWorld(true);
        }

        // Sync finger bodies (dynamic, driven by D6 joints)
        for (const finger of this.gripperFingers) {
            const fp = finger.link.getGlobalPose();
            const fpp = fp.get_p();
            const fpq = fp.get_q();
            finger.mesh.position.set(fpp.get_x(), fpp.get_y(), fpp.get_z());
            finger.mesh.quaternion.set(fpq.get_x(), fpq.get_y(), fpq.get_z(), fpq.get_w());
            finger.mesh.updateMatrix();
            finger.mesh.updateMatrixWorld(true);
        }
    }

    /**
     * Get the pose of a PhysX actor as a THREE.Matrix4 (world-to-local inverse)
     */
    getInverseMatrix(pxActor, outMatrix) {
        const pose = pxActor.getGlobalPose();
        const p = pose.get_p();
        const q = pose.get_q();

        const tmpQuat = new THREE.Quaternion(q.get_x(), q.get_y(), q.get_z(), q.get_w());
        const tmpPos = new THREE.Vector3(p.get_x(), p.get_y(), p.get_z());

        outMatrix.compose(tmpPos, tmpQuat, new THREE.Vector3(1, 1, 1));
        outMatrix.invert();
        return outMatrix;
    }
}
