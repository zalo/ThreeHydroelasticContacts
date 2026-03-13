import * as THREE from '../node_modules/three/build/three.module.js';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { mergeVertices } from '../node_modules/three/examples/jsm/utils/BufferGeometryUtils.js';
import World from './World.js';
import PhysicsWorld from './PhysicsWorld.js';

import { TransformControls } from '../node_modules/three/examples/jsm/controls/TransformControls.js';
import { edgeTable, triTable } from '../node_modules/three/examples/jsm/objects/MarchingCubes.js';

// Import the BVH Acceleration Structure and monkey-patch the Mesh class with it
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast, SAH, ExtendedTriangle, getTriangleHitPointInfo, MeshBVH } from '../node_modules/three-mesh-bvh/build/index.module.js';
THREE.Mesh.prototype.raycast = acceleratedRaycast;
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;

/** The fundamental set up and animation structures for 3D Visualization */
export default class Main {

    constructor() {
        // Intercept Main Window Errors
        window.realConsoleError = console.error;
        window.addEventListener('error', (event) => {
            let path = event.filename.split("/");
            this.display((path[path.length - 1] + ":" + event.lineno + " - " + event.message));
        });
        console.error = this.fakeError.bind(this);
        this.deferredConstructor();
    }
    async deferredConstructor() {
        // --- Physics ---
        this.physicsWorld = new PhysicsWorld();
        await this.physicsWorld.init();
        this.physicsWorld.addGround();

        // Configure Settings
        this.contactParams = {
            resolution: 20,
            gripperOpen: true,
            useHydroelastic: false,
            stiffness: 5e4,
            dissipation: 1.0,
            friction: 0.8,
            resetScene: () => this.resetScene(),
        };

        this.gui = new GUI();
        this.gui.add(this.contactParams, 'useHydroelastic').name('Hydroelastic (vs PhysX)').onChange((v) => {
            this.physicsWorld.setContactMode(v);
            // Show/hide manifold meshes
            for (const fc of this.fingerContacts) fc.manifoldMesh.visible = v;
        });
        this.gui.add(this.contactParams, 'resolution', 3, 40, 1).name('MC Resolution').onFinishChange(() => this.updateAllContacts());
        this.gui.add(this.contactParams, 'gripperOpen').name('Gripper Open').onChange((open) => {
            this.physicsWorld.setGripperTarget(open ? 0.0 : this.physicsWorld.params.gripForce);
        });

        const physFolder = this.gui.addFolder('Hydroelastic Params');
        physFolder.add(this.contactParams, 'stiffness', 1e3, 1e6).name('Stiffness (Pa/m)').onChange(v => { this.physicsWorld.params.stiffness = v; });
        physFolder.add(this.contactParams, 'dissipation', 0, 5).name('Dissipation (s/m)').onChange(v => { this.physicsWorld.params.dissipation = v; });
        physFolder.add(this.contactParams, 'friction', 0, 2).name('Friction μ').onChange(v => { this.physicsWorld.params.friction = v; });
        this.gui.add(this.contactParams, 'resetScene').name('Reset Scene');

        this.sphereGeo   = new THREE.SphereGeometry  (1.0, 32, 32);
        this.cylinderGeo = new THREE.CylinderGeometry(1, 1, 1, 8);

        this.tmpInverseMatrix1 = new THREE.Matrix4();
        this.tmpInverseMatrix2 = new THREE.Matrix4();
        this.pointLocal1 = new THREE.Vector3();
        this.pointLocal2 = new THREE.Vector3();
        this.color = new THREE.Color();
        this.closest1 = {};

        // Construct the render world
        this.world = new World(this);

        // TransformControls for repositioning the gripper
        this.control = new TransformControls( this.world.camera, this.world.renderer.domElement );
        this.control.addEventListener( 'dragging-changed', ( event ) => {
            this.world.controls.enabled = !event.value;
            if (event.value) {
                this.physicsWorld.onDragStart(this.gripper.baseMesh.position);
            } else {
                this.physicsWorld.onDragEnd();
            }
        });

        // --- Create the gripped object (Suzanne) ---
        this.objMaterial = new THREE.MeshPhysicalMaterial({ color: 0xdddddd, wireframe: true, side: THREE.FrontSide });

        this.isDeployed = document.pathname !== '/';

        // Create a simple box as the default gripped object
        this.geometry = new THREE.BoxGeometry();
        /** @type {MeshBVH} */
        this.bvh1 = this.geometry.computeBoundsTree();
        this.mesh = new THREE.Mesh(this.geometry, this.objMaterial);
        this.world.scene.add(this.mesh);
        this.mesh.position.set(0.0, 0.5, 0.0);

        // Add the box as a dynamic PhysX body
        this.objectBody = this.physicsWorld.addDynamicBox(this.mesh, 1.0, { x: 0.5, y: 0.5, z: 0.5 }, true);

        // --- Build gripper ---
        this.gripperGroup = new THREE.Group();
        this.world.scene.add(this.gripperGroup);

        const gripperBasePos = { x: 0.0, y: 3.5, z: 0.0 };
        this.gripper = this.physicsWorld.buildGripper(this.gripperGroup, gripperBasePos);

        // Attach TransformControls to the gripper base mesh for repositioning
        this.control.attach(this.gripper.baseMesh);
        this.world.scene.add(this.control);

        // When the user drags the gizmo, move the kinematic base to match
        this.control.addEventListener('objectChange', () => {
            this.physicsWorld.moveGripper(this.gripper.baseMesh.position);
        });

        // Cached inverse matrices for finger SDF sampling
        this._fingerInverseMatrices = this.gripper.fingers.map(() => new THREE.Matrix4());

        // Start in PhysX contact mode with gripper open
        this.physicsWorld.setContactMode(false);
        this.physicsWorld.setGripperTarget(0.0);

        // --- Implicit surface visualization (contact manifold) ---
        this.tempRay = new THREE.Ray();
        this.tempRay.direction.set(1, 1, 1).normalize();

        let implicitMaterial = new THREE.ShaderMaterial( {
            side: THREE.DoubleSide,
            vertexShader: `
                attribute float penetrationDepth; varying float vPenetrationDepth;
                void main() {
                    vPenetrationDepth = penetrationDepth;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
                }`,
            fragmentShader: `
                varying float vPenetrationDepth;
                vec4 turbo(float x) {
                    float r = 0.1357 + x * ( 4.5974 - x * (42.3277 - x * (130.5887 - x * (150.5666 - x * 58.1375))));
                    float g = 0.0914 + x * ( 2.1856 + x * ( 4.8052 - x * ( 14.0195 - x * (  4.2109 + x *  2.7747))));
                    float b = 0.1067 + x * (12.5925 - x * (60.1097 - x * (109.0745 - x * ( 88.5066 - x * 26.8183))));
                    return vec4(r, g, b, 1.0);
                }
                void main() {
                    gl_FragColor = turbo( clamp(vPenetrationDepth * 3.0, 0.0, 1.0) );
                }`
        });

        // One manifold mesh per finger
        this.fingerContacts = [];
        for (let i = 0; i < this.gripper.fingers.length; i++) {
            const manifoldMesh = new THREE.Mesh(new THREE.BufferGeometry(), implicitMaterial.clone());
            this.world.scene.add(manifoldMesh);
            this.fingerContacts.push({
                manifoldMesh,
                overlap: new THREE.Box3(),
                overlap2: new THREE.Box3(),
            });
        }

        this.arrowGroup = new THREE.Group();
        this.world.scene.add(this.arrowGroup);

        // Pre-compute BVH for finger geometries (simple boxes — use analytical SDF instead)
        // For fingers we use analytical signed distance (boxes), so no BVH needed
        this.fingerBVHs = [];
        for (const finger of this.gripper.fingers) {
            const geo = new THREE.BoxGeometry(
                finger.halfExtents.x * 2,
                finger.halfExtents.y * 2,
                finger.halfExtents.z * 2
            );
            const bvh = geo.computeBoundsTree();
            this.fingerBVHs.push({ geometry: geo, bvh });
        }

        // Track last frame time for physics substeps
        this.lastTimeMS = 0;
        this.accumulator = 0;
    }

    resetScene() {
        // Reset object position
        const PX = this.physicsWorld.PhysX;
        const pose = new PX.PxTransform(PX.PxIDENTITYEnum.PxIdentity);
        const p = new PX.PxVec3(0, 0.5, 0);
        pose.set_p(p);
        this.objectBody.pxActor.setGlobalPose(pose);
        this.objectBody.pxActor.setLinearVelocity(new PX.PxVec3(0, 0, 0));
        this.objectBody.pxActor.setAngularVelocity(new PX.PxVec3(0, 0, 0));

        PX.destroy(p);
        PX.destroy(pose);
    }

    updateAllContacts() {
        // Force re-evaluation on next frame
    }

    // ====================================================================
    // Marching Cubes (preserved from original, unchanged)
    // ====================================================================

    updateMarchingCubes(calculateImplicitFunction, resolution = 15,
        axisMin = new THREE.Vector3(-10, -10, -10), axisMax = new THREE.Vector3(10, 10, 10)) {
        this.points = [];
        this.values = [];
        this.depthValues = [];

        /** @type {THREE.Vector3} */
        let axisRange = axisMax.clone().sub(axisMin);
        let distanceResult = new THREE.Vector2();

        for (let k = 0; k < resolution; k++)
        for (let j = 0; j < resolution; j++)
        for (let i = 0; i < resolution; i++)
        {
            let x = axisMin.x + axisRange.x * i / (resolution - 1);
            let y = axisMin.y + axisRange.y * j / (resolution - 1);
            let z = axisMin.z + axisRange.z * k / (resolution - 1);
            this.points.push( new THREE.Vector3(x,y,z) );
            calculateImplicitFunction(x, y, z, distanceResult);
            this.values.push( (distanceResult.x - distanceResult.y) );
            this.depthValues.push( (Math.max(distanceResult.x, distanceResult.y)) );
        }

        let size2 = resolution * resolution;
        let vlist = new Array(12);
        let dvlist = new Array(12);

        let vertexIndex = 0;
        let vertices = [];
        let indices = [];
        let penetrationDepth = [];

        for (var z = 0; z < resolution - 1; z++)
        for (var y = 0; y < resolution - 1; y++)
        for (var x = 0; x < resolution - 1; x++)
        {
            let p    = x + resolution * y + size2 * z,
                px   = p   + 1,
                py   = p   + resolution,
                pxy  = py  + 1,
                pz   = p   + size2,
                pxz  = px  + size2,
                pyz  = py  + size2,
                pxyz = pxy + size2;

            let value0 = this.values[ p    ],
                value1 = this.values[ px   ],
                value2 = this.values[ py   ],
                value3 = this.values[ pxy  ],
                value4 = this.values[ pz   ],
                value5 = this.values[ pxz  ],
                value6 = this.values[ pyz  ],
                value7 = this.values[ pxyz ];

            let isolevel = 0;
            let cubeindex = 0;
            if ( value0 < isolevel ) cubeindex |= 1;
            if ( value1 < isolevel ) cubeindex |= 2;
            if ( value2 < isolevel ) cubeindex |= 8;
            if ( value3 < isolevel ) cubeindex |= 4;
            if ( value4 < isolevel ) cubeindex |= 16;
            if ( value5 < isolevel ) cubeindex |= 32;
            if ( value6 < isolevel ) cubeindex |= 128;
            if ( value7 < isolevel ) cubeindex |= 64;

            let bits = edgeTable[ cubeindex ];
            if ( bits === 0 ) continue;

            let mu = 0.5;

            if ( bits & 1 )
            {
                mu = ( isolevel - value0 ) / ( value1 - value0 );
                vlist [0] = this.points[p].clone().lerp( this.points[px], mu );
                dvlist[0] = THREE.MathUtils.lerp(this.depthValues[p], this.depthValues[px], mu);
            }
            if ( bits & 2 )
            {
                mu = ( isolevel - value1 ) / ( value3 - value1 );
                vlist [1] = this.points[px].clone().lerp( this.points[pxy], mu );
                dvlist[1] = THREE.MathUtils.lerp(this.depthValues[px], this.depthValues[pxy], mu);
            }
            if ( bits & 4 )
            {
                mu = ( isolevel - value2 ) / ( value3 - value2 );
                vlist [2] = this.points[py].clone().lerp( this.points[pxy], mu );
                dvlist[2] = THREE.MathUtils.lerp(this.depthValues[py], this.depthValues[pxy], mu);
            }
            if ( bits & 8 )
            {
                mu = ( isolevel - value0 ) / ( value2 - value0 );
                vlist [3] = this.points[p].clone().lerp( this.points[py], mu );
                dvlist[3] = THREE.MathUtils.lerp(this.depthValues[p], this.depthValues[py], mu);
            }
            if ( bits & 16 )
            {
                mu = ( isolevel - value4 ) / ( value5 - value4 );
                vlist [4] = this.points[pz].clone().lerp( this.points[pxz], mu );
                dvlist[4] = THREE.MathUtils.lerp(this.depthValues[pz], this.depthValues[pxz], mu);
            }
            if ( bits & 32 )
            {
                mu = ( isolevel - value5 ) / ( value7 - value5 );
                vlist [5] = this.points[pxz].clone().lerp( this.points[pxyz], mu );
                dvlist[5] = THREE.MathUtils.lerp(this.depthValues[pxz], this.depthValues[pxyz], mu);
            }
            if ( bits & 64 )
            {
                mu = ( isolevel - value6 ) / ( value7 - value6 );
                vlist [6] = this.points[pyz].clone().lerp( this.points[pxyz], mu );
                dvlist[6] = THREE.MathUtils.lerp(this.depthValues[pyz], this.depthValues[pxyz], mu);
            }
            if ( bits & 128 )
            {
                mu = ( isolevel - value4 ) / ( value6 - value4 );
                vlist [7] = this.points[pz].clone().lerp( this.points[pyz], mu );
                dvlist[7] = THREE.MathUtils.lerp(this.depthValues[pz], this.depthValues[pyz], mu);
            }
            if ( bits & 256 )
            {
                mu = ( isolevel - value0 ) / ( value4 - value0 );
                vlist [8] = this.points[p].clone().lerp( this.points[pz], mu );
                dvlist[8] = THREE.MathUtils.lerp(this.depthValues[p], this.depthValues[pz], mu);
            }
            if ( bits & 512 )
            {
                mu = ( isolevel - value1 ) / ( value5 - value1 );
                vlist [9] = this.points[px].clone().lerp( this.points[pxz], mu );
                dvlist[9] = THREE.MathUtils.lerp(this.depthValues[px], this.depthValues[pxz], mu);
            }
            if ( bits & 1024 )
            {
                mu = ( isolevel - value3 ) / ( value7 - value3 );
                vlist [10] = this.points[pxy].clone().lerp( this.points[pxyz], mu );
                dvlist[10] = THREE.MathUtils.lerp(this.depthValues[pxy], this.depthValues[pxyz], mu);
            }
            if ( bits & 2048 )
            {
                mu = ( isolevel - value2 ) / ( value6 - value2 );
                vlist [11] = this.points[py].clone().lerp( this.points[pyz], mu );
                dvlist[11] = THREE.MathUtils.lerp(this.depthValues[py], this.depthValues[pyz], mu);
            }

            let i = 0;
            cubeindex <<= 4;
            while ( triTable[ cubeindex + i ] != -1 )
            {
                let index1 = triTable[cubeindex + i    ];
                let index2 = triTable[cubeindex + i + 1];
                let index3 = triTable[cubeindex + i + 2];

                vertices.push( vlist[index1].x, vlist[index1].y, vlist[index1].z );
                penetrationDepth.push(dvlist[index1]);

                vertices.push( vlist[index2].x, vlist[index2].y, vlist[index2].z );
                penetrationDepth.push(dvlist[index2]);

                vertices.push( vlist[index3].x, vlist[index3].y, vlist[index3].z );
                penetrationDepth.push(dvlist[index3]);

                indices .push( vertexIndex, vertexIndex + 1, vertexIndex + 2 );
                vertexIndex += 3;
                i += 3;
            }
        }

        // Clip at penetration depth = 0
        let clipped = this.clipTriangles(vertices, penetrationDepth);
        return clipped;
    }

    clipTriangles(vertices, penetrationDepth) {
        let outVerts = [];
        let outDepths = [];
        let outIndices = [];
        let vertIdx = 0;

        let numTris = penetrationDepth.length / 3;
        for (let t = 0; t < numTris; t++) {
            let base = t * 9;
            let dBase = t * 3;

            let d = [penetrationDepth[dBase], penetrationDepth[dBase + 1], penetrationDepth[dBase + 2]];
            let v = [
                [vertices[base], vertices[base + 1], vertices[base + 2]],
                [vertices[base + 3], vertices[base + 4], vertices[base + 5]],
                [vertices[base + 6], vertices[base + 7], vertices[base + 8]]
            ];

            let numInside = (d[0] >= 0 ? 1 : 0) + (d[1] >= 0 ? 1 : 0) + (d[2] >= 0 ? 1 : 0);

            if (numInside === 0) continue;

            if (numInside === 3) {
                outVerts.push(...v[0], ...v[1], ...v[2]);
                outDepths.push(d[0], d[1], d[2]);
                outIndices.push(vertIdx, vertIdx + 1, vertIdx + 2);
                vertIdx += 3;
            } else if (numInside === 1) {
                let a = d[0] >= 0 ? 0 : (d[1] >= 0 ? 1 : 2);
                let b = (a + 1) % 3, c = (a + 2) % 3;

                let tab = d[a] / (d[a] - d[b]);
                let tac = d[a] / (d[a] - d[c]);

                let vab = v[a].map((val, i) => val + (v[b][i] - val) * tab);
                let vac = v[a].map((val, i) => val + (v[c][i] - val) * tac);

                outVerts.push(...v[a], ...vab, ...vac);
                outDepths.push(d[a], 0, 0);
                outIndices.push(vertIdx, vertIdx + 1, vertIdx + 2);
                vertIdx += 3;
            } else {
                let a = d[0] < 0 ? 0 : (d[1] < 0 ? 1 : 2);
                let b = (a + 1) % 3, c = (a + 2) % 3;

                let tba = d[b] / (d[b] - d[a]);
                let tca = d[c] / (d[c] - d[a]);

                let vba = v[b].map((val, i) => val + (v[a][i] - val) * tba);
                let vca = v[c].map((val, i) => val + (v[a][i] - val) * tca);

                outVerts.push(...v[b], ...vba, ...v[c]);
                outDepths.push(d[b], 0, d[c]);
                outIndices.push(vertIdx, vertIdx + 1, vertIdx + 2);
                vertIdx += 3;

                outVerts.push(...vba, ...vca, ...v[c]);
                outDepths.push(0, 0, d[c]);
                outIndices.push(vertIdx, vertIdx + 1, vertIdx + 2);
                vertIdx += 3;
            }
        }

        return { vertices: outVerts, penetrationDepth: outDepths, indices: outIndices };
    }

    // ====================================================================
    // SDF Evaluation (with caching for the object mesh)
    // ====================================================================

    /** @param {THREE.Mesh} mesh */
    sampleSignedDistance(mesh, x, y, z) {
        this.tempRay.origin.set(x, y, z);
        let hit1 = mesh.geometry.boundsTree.raycastFirst(this.tempRay, THREE.DoubleSide);
        mesh.geometry.boundsTree.closestPointToPoint(this.tempRay.origin, this.closest1);
        return this.closest1.distance * ((hit1 && hit1.face.normal.dot( this.tempRay.direction ) > 0.0) ? 1.0 : -1.0);
    }

    sampleCacheAtIndices(mesh, xIndex, yIndex, zIndex) {
        const dim = 72;
        let index = xIndex + yIndex * dim + zIndex * dim * dim;
        index = Math.min(Math.max(index, 0), dim * dim * dim - 1);

        if(mesh.userData.distanceCache[index] === 0.0) {
            let quantizedBoundingBoxRelative = new THREE.Vector3(Math.floor(index % dim) / dim,
                                                                 Math.floor(index / dim) % dim  / dim,
                                                                 Math.floor(index / dim  / dim) / dim);
            this.pointLocal1.copy(quantizedBoundingBoxRelative).multiply(mesh.userData.boundingBoxSize).add(mesh.userData.boundingBoxMin);

            mesh.userData.distanceCache[index] = this.sampleSignedDistance(mesh, this.pointLocal1.x, this.pointLocal1.y, this.pointLocal1.z);
        }

        return mesh.userData.distanceCache[index];
    }

    /** @param {THREE.Mesh} mesh */
    sampleSignedDistanceWithCache(mesh, x, y, z) {
        const dim = 72;
        if(!mesh.userData.distanceCache) {
            mesh.userData.distanceCache = new Float32Array(dim * dim * dim);

            mesh.userData.boundingBox     = mesh.geometry.boundingBox;
            mesh.userData.boundingBoxSize = mesh.userData.boundingBox.getSize(new THREE.Vector3());
            mesh.userData.boundingBoxMin  = mesh.userData.boundingBox.min.clone();
            mesh.userData.boundingBoxMin .subScalar(0.01);
            mesh.userData.boundingBoxSize.addScalar(0.02);

            mesh.userData.boundingBoxRelative = new THREE.Vector3();
            mesh.userData.qBBR = new THREE.Vector3();
            mesh.userData.fractionalBoundingBoxRelative = new THREE.Vector3();
        }
        this.pointLocal1.set(x, y, z).applyMatrix4( mesh.matrix );

        mesh.userData.boundingBoxRelative.copy(this.pointLocal1).sub(mesh.userData.boundingBoxMin).divide(mesh.userData.boundingBoxSize).multiplyScalar(dim);
        mesh.userData.qBBR.set(Math.floor(mesh.userData.boundingBoxRelative.x),
                               Math.floor(mesh.userData.boundingBoxRelative.y),
                               Math.floor(mesh.userData.boundingBoxRelative.z));

        mesh.userData.fractionalBoundingBoxRelative.copy(mesh.userData.boundingBoxRelative).sub(mesh.userData.qBBR);
        let x00 = this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x  , mesh.userData.qBBR.y  , mesh.userData.qBBR.z  ) * (1.0 - mesh.userData.fractionalBoundingBoxRelative.x) +
                  this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x+1, mesh.userData.qBBR.y  , mesh.userData.qBBR.z  ) *        mesh.userData.fractionalBoundingBoxRelative.x;
        let x01 = this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x  , mesh.userData.qBBR.y  , mesh.userData.qBBR.z+1) * (1.0 - mesh.userData.fractionalBoundingBoxRelative.x) +
                  this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x+1, mesh.userData.qBBR.y  , mesh.userData.qBBR.z+1) *        mesh.userData.fractionalBoundingBoxRelative.x;
        let x10 = this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x  , mesh.userData.qBBR.y+1, mesh.userData.qBBR.z  ) * (1.0 - mesh.userData.fractionalBoundingBoxRelative.x) +
                  this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x+1, mesh.userData.qBBR.y+1, mesh.userData.qBBR.z  ) *        mesh.userData.fractionalBoundingBoxRelative.x;
        let x11 = this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x  , mesh.userData.qBBR.y+1, mesh.userData.qBBR.z+1) * (1.0 - mesh.userData.fractionalBoundingBoxRelative.x) +
                  this.sampleCacheAtIndices(mesh, mesh.userData.qBBR.x+1, mesh.userData.qBBR.y+1, mesh.userData.qBBR.z+1) *        mesh.userData.fractionalBoundingBoxRelative.x;
        let y00 = x00 * (1.0 - mesh.userData.fractionalBoundingBoxRelative.y) + x10 * mesh.userData.fractionalBoundingBoxRelative.y;
        let y01 = x01 * (1.0 - mesh.userData.fractionalBoundingBoxRelative.y) + x11 * mesh.userData.fractionalBoundingBoxRelative.y;
        let z00 = y00 * (1.0 - mesh.userData.fractionalBoundingBoxRelative.z) + y01 * mesh.userData.fractionalBoundingBoxRelative.z;

        return z00;
    }

    /**
     * Analytical signed distance for an axis-aligned box in local space.
     * Much faster than BVH queries for finger geometries.
     */
    sampleBoxSDF(halfExtents, x, y, z) {
        // SDF of a box centered at origin with given half-extents
        const dx = Math.abs(x) - halfExtents.x;
        const dy = Math.abs(y) - halfExtents.y;
        const dz = Math.abs(z) - halfExtents.z;
        const outsideDist = Math.sqrt(
            Math.max(dx, 0) ** 2 + Math.max(dy, 0) ** 2 + Math.max(dz, 0) ** 2
        );
        const insideDist = Math.min(Math.max(dx, dy, dz), 0);
        return outsideDist + insideDist;
    }

    /**
     * Sample SDF for a finger mesh at a world-space point.
     * Uses analytical box SDF in the finger's local frame.
     */
    sampleFingerSDF(fingerIndex, x, y, z) {
        const finger = this.gripper.fingers[fingerIndex];
        // Use cached inverse matrix (set before marching cubes loop)
        this.pointLocal2.set(x, y, z).applyMatrix4(this._fingerInverseMatrices[fingerIndex]);
        return this.sampleBoxSDF(finger.halfExtents, this.pointLocal2.x, this.pointLocal2.y, this.pointLocal2.z);
    }

    // ====================================================================
    // Per-finger contact manifold computation
    // ====================================================================

    computeFingerContact(fingerIndex) {
        const fc = this.fingerContacts[fingerIndex];
        const finger = this.gripper.fingers[fingerIndex];

        // Compute bounding box overlap between finger and object
        fc.overlap.setFromObject(finger.mesh);
        fc.overlap2.setFromObject(this.mesh);

        if (!fc.overlap.intersectsBox(fc.overlap2)) {
            fc.manifoldMesh.visible = false;
            return null;
        }

        fc.manifoldMesh.visible = true;
        fc.overlap.intersect(fc.overlap2);

        // Set up inverse matrices for SDF sampling
        this.mesh.matrix.copy(this.mesh.matrixWorld).invert();
        this._fingerInverseMatrices[fingerIndex].copy(finger.mesh.matrixWorld).invert();

        // Build the implicit function: SDF_finger - SDF_object
        const calcFn = (x, y, z, result) => {
            const dFinger = this.sampleFingerSDF(fingerIndex, x, y, z);
            const dObject = this.sampleSignedDistanceWithCache(this.mesh, x, y, z);
            result.set(dFinger, dObject);
        };

        const clipped = this.updateMarchingCubes(
            calcFn,
            this.contactParams.resolution,
            fc.overlap.min,
            fc.overlap.max
        );

        // Update manifold visualization
        fc.manifoldMesh.geometry.dispose();
        fc.manifoldMesh.geometry = new THREE.BufferGeometry();
        fc.manifoldMesh.geometry.setIndex(clipped.indices);
        fc.manifoldMesh.geometry.setAttribute('position',
            new THREE.BufferAttribute(new Float32Array(clipped.vertices), 3));
        fc.manifoldMesh.geometry.setAttribute('penetrationDepth',
            new THREE.BufferAttribute(new Float32Array(clipped.penetrationDepth), 1));

        return clipped;
    }

    // ====================================================================
    // Arrow visualization for contact forces
    // ====================================================================

    drawForceArrow(centroid, forceVec, color = 0xff2200) {
        const mag = forceVec.length();
        if (mag < 1e-6) return;
        const dir = forceVec.clone().normalize();
        const arrowLen = Math.sqrt(mag) * 0.01 + 0.05;
        const headLen = arrowLen * 0.3;
        const headWidth = headLen * 0.5;
        const arrow = new THREE.ArrowHelper(dir, centroid, arrowLen, color, headLen, headWidth);
        this.arrowGroup.add(arrow);
    }

    // ====================================================================
    // Main update loop
    // ====================================================================

    update(timeMS) {
        this.timeMS = timeMS;

        if (!this.physicsWorld.ready) {
            this.world.renderer.render(this.world.scene, this.world.camera);
            return;
        }

        // --- Physics substep ---
        const dt = this.physicsWorld.params.dt;
        const elapsed = this.lastTimeMS > 0 ? Math.min((timeMS - this.lastTimeMS) / 1000, 0.05) : dt;
        this.lastTimeMS = timeMS;
        this.accumulator += elapsed;

        // Update gripper velocity from drag
        this.physicsWorld.updateGripperVelocity(this.gripper.baseMesh.position, elapsed);

        while (this.accumulator >= dt) {
            // 1. Compute hydroelastic contact manifolds and apply forces (only in hydroelastic mode)
            if (this.physicsWorld.useHydroelastic) {
                this.computeAndApplyHydroelasticForces();
            }

            // 2. Step PhysX
            this.physicsWorld.step(dt);

            // 3. Sync poses from PhysX → three.js
            this.physicsWorld.syncToThreeJS();

            this.accumulator -= dt;
        }

        // --- Update visual contact manifolds (at render rate, only in hydroelastic mode) ---
        if (this.physicsWorld.useHydroelastic) {
            this.updateVisualManifolds();
        } else {
            // Clear arrows when in PhysX mode
            for (let i = this.arrowGroup.children.length - 1; i >= 0; i--) {
                this.arrowGroup.remove(this.arrowGroup.children[i]);
            }
        }

        this.world.controls.update();
        this.world.renderer.render(this.world.scene, this.world.camera);
        this.world.stats.update();
    }

    computeAndApplyHydroelasticForces() {
        if (!this.gripper) return;

        for (let i = 0; i < this.gripper.fingers.length; i++) {
            const finger = this.gripper.fingers[i];
            const fc = this.fingerContacts[i];

            // Quick bounding box check
            fc.overlap.setFromObject(finger.mesh);
            fc.overlap2.setFromObject(this.mesh);

            if (!fc.overlap.intersectsBox(fc.overlap2)) continue;

            fc.overlap.intersect(fc.overlap2);

            // Set up matrices for SDF sampling
            this.mesh.matrix.copy(this.mesh.matrixWorld).invert();
            this._fingerInverseMatrices[i].copy(finger.mesh.matrixWorld).invert();

            const calcFn = (x, y, z, result) => {
                const dFinger = this.sampleFingerSDF(i, x, y, z);
                const dObject = this.sampleSignedDistanceWithCache(this.mesh, x, y, z);
                result.set(dFinger, dObject);
            };

            // Use lower resolution for physics (faster)
            const physRes = Math.max(8, Math.floor(this.contactParams.resolution * 0.6));
            const clipped = this.updateMarchingCubes(calcFn, physRes, fc.overlap.min, fc.overlap.max);

            const numTris = clipped.penetrationDepth.length / 3;
            if (numTris > 0) {
                // Apply Drake-style hydroelastic forces
                this.physicsWorld.applyHydroelasticForces(
                    clipped.vertices,
                    clipped.penetrationDepth,
                    numTris,
                    { pxActor: finger.link },     // finger articulation link
                    this.objectBody               // dynamic object
                );
            }
        }
    }

    updateVisualManifolds() {
        // Clear arrows
        for (let i = this.arrowGroup.children.length - 1; i >= 0; i--) {
            this.arrowGroup.remove(this.arrowGroup.children[i]);
        }

        for (let i = 0; i < this.gripper.fingers.length; i++) {
            const clipped = this.computeFingerContact(i);
            if (!clipped) continue;

            const numTris = clipped.penetrationDepth.length / 3;
            if (numTris === 0) continue;

            // Draw aggregate force arrows per connected component
            this.drawContactForceArrows(clipped.vertices, clipped.penetrationDepth, numTris);
        }
    }

    findConnectedComponents(vertices, numTris) {
        let edgeToTris = new Map();

        function vertKey(vertices, vIdx) {
            let base = vIdx * 3;
            return `${vertices[base].toFixed(5)}_${vertices[base + 1].toFixed(5)}_${vertices[base + 2].toFixed(5)}`;
        }

        function edgeKey(k1, k2) {
            return k1 < k2 ? `${k1}|${k2}` : `${k2}|${k1}`;
        }

        for (let t = 0; t < numTris; t++) {
            let v0 = vertKey(vertices, t * 3);
            let v1 = vertKey(vertices, t * 3 + 1);
            let v2 = vertKey(vertices, t * 3 + 2);

            let edges = [edgeKey(v0, v1), edgeKey(v1, v2), edgeKey(v0, v2)];
            for (let e of edges) {
                if (!edgeToTris.has(e)) edgeToTris.set(e, []);
                edgeToTris.get(e).push(t);
            }
        }

        let adj = Array.from({ length: numTris }, () => []);
        for (let [, tris] of edgeToTris) {
            for (let i = 0; i < tris.length; i++) {
                for (let j = i + 1; j < tris.length; j++) {
                    adj[tris[i]].push(tris[j]);
                    adj[tris[j]].push(tris[i]);
                }
            }
        }

        let component = new Int32Array(numTris).fill(-1);
        let numComponents = 0;

        for (let t = 0; t < numTris; t++) {
            if (component[t] >= 0) continue;
            let stack = [t];
            component[t] = numComponents;
            while (stack.length > 0) {
                let cur = stack.pop();
                for (let neighbor of adj[cur]) {
                    if (component[neighbor] < 0) {
                        component[neighbor] = numComponents;
                        stack.push(neighbor);
                    }
                }
            }
            numComponents++;
        }

        return { component, numComponents };
    }

    drawContactForceArrows(vertices, penetrationDepth, numTris) {
        let { component, numComponents } = this.findConnectedComponents(vertices, numTris);

        let forces = Array.from({ length: numComponents }, () => new THREE.Vector3());
        let centroids = Array.from({ length: numComponents }, () => new THREE.Vector3());
        let totalAreas = new Float64Array(numComponents);

        let v0 = new THREE.Vector3(), v1 = new THREE.Vector3(), v2 = new THREE.Vector3();
        let e1 = new THREE.Vector3(), e2 = new THREE.Vector3();
        let normal = new THREE.Vector3();

        for (let t = 0; t < numTris; t++) {
            let base = t * 9;
            let dBase = t * 3;
            let comp = component[t];

            v0.set(vertices[base], vertices[base + 1], vertices[base + 2]);
            v1.set(vertices[base + 3], vertices[base + 4], vertices[base + 5]);
            v2.set(vertices[base + 6], vertices[base + 7], vertices[base + 8]);

            e1.subVectors(v1, v0);
            e2.subVectors(v2, v0);
            normal.crossVectors(e1, e2);

            let area = normal.length() * 0.5;
            if (area < 1e-10) continue;

            normal.normalize();

            let avgDepth = (penetrationDepth[dBase] + penetrationDepth[dBase + 1] + penetrationDepth[dBase + 2]) / 3;
            let pressure = avgDepth * area;

            forces[comp].addScaledVector(normal, pressure);

            let cx = (v0.x + v1.x + v2.x) / 3;
            let cy = (v0.y + v1.y + v2.y) / 3;
            let cz = (v0.z + v1.z + v2.z) / 3;
            centroids[comp].x += cx * area;
            centroids[comp].y += cy * area;
            centroids[comp].z += cz * area;
            totalAreas[comp] += area;
        }

        for (let c = 0; c < numComponents; c++) {
            if (totalAreas[c] < 1e-10) continue;
            centroids[c].divideScalar(totalAreas[c]);

            let forceMag = forces[c].length();
            if (forceMag < 1e-10) continue;

            let dir = forces[c].clone().normalize().negate();
            let arrowLength = Math.sqrt(forceMag) * 3 + 0.15;
            let headLength = arrowLength * 0.3;
            let headWidth = headLength * 0.5;
            let arrow = new THREE.ArrowHelper(dir, centroids[c], arrowLength, 0xff2200, headLength, headWidth);
            this.arrowGroup.add(arrow);
        }
    }

    // Log Errors as <div>s over the main viewport
    fakeError(...args) {
        if (args.length > 0 && args[0]) { this.display(JSON.stringify(args[0])); }
        window.realConsoleError.apply(console, arguments);
    }

    display(text) {
        let errorNode = window.document.createElement("div");
        errorNode.innerHTML = text.fontcolor("red");
        window.document.getElementById("info").appendChild(errorNode);
    }
}

var main = new Main();
