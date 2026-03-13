import * as THREE from '../node_modules/three/build/three.module.js';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { mergeVertices } from '../node_modules/three/examples/jsm/utils/BufferGeometryUtils.js';
import World from './World.js';

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
        // Configure Settings
        this.contactParams = {
            //loadMesh: this.loadMesh.bind(this),
            //showMesh: true,
            resolution: 20,
        };
        this.gui = new GUI();
        //this.gui.add(this.latticeParams, 'loadMesh' ).name( 'Load Mesh' );
        //this.gui.add(this.contactParams, 'showMesh').name( 'Show Mesh' ).onFinishChange(async (value) => {
        //    if(this.mesh){ this.mesh.visible = value; }});
        this.gui.add(this.contactParams, 'resolution', 3, 40, 1).name( 'Resolution' ).onFinishChange(async (value) => { this.updateImplicitMesh(); });

        this.sphereGeo   = new THREE.SphereGeometry  (1.0, 32, 32);
        this.cylinderGeo = new THREE.CylinderGeometry(1, 1, 1, 8);
        this.geometry    = new THREE.BoxGeometry();
        /** @type {MeshBVH} */
        this.bvh1 = this.geometry.computeBoundsTree();
        this.material = new THREE.MeshPhysicalMaterial({ color: 0xffffff,  wireframe: true, side: THREE.FrontSide }); //transparent: true, opacity: 0.25, side: THREE.FrontSide,

        this.tmpInverseMatrix1 = new THREE.Matrix4();
        this.tmpInverseMatrix2 = new THREE.Matrix4();
        this.pointLocal1 = new THREE.Vector3();
        this.pointLocal2 = new THREE.Vector3();
        this.color = new THREE.Color();
        this.closest1 = {};

        // Construct the render world
        this.world = new World(this);

        this.control = new TransformControls( this.world.camera, this.world.renderer.domElement );
        this.control.addEventListener( 'dragging-changed', ( event ) => { this.world.controls.enabled = ! event.value; } );

        this.mesh = new THREE.Mesh( this.geometry , this.material  );
        this.world.scene.add( this.mesh );
        this.mesh.position.set(0.0, 2.1, 0.4);
        this.control.attach( this.mesh );
		this.world.scene.add( this.control );

        this.isDeployed = document.pathname !== '/';
        new THREE.BufferGeometryLoader().setPath( this.isDeployed ? './assets/' : '../assets/' ).load( 'suzanne_buffergeometry.json', ( geometry ) => {
            let mergedGeometry = mergeVertices(geometry, 1e-6);

            this.bvh2 = mergedGeometry.computeBoundsTree();
            this.mesh2 = new THREE.Mesh( mergedGeometry , this.material  );
            this.world.scene.add( this.mesh2 );
            this.mesh2.position.set(0, 1.0, 0.0);
    
            this.overlap  = new THREE.Box3();
            this.overlap2 = new THREE.Box3();
            this.overlap .setFromObject(this.mesh );
            this.overlap2.setFromObject(this.mesh2);
            this.overlap = this.overlap.intersect(this.overlap2);

            this.tempRay = new THREE.Ray();
            this.tempRay.direction.set(1, 1, 1).normalize();
            
            let implicitMaterial = new THREE.ShaderMaterial( {
                side: THREE.DoubleSide,
                //wireframe: true,
                vertexShader: `
                    attribute float penetrationDepth; varying float vPenetrationDepth;
                    void main() {
                        vPenetrationDepth = penetrationDepth;
                        gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
                    }`,
                fragmentShader: `
                	varying float vPenetrationDepth;
                    // fifth-order polynomial approximation of Turbo based on: https://observablehq.com/@mbostock/turbo
                    vec4 turbo(float x) {
                        float r = 0.1357 + x * ( 4.5974 - x * (42.3277 - x * (130.5887 - x * (150.5666 - x * 58.1375))));
                        float g = 0.0914 + x * ( 2.1856 + x * ( 4.8052 - x * ( 14.0195 - x * (  4.2109 + x *  2.7747))));
                        float b = 0.1067 + x * (12.5925 - x * (60.1097 - x * (109.0745 - x * ( 88.5066 - x * 26.8183))));
                        return vec4(r, g, b, 1.0);
                    }
                    void main() {
                        gl_FragColor = turbo( clamp(vPenetrationDepth * 3.0, 0.0, 1.0) );
                    }`
            } );

            this.implicitMesh = new THREE.Mesh(new THREE.BufferGeometry(), implicitMaterial );
            this.world.scene.add(this.implicitMesh);
            this.arrowGroup = new THREE.Group();
            this.world.scene.add(this.arrowGroup);
            this.updateImplicitMesh();
        });
    }

    updateMarchingCubes(calculateImplicitFunction, resolution = 15, 
        axisMin = new THREE.Vector3(-10, -10, -10), axisMax = new THREE.Vector3(10, 10, 10)) {
        // custom global variables
        this.points = [];
        this.values = [];
        this.depthValues = [];

        /** @type {THREE.Vector3} */
        let axisRange = axisMax.clone().sub(axisMin);
        let distanceResult = new THREE.Vector2();
        
        // Generate a list of 3D points and values at those points
        for (let k = 0; k < resolution; k++)
        for (let j = 0; j < resolution; j++)
        for (let i = 0; i < resolution; i++)
        {
            // actual values
            let x = axisMin.x + axisRange.x * i / (resolution - 1);
            let y = axisMin.y + axisRange.y * j / (resolution - 1);
            let z = axisMin.z + axisRange.z * k / (resolution - 1);
            this.points.push( new THREE.Vector3(x,y,z) );
            calculateImplicitFunction(x, y, z, distanceResult);
            this.values.push( (distanceResult.x - distanceResult.y));// / (Math.abs(distanceResult.x) + Math.abs(distanceResult.y)) );
            this.depthValues.push( (Math.max(distanceResult.x, distanceResult.y)) );
        }
        
        // Marching Cubes Algorithm
        
        let size2 = resolution * resolution;

        // Vertices may occur along edges of cube, when the values at the edge's endpoints
        //   straddle the isolevel value.
        // Actual position along edge weighted according to function values.
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
            // index of base point, and also adjacent points on cube
            let p    = x + resolution * y + size2 * z,
                px   = p   + 1,
                py   = p   + resolution,
                pxy  = py  + 1,
                pz   = p   + size2,
                pxz  = px  + size2,
                pyz  = py  + size2,
                pxyz = pxy + size2;
            
            // store scalar values corresponding to vertices
            let value0 = this.values[ p    ],
                value1 = this.values[ px   ],
                value2 = this.values[ py   ],
                value3 = this.values[ pxy  ],
                value4 = this.values[ pz   ],
                value5 = this.values[ pxz  ],
                value6 = this.values[ pyz  ],
                value7 = this.values[ pxyz ];
            
            // place a "1" in bit positions corresponding to vertices whose
            //   isovalue is less than given constant.
            
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
            
            // bits = 12 bit number, indicates which edges are crossed by the isosurface
            let bits = edgeTable[ cubeindex ];
            
            // if none are crossed, proceed to next iteration
            if ( bits === 0 ) continue;
            
            // check which edges are crossed, and estimate the point location
            //    using a weighted average of scalar values at edge endpoints.
            // store the vertex in an array for use later.
            let mu = 0.5; 
            
            // bottom of the cube
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
            // top of the cube
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
            // vertical lines of the cube
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
            
            // construct triangles -- get correct vertices from triTable.
            let i = 0;
            cubeindex <<= 4;  // multiply by 16... 
            // "Re-purpose cubeindex into an offset into triTable." 
            //  since each row really isn't a row.
            
            // the while loop should run at most 5 times,
            //   since the 16th entry in each row is a -1.
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

        // Clip triangles at penetrationDepth = 0 boundary
        let clipped = this.clipTriangles(vertices, penetrationDepth);

        this.implicitMesh.geometry.setIndex( clipped.indices );
        this.implicitMesh.geometry.setAttribute( 'position', new THREE.BufferAttribute( new Float32Array(clipped.vertices), 3 ) );
        this.implicitMesh.geometry.setAttribute( 'penetrationDepth', new THREE.BufferAttribute( new Float32Array(clipped.penetrationDepth), 1 ) );
        this.implicitMesh.geometry.needsUpdate = true;
        this.implicitMesh.geometry.buffersNeedUpdate = true;

        // Store clipped data for contact force computation
        this.clippedVertices = clipped.vertices;
        this.clippedDepths = clipped.penetrationDepth;
        this.clippedNumTris = clipped.penetrationDepth.length / 3;
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

    computeAndDrawContactForces(vertices, penetrationDepth, numTris) {
        // Clear previous arrows
        for (let i = this.arrowGroup.children.length - 1; i >= 0; i--) {
            this.arrowGroup.remove(this.arrowGroup.children[i]);
        }

        if (numTris === 0) return;

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

    /** @param {THREE.Mesh} mesh */
    sampleSignedDistance(mesh, x, y, z) {
        this.tempRay.origin.set(x, y, z);
        let hit1 = mesh .geometry.boundsTree.raycastFirst(this.tempRay, THREE.DoubleSide);
        mesh.geometry.boundsTree.closestPointToPoint(this.tempRay.origin, this.closest1);
        //this.derp = getTriangleHitPointInfo(this.closest1.point, this.mesh.geometry, this.closest1.faceIndex, this.derp);
        return this.closest1.distance * ((hit1 && hit1.face.normal.dot( this.tempRay.direction ) > 0.0) ? 1.0 : -1.0);
    }

    sampleCacheAtIndices(mesh, xIndex, yIndex, zIndex) {
        const dim = 72;
        // Calculate the index of the cache
        let index = xIndex + yIndex * dim + zIndex * dim * dim;
        index = Math.min(Math.max(index, 0), dim * dim * dim - 1);

        // Check if the cache is valid
        if(mesh.userData.distanceCache[index] === 0.0) {
            // Transform the quantized position back into bounding box relative
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
            
            // Pre define the bounding box in local space
            mesh.userData.boundingBox     = mesh.geometry.boundingBox;
            mesh.userData.boundingBoxSize = mesh.userData.boundingBox.getSize(new THREE.Vector3());
            mesh.userData.boundingBoxMin  = mesh.userData.boundingBox.min.clone();
            mesh.userData.boundingBoxMin .subScalar(0.01);
            mesh.userData.boundingBoxSize.addScalar(0.02);

            // Preallocate temp variables
            mesh.userData.boundingBoxRelative = new THREE.Vector3();
            mesh.userData.qBBR = new THREE.Vector3();
            mesh.userData.fractionalBoundingBoxRelative = new THREE.Vector3();
        }
        // Transform the point into local space
        this.pointLocal1.set(x, y, z).applyMatrix4( mesh.matrix );

        // Transform into bounding box relative coordinates
        mesh.userData.boundingBoxRelative.copy(this.pointLocal1).sub(mesh.userData.boundingBoxMin).divide(mesh.userData.boundingBoxSize).multiplyScalar(dim);
        // Quantized Bounding Box Relative
        mesh.userData.qBBR.set(Math.floor(mesh.userData.boundingBoxRelative.x),
                               Math.floor(mesh.userData.boundingBoxRelative.y),
                               Math.floor(mesh.userData.boundingBoxRelative.z));

        // Use the nearest cache value
        //return this.sampleCacheAtIndices(mesh, qBBR.x, qBBR.y, qBBR.z, boundingBoxSize, boundingBoxMin);

        // Use trilinear interpolation to get an interpolated value from the cache
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

    calculateImplicitFunction(x, y, z, result) {
        result.set(this.sampleSignedDistanceWithCache(this.mesh , x, y, z),
                   this.sampleSignedDistanceWithCache(this.mesh2, x, y, z));
    }

    updateImplicitMesh() {
        if(this.implicitMesh){
            let boundingBoxTiming = performance.now();
            this.overlap .setFromObject(this.mesh );
            this.overlap2.setFromObject(this.mesh2);
            boundingBoxTiming = performance.now() - boundingBoxTiming;
            let geometryTiming = performance.now();
            if(this.overlap.intersectsBox(this.overlap2)) {
                this.implicitMesh.visible = true;
                this.overlap = this.overlap.intersect(this.overlap2);

                this.mesh .matrix.copy( this.mesh .matrixWorld ).invert();
                this.mesh2.matrix.copy( this.mesh2.matrixWorld ).invert();

                this.updateMarchingCubes(this.calculateImplicitFunction.bind(this), this.contactParams.resolution, this.overlap.min, this.overlap.max);
                this.computeAndDrawContactForces(this.clippedVertices, this.clippedDepths, this.clippedNumTris);
            }else{
                this.implicitMesh.visible = false;
                // Clear arrows when no overlap
                for (let i = this.arrowGroup.children.length - 1; i >= 0; i--) {
                    this.arrowGroup.remove(this.arrowGroup.children[i]);
                }
            }
            geometryTiming = performance.now() - geometryTiming;
            //console.log("Time to compute overlap box: " + boundingBoxTiming + "ms", "Time to compute geometry: " + geometryTiming + "ms");
        }
    }

    /** Update the simulation */
    update(timeMS) {
        this.timeMS = timeMS;
        if(!this.world.controls.enabled){
            this.updateImplicitMesh();
        }
        this.world.controls.update();
        this.world.renderer.render(this.world.scene, this.world.camera);
        this.world.stats.update();
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
