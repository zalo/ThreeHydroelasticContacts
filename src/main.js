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
            showMesh: true,
            resolution: 10,
        };
        this.gui = new GUI();
        //this.gui.add(this.latticeParams, 'loadMesh' ).name( 'Load Mesh' );
        this.gui.add(this.contactParams, 'showMesh').name( 'Show Mesh' ).onFinishChange(async (value) => {
            if(this.mesh){ this.mesh.visible = value; }});
        this.gui.add(this.contactParams, 'resolution', 3, 50, 1).name( 'Resolution' ).onFinishChange(async (value) => {
            if(this.mesh){  }});

        this.sphereGeo   = new THREE.SphereGeometry  (1.0, 32, 32);
        this.cylinderGeo = new THREE.CylinderGeometry(1, 1, 1, 8);
        this.geometry    = new THREE.BoxGeometry();
        /** @type {MeshBVH} */
        this.bvh1 = this.geometry.computeBoundsTree();
        this.material = new THREE.MeshPhysicalMaterial({ color: 0xffffff,  wireframe: true, side: THREE.FrontSide }); //transparent: true, opacity: 0.25, side: THREE.FrontSide,

        this.tmpInverseMatrix = new THREE.Matrix4();
        this.pointLocal1 = new THREE.Vector3();
        this.pointLocal2 = new THREE.Vector3();
        this.color = new THREE.Color();

        // Construct the render world
        this.world = new World(this);

        this.control = new TransformControls( this.world.camera, this.world.renderer.domElement );
        this.control.addEventListener( 'dragging-changed', ( event ) => { this.world.controls.enabled = ! event.value; } );

        this.mesh = new THREE.Mesh( this.geometry , this.material  );
        this.world.scene.add( this.mesh );
        this.mesh.position.set(0.0, 2.2, 0.5);
        this.control.attach( this.mesh );
		this.world.scene.add( this.control );

        new THREE.BufferGeometryLoader().setPath( '../assets/' ).load( 'suzanne_buffergeometry.json', ( geometry ) => {
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
                    // fifth-order polynomial approximation of Turbo based on: https://observablehq.com/@mbostock/turbo
                    vec4 turbo(float x) {
                        float r = 0.1357 + x * ( 4.5974 - x * (42.3277 - x * (130.5887 - x * (150.5666 - x * 58.1375))));
                        float g = 0.0914 + x * ( 2.1856 + x * ( 4.8052 - x * ( 14.0195 - x * (  4.2109 + x *  2.7747))));
                        float b = 0.1067 + x * (12.5925 - x * (60.1097 - x * (109.0745 - x * ( 88.5066 - x * 26.8183))));
                        return vec4(r, g, b, 1.0);
                    }
                    void main() { gl_FragColor = turbo( clamp(vPenetrationDepth, 0.0, 1.0) ); }`
            } );

            this.implicitMesh = new THREE.Mesh(new THREE.BufferGeometry(), implicitMaterial );
            this.world.scene.add(this.implicitMesh);
        });
    }

    updateMesh(calculateImplicitFunction, resolution = 15, 
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
            this.values.push( (distanceResult.x - distanceResult.y) / (Math.abs(distanceResult.x) + Math.abs(distanceResult.y)) );
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

        this.implicitMesh.geometry.setIndex( indices );
        this.implicitMesh.geometry.setAttribute( 'position', new THREE.BufferAttribute( new Float32Array(vertices), 3 ) );
        //this.implicitMesh.geometry.setAttribute( 'color', new THREE.BufferAttribute( new Float32Array(penetrationDepth), 3 ) );
        this.implicitMesh.geometry.setAttribute( 'penetrationDepth', new THREE.BufferAttribute( new Float32Array(penetrationDepth), 1 ) );
        this.implicitMesh.geometry.needsUpdate = true;
        this.implicitMesh.geometry.buffersNeedUpdate = true;
    }

    calculateImplicitFunction(x, y, z, result) {
        this.pointLocal1.set(x, y, z);
        this.pointLocal2.set(x, y, z);

        this.closest1 = this.bvh1.closestPointToPoint(this.pointLocal1.applyMatrix4( this.tmpInverseMatrix.copy( this.mesh.matrixWorld ).invert() ), this.closest1);
        this.closest2 = this.bvh2.closestPointToPoint(this.pointLocal2.applyMatrix4( this.tmpInverseMatrix.copy( this.mesh2.matrixWorld ).invert() ), this.closest2);
        let distance1 = this.closest1.distance;
        let distance2 = this.closest2.distance;

        this.derp   = getTriangleHitPointInfo(this.closest1.point, this.mesh.geometry, this.closest1.faceIndex, this.derp);
        let inside1 = this.derp.face.normal.dot(this.pointLocal1.clone().sub(this.closest1.point)) > 0;
        this.derp   = getTriangleHitPointInfo(this.closest2.point, this.mesh2.geometry, this.closest2.faceIndex, this.derp);
        let inside2 = this.derp.face.normal.dot(this.pointLocal2.clone().sub(this.closest2.point)) > 0;
        result.set(distance1 * inside1 ? -1.0 : 1.0, 
                   distance2 * inside2 ? -1.0 : 1.0);
    }

    /** Update the simulation */
    update(timeMS) {
        this.timeMS = timeMS;
        if(this.implicitMesh){
            this.overlap .setFromObject(this.mesh );
            this.overlap2.setFromObject(this.mesh2);
            if(this.overlap.intersectsBox(this.overlap2)) {
                this.implicitMesh.visible = true;
                this.overlap = this.overlap.intersect(this.overlap2);
                this.updateMesh(this.calculateImplicitFunction.bind(this), 10, this.overlap.min, this.overlap.max);
            }else{
                this.implicitMesh.visible = false;
            }
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
