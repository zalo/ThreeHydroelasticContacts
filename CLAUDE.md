# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Browser-based proof of concept for computing Hydroelastic-Style Contact Manifolds between meshes/SDFs using three.js, integrated with PhysX WASM for rigid body dynamics. Implements Drake's compliant hydroelastic contact model (Hunt-Crossley dissipation + regularized Coulomb friction) for robotics gripper simulation. Live demo deployed to GitHub Pages.

## Build & Run

```bash
npm install
npm run build  # esbuild: bundles src/main.js -> build/main.js (ESM, minified, sourcemapped)
```

**No build required for local dev** — open `index.html` directly in Chrome/Edge/Opera (requires Import Maps support). The HTML points to `./src/main.js` for development; the CI pipeline rewrites it to `./build/main.js` for production.

PhysX WASM files (`dist/physx-js-webidl.mjs` + `.wasm`) are loaded at runtime via dynamic import and kept external from the esbuild bundle.

## Architecture

Three source files, no test suite:

- **`src/main.js`** — `Main` class: core application logic
  - Creates a dynamic box as the gripped object with BVH-based SDF
  - Builds a parallel-jaw gripper via `PhysicsWorld.buildGripper()` (PhysX articulation)
  - Per-finger contact manifold computation using marching cubes on `SDF_finger - SDF_object`
  - Analytical box SDF for finger geometries (fast, no BVH needed)
  - BVH-cached SDF for the gripped object (72x72x72 grid, trilinear interpolation)
  - Physics substep loop: compute contacts → apply forces → step PhysX → sync poses
  - Visual manifold rendering with Turbo colormap + force arrow visualization
  - GUI controls for gripper open/close, MC resolution, hydroelastic parameters

- **`src/PhysicsWorld.js`** — `PhysicsWorld` class: PhysX WASM integration
  - Initializes PhysX 5.6.1 (foundation, physics, scene with gravity)
  - Collision filtering: fingers (group 2) and object (group 4) only collide with ground (group 1); finger-object contacts handled by hydroelastic forces
  - `buildGripper()`: PxArticulationReducedCoordinate with fixed base + 2 revolute finger joints with position drives
  - `applyHydroelasticForces()`: Drake's compliant contact model per-triangle:
    - Normal force: `F_n = k * depth * (1 + d * max(v_n, 0)) * area` (Hunt-Crossley)
    - Friction: `F_t = -μ * |F_n| * v_slip / max(|v_slip|, v_stiction)` (regularized Coulomb)
    - Applied via `PxRigidBodyExt.addForceAtPos()` at triangle centroids, equal and opposite on both bodies
  - `syncToThreeJS()`: copies PhysX poses to three.js meshes each frame

- **`src/World.js`** — `World` class: three.js scene boilerplate (camera, lights, renderer, OrbitControls, Stats)

- **`index.html`** — Entry point with importmap for three.js, inline styles

## Key Algorithms

The contact manifold is the zero-isosurface of `SDF1(p) - SDF2(p)`, restricted to the overlapping bounding box of both meshes. Penetration depth at each vertex is `max(SDF1, SDF2)`. Triangles are clipped at the `depth=0` boundary. The Marching Cubes implementation uses edge/tri tables imported from three.js's `MarchingCubes` example.

Drake's compliant hydroelastic contact model computes physically-meaningful forces:
- **Hunt-Crossley dissipation** damps normal approach velocity, preventing oscillation
- **Regularized Coulomb friction** provides smooth tangential forces that enable stable grasping
- Forces are integrated over the contact surface (per-triangle quadrature), not applied at discrete points

## Dependencies

- `three` (v0.166) — rendering, geometry, controls
- `three-mesh-bvh` (v0.9) — BVH acceleration for SDF queries (monkey-patched onto `THREE.Mesh`/`THREE.BufferGeometry` prototypes)
- `physx-js-webidl` — PhysX 5.6.1 WASM bindings (in `dist/`, kept external from bundle)
- `esbuild` — bundler (build only)

## Deployment

GitHub Actions (`.github/workflows/main.yml`) auto-deploys to GitHub Pages on push to `main`. The workflow runs `npm install`, rewrites `index.html` to point at `build/`, runs esbuild, then deploys.
