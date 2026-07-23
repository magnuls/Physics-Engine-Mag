# Physics-Engine (3DEngineCpp)

A real-time **3D rigid-body physics engine** written from scratch in **C++20**,
integrated into an OpenGL/SDL2 forward renderer as an interactive sandbox. The
physics subsystem, collision detection, a warm started sequential-impulse
constraint solver, friction, continuous collision detection, and sleeping, is
the focus here; the rendering half is a complete forward renderer the physics
plugs into.

Drop objects, throw balls, and watch them collide, stack, tip, and settle in
real time.

> Built on a reorganized C++20 fork of the upstream `3DEngineCpp` renderer (see
> Credits). The physics engine, solver, and test suite are original.

## Highlights

- **~8,000 concurrently-simulated, colliding rigid bodies at 60 FPS**
  (13.6 ms/step, single-threaded, release `-O2`; 1,000 bodies ≈ 0.5 ms).
- **116 bytes per rigid body** — ~11 MB of state for 100k bodies.
- **93 GoogleTest cases** (20 suites), 100% passing, run in CI.

## Features

**Physics (`src/physics/`)**
- **Colliders:** sphere, AABB, **oriented box (OBB)**, and infinite plane.
- **Narrow phase:** `Physics::collision<A, B>()` — a `std::variant` dispatched
  set of routines specialized for every shape pair (Separating Axis Theorem for
  oriented boxes), returning an `IntersectData` contact manifold (hit flag,
  penetration depth, unit normal, contact point).
- **Broad phase:** `Physics::Broadphase::sweepAndPrune()` culls the O(N²) pair
  check to candidate AABB overlaps.
- **Solver:** a **sequential-impulse** constraint solver (8 velocity iterations,
  **warm-started** from a persistent contact cache) with **Coulomb friction**,
  restitution, and Baumgarte positional correction with stable stacks and objects
  that settle instead of jittering.
- **Rotational dynamics:** per shape inverse inertia tensors and quaternion
  orientation integration dropped boxes tip on their corners.
- **Continuous collision detection:** speculative contacts stop fast bodies
  (thrown balls) from tunnelling through thin geometry opt in per body.
- **Sleeping / islands:** resting bodies stop being integrated and solved
  (union-find islands sleep and wake together) to cut solver cost.
- **Colliders as components:** `Physics::{Sphere,AABB,OBB,Plane}Collider` are
  `EntityComponent`s driven by their entity's `Transform`.

**Rendering (inherited engine)**
- Entity–Component scene graph on a fixed-timestep core loop (`CoreEngine`).
- Forward renderer: point/spot/directional lights, variance shadow maps,
  gaussian blur, FXAA, normal + parallax/displacement mapping.
- Reference-counted shaders / meshes / textures / materials; ASSIMP model
  loading; SDL2 window + GL context.
- Templated math library (`Vector`, `Matrix4f`, `Quaternion`, `Transform`).

## Controls (the demo)

| Key | Action |
|-----|--------|
| `SPACE` | Spawn / throw a ball from the camera along its aim (uses CCD) |
| `R` | Reset the scene |
| `W A S D` | Move the camera |
| Mouse | Look around |
| `ESC` | Release / recapture the mouse |

The demo scene shows a friction ramp (a rolling ball beside a sliding one), a
stable box stack, and a ball-on-ball drop, all on a checkerboard floor under a
shadow-casting directional light.

## Requirements

- A **C++20** compiler (Apple Clang / Clang / GCC).
- **CMake ≥ 3.23**.
- **OpenGL**, **GLEW**, **SDL2**, **ASSIMP**.
- **GoogleTest** (for the test suite).

```shell
# Ubuntu / Debian
sudo apt-get install cmake libglew-dev libsdl2-dev libassimp-dev libgtest-dev

# macOS (Homebrew)
brew install cmake glew sdl2 assimp googletest
```

On Windows, `third_party/lib/` vendors SDL2 / GLEW / ASSIMP import libraries and
headers, and the `cmake/Find*.cmake` modules fall back to it.

## Project layout

```
CMakeLists.txt          # root build: engine static lib + demo + test runner
cmake/                  # FindGLEW / FindSDL2 / FindASSIMP modules
res/                    # runtime assets: models/ shaders/ textures/
scripts/                # Unix-*.sh / Windows-*.bat / run build helpers
src/
  core/                 # engine spine: scene graph, math, timing, transforms
  rendering/            # OpenGL forward renderer (window, shaders, meshes, lights)
  components/           # EntityComponents (camera controls, mesh + physics glue)
  physics/              # colliders, collision dispatch, broadphase, solver, engine
  main.cpp  3DEngine.h  # demo game + umbrella public header
third_party/            # stb_image, SIMD headers, Win32 import libs (vendored)
tests/                  # GoogleTest suites
```

## Build & run

```shell
# convenience script (Debug by default; pass Release or extra CMake args)
./scripts/Unix-Build.sh [Debug|Release] [extra CMake args]

# ...or manually
mkdir -p build && cd build
cmake ..
make -j4
```

CMake copies `res/` into the build directory automatically, so the demo finds
its assets. Run it:

```shell
./scripts/run          # or: cd build && ./3DEngineCpp
```

## Tests

```shell
cd build
ctest                  # or run the binary directly:
./unit_tests
```

**93 tests across 20 suites**, covering raw collision detection, contact
manifolds, broad phase, OBB, the impulse solver + friction, restitution,
angular dynamics, continuous collision detection, sleeping/islands, and the math
library.

> The build globs sources, so after adding a new `.cpp` re-run `cmake ..` before
> `make` so it is picked up. The test binary is named `unit_tests` (not
> `testing`, which collides with CTest's `Testing/` directory on case-insensitive
> filesystems).

## Credits

- Etay Meiri, for <http://ogldev.atspace.co.uk/> which inspired the base engine.
- [@mxaddict](https://github.com/mxaddict) for the original CMake build system.
- The upstream `3DEngineCpp` project and contributors (Apache-2.0).
