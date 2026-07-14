#include <iostream>

#include "3DEngine.h"
#include "components/ballSpawnerComponent.h"
#include "components/freeLook.h"
#include "components/freeMove.h"
#include "components/physicsEngineComponent.h"
#include "components/physicsObjectComponent.h"
#include "physics/physicsObject.h"

// Interactive physics sandbox demo. Three set-pieces: a tilted friction ramp
// (rolling vs sliding ball), a 5-box stack that stays standing, and a
// ball-on-ball drop. SPACE spawns a ball along the camera aim, R resets.
//
// Sizing: sphere.obj is unit-radius and cube.obj spans [-1,1] per axis, so at
// Transform scale s the physics radius / half-extent matches the mesh.
//
// Heads up: a Material's data is evicted from the static resource map once its
// last reference drops (material.cpp), so Material("x") asserts if "x" is gone.
// Every Init MeshRenderer keeps its own material alive; the spawner holds
// Mesh+Material by value. Materials: checker=floor, metal=ramp, bricks2=stack,
// bricks=all balls.
class TestGame : public Game {
   public:
    TestGame() {}

    virtual void Init(const Window& window);

   protected:
   private:
    TestGame(const TestGame& other) = delete;
    void operator=(const TestGame& other) = delete;
};

void TestGame::Init(const Window& window) {
    // Materials (distinct per body kind; each anchored by a MeshRenderer below).
    Material bricks2("bricks2", Texture("bricks2.jpg"), 0.0f, 0,
                     Texture("bricks2_normal.png"), Texture("bricks2_disp.jpg"),
                     0.04f, -1.0f);
    Material checker("checker", Texture("checker.png"), 0.1f, 2);
    Material metal("metal", Texture("metal.jpg"), 0.6f, 8);
    Material bricks("bricks", Texture("bricks.jpg"), 0.2f, 4);

    // Camera framed on all three set-pieces (ramp at -x, ball-on-ball centre,
    // stack at +x). Captured so the ball spawner can read its position/aim.
    Entity* cameraEntity =
        (new Entity(Vector3f(-1, 7, 28)))
            ->AddComponent(new CameraComponent(Matrix4f().InitPerspective(
                ToRadians(70.0f), window.GetAspect(), 0.1f, 1000.0f)))
            ->AddComponent(new FreeLook(window.GetCenter()))
            ->AddComponent(new FreeMove(10.0f));
    // Aim the camera at the action on load (tilts down onto the set-pieces
    // instead of staring at the horizon). FreeLook takes over on mouse movement.
    cameraEntity->GetTransform()->LookAt(Vector3f(2, 3, 0), Vector3f(0, 1, 0));
    AddToScene(cameraEntity);

    // Point light overhead + a directional "sun" so nothing is dark.
    AddToScene((new Entity(Vector3f(0, 15, 8)))
                   ->AddComponent(new PointLight(Vector3f(1, 1, 1), 60,
                                                 Attenuation(0, 0, 1))));
    // The sun casts shadows: shadowMapSizeAsPowerOf2=10 (1024^2 variance shadow
    // map) turns on shadow mapping, so every body drops a grounded shadow onto
    // the checker floor. shadowArea=60 covers the scene (ramp at -x .. stack at
    // +x) in the shadow camera's ortho frustum.
    AddToScene(
        (new Entity(Vector3f(0, 0, 0),
                    Quaternion(Vector3f(1, 0, 0), ToRadians(50.0f))))
            ->AddComponent(new DirectionalLight(Vector3f(1.0f, 1.0f, 0.96f),
                                                0.6f, 10, 60.0f)));

    // Shared physics engine on its own scene entity; its Update() steps the sim.
    PhysicsEngineComponent* physics = new PhysicsEngineComponent();
    AddToScene((new Entity())->AddComponent(physics));
    Physics::PhysicsEngine& engine = physics->GetPhysicsEngine();

    // Partly-inelastic so drops lose energy and settle quickly (engine default
    // is 1.0). 0.2 so bodies stop bouncing sooner.
    engine.SetRestitution(0.2f);

    // Stronger-than-earth gravity so bodies fall with weight instead of floating.
    engine.SetGravity(Vector3f(0, -15, 0));

    // Let settled piles sleep (skip integrate+solve) so a big stack of resting
    // balls stops costing solver time; contact or reset wakes them again.
    engine.SetSleepingEnabled(true);

    // Ground: a static plane at y=0 for physics, plus an INFINITE-LOOKING visual
    // floor. Instead of a finite cube slab (whose [0,1] UVs would smear the
    // checker into one giant square when scaled up), build one huge quad with
    // TILED UVs — the checker (GL_REPEAT by default) stays crisp near the action
    // while the edges sit ~500 units out, past view. The quad is DOUBLE-WOUND
    // (each triangle plus its reverse) with up-normals, so it renders correctly
    // regardless of the engine's backface-cull winding.
    std::size_t floorIdx =
        engine.AddObject(Physics::PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));
    // Give the floor friction so bodies grip and settle instead of sliding
    // forever. Friction is combined per-contact, so the floor needs it too, not
    // just the balls.
    engine.GetObject(floorIdx).SetFriction(0.6f);
    {
        const float S = 500.0f;       // half-size -> 1000x1000, edges past view
        const float R = 250.0f;       // UV tiling -> ~4-unit checker squares
        IndexedModel floor;
        const Vector3f corners[4] = {Vector3f(-S, 0, -S), Vector3f(S, 0, -S),
                                     Vector3f(-S, 0, S), Vector3f(S, 0, S)};
        const Vector2f uvs[4] = {Vector2f(0, 0), Vector2f(R, 0), Vector2f(0, R),
                                 Vector2f(R, R)};
        for (int side = 0; side < 2; ++side) {  // two coincident, opposite windings
            unsigned int base = side * 4;
            for (int i = 0; i < 4; ++i) {
                floor.AddVertex(corners[i]);
                floor.AddTexCoord(uvs[i]);
                floor.AddNormal(Vector3f(0, 1, 0));  // lit as up-facing either way
            }
            if (side == 0) {
                floor.AddFace(base + 0, base + 1, base + 2);
                floor.AddFace(base + 2, base + 1, base + 3);
            } else {
                floor.AddFace(base + 2, base + 1, base + 0);
                floor.AddFace(base + 3, base + 1, base + 2);
            }
        }
        floor.CalcTangents();
        AddToScene((new Entity())
                       ->AddComponent(new MeshRenderer(
                           Mesh("infiniteFloor", floor.Finalize()),
                           Material("checker"))));
    }

    // Helpers: spawn a dynamic body + its rendered entity, wired by stable index,
    // with a per-body friction coefficient (default 0 = frictionless).
    // Damping so rolled/spun bodies bleed energy and come to rest instead of
    // spinning forever. Angular dominates so spin decays; tiny linear so throws
    // still travel.
    const float kAngularDamping = 0.4f;
    const float kLinearDamping = 0.05f;
    auto addSphere = [&](const Vector3f& pos, const char* material,
                         float friction) {
        std::size_t index =
            engine.AddObject(Physics::PhysicsObject::Sphere(pos, 1.0f));
        engine.GetObject(index).SetFriction(friction);
        engine.GetObject(index).SetAngularDamping(kAngularDamping);
        engine.GetObject(index).SetLinearDamping(kLinearDamping);
        AddToScene(
            (new Entity(pos))
                ->AddComponent(
                    new MeshRenderer(Mesh("sphere.obj"), Material(material)))
                ->AddComponent(new PhysicsObjectComponent(&engine, index)));
    };
    auto addBox = [&](const Vector3f& pos, const char* material, float friction) {
        const Vector3f half(1, 1, 1);
        std::size_t index = engine.AddObject(
            Physics::PhysicsObject::Box(pos - half, pos + half));
        engine.GetObject(index).SetFriction(friction);
        engine.GetObject(index).SetAngularDamping(kAngularDamping);
        engine.GetObject(index).SetLinearDamping(kLinearDamping);
        AddToScene(
            (new Entity(pos))
                ->AddComponent(
                    new MeshRenderer(Mesh("cube.obj"), Material(material)))
                ->AddComponent(new PhysicsObjectComponent(&engine, index)));
    };

    // ---- SET-PIECE 1: FRICTION RAMP (-x) -----------------------------------
    // A static (invMass 0) OrientedBox tilted ~20deg, with friction. Two balls
    // dropped onto it in separate z-rows: high-friction one rolls (spins down
    // the slope), frictionless one slides without spin.
    const Vector3f rampCenter(-8, 2.5f, 0);
    const float rampHalf = 3.0f;
    const Quaternion rampRot(Vector3f(0, 0, 1), ToRadians(20.0f));
    std::size_t rampIdx = engine.AddObject(Physics::PhysicsObject::OrientedBox(
        rampCenter, Vector3f(rampHalf, rampHalf, rampHalf), rampRot,
        Vector3f(0, 0, 0), 0.0f));  // invMass 0 => static ramp
    engine.GetObject(rampIdx).SetFriction(0.7f);
    AddToScene((new Entity(rampCenter, rampRot, rampHalf))
                   ->AddComponent(
                       new MeshRenderer(Mesh("cube.obj"), Material("metal"))));
    addSphere(Vector3f(-8, 10, -1.5f), "bricks", 0.9f);  // rolls
    addSphere(Vector3f(-8, 10, 1.5f), "bricks", 0.0f);   // slides

    // ---- SET-PIECE 2: STABLE STACK (+x) ------------------------------------
    // Five axis-aligned boxes stacked exactly (2 tall each). Axis-aligned on
    // purpose (BOX has no rotational inertia; OBB is the rotatable one). The
    // sequential-impulse solver + friction keeps this tower standing.
    for (int i = 0; i < 5; ++i)
        addBox(Vector3f(8, 1.0f + 2.0f * i, 0), "bricks2", 0.6f);

    // ---- SET-PIECE 3: BALL-ON-BALL (centre) --------------------------------
    addSphere(Vector3f(0, 1, 0), "bricks", 0.5f);  // resting on the floor
    addSphere(Vector3f(0, 8, 0), "bricks", 0.5f);  // drops onto the one above

    // SPACE throws a ball from the camera aim; R resets. The spawner holds its
    // Mesh+Material by value so nothing gets evicted. Red "bricks" balls read
    // apart from the metal ramp.
    Entity* spawner = new Entity();
    // launchSpeed 12 (default is 20): slower throws so balls don't spin up to a
    // manic rate on impact — a gripping ball reaches rolling speed omega = v/r,
    // so a slower throw spins slower.
    spawner->AddComponent(new BallSpawnerComponent(
        &engine, spawner, cameraEntity->GetTransform(), Mesh("sphere.obj"),
        Material("bricks"), 1.0f /*radius*/, 12.0f /*launchSpeed*/));
    AddToScene(spawner);
}

int main() {
    // No text/HUD renderer in this engine, so print the controls to the console
    // at startup.
    std::cout << "\n=== Physics Sandbox — controls ===\n"
                 "  SPACE      spawn / throw a ball from the camera aim\n"
                 "  R          reset the scene\n"
                 "  W A S D    move the camera\n"
                 "  Mouse      look around\n"
                 "  ESC        release / recapture the mouse\n"
                 "  Watch: the ramp (rolling vs sliding ball), the 5-box stack\n"
                 "  standing under the solver, and the ball-on-ball drop.\n"
                 "==================================\n\n";

    TestGame game;
    Window window(1000, 600, "3D Engine Visual");

    RenderingEngine renderer(window);
    // Dark slate background so the sandbox isn't framed by a black void.
    renderer.SetClearColor(Vector3f(0.06f, 0.07f, 0.09f));

    CoreEngine engine(60, &window, &renderer, &game);
    engine.Start();

    return 0;
}
