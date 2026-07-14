#include <iostream>

#include "3DEngine.h"
#include "components/ballSpawnerComponent.h"
#include "components/freeLook.h"
#include "components/freeMove.h"
#include "components/physicsEngineComponent.h"
#include "components/physicsObjectComponent.h"
#include "physics/physicsObject.h"

// Interactive physics sandbox demo: friction ramp, box stack, ball drop.
// SPACE spawns a ball, R resets. sphere.obj is unit radius, cube.obj is [-1,1].
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
    // One material per body kind. Each is kept alive by a MeshRenderer below.
    Material bricks2("bricks2", Texture("bricks2.jpg"), 0.0f, 0,
                     Texture("bricks2_normal.png"), Texture("bricks2_disp.jpg"),
                     0.04f, -1.0f);
    Material checker("checker", Texture("checker.png"), 0.1f, 2);
    Material metal("metal", Texture("metal.jpg"), 0.6f, 8);
    Material bricks("bricks", Texture("bricks.jpg"), 0.2f, 4);

    // Camera framed on the scene. Captured so the ball spawner can read its aim.
    Entity* cameraEntity =
        (new Entity(Vector3f(-1, 7, 28)))
            ->AddComponent(new CameraComponent(Matrix4f().InitPerspective(
                ToRadians(70.0f), window.GetAspect(), 0.1f, 1000.0f)))
            ->AddComponent(new FreeLook(window.GetCenter()))
            ->AddComponent(new FreeMove(10.0f));
    // Aim at the scene on load; FreeLook takes over on mouse movement.
    cameraEntity->GetTransform()->LookAt(Vector3f(2, 3, 0), Vector3f(0, 1, 0));
    AddToScene(cameraEntity);

    // Overhead point light plus a directional sun so nothing is dark.
    AddToScene((new Entity(Vector3f(0, 15, 8)))
                   ->AddComponent(new PointLight(Vector3f(1, 1, 1), 60,
                                                 Attenuation(0, 0, 1))));
    // Enable shadows on the sun: 10 = 1024^2 shadow map, 60 = area covered.
    AddToScene(
        (new Entity(Vector3f(0, 0, 0),
                    Quaternion(Vector3f(1, 0, 0), ToRadians(50.0f))))
            ->AddComponent(new DirectionalLight(Vector3f(1.0f, 1.0f, 0.96f),
                                                0.6f, 10, 60.0f)));

    // Shared physics engine on its own entity; its Update method steps the sim.
    PhysicsEngineComponent* physics = new PhysicsEngineComponent();
    AddToScene((new Entity())->AddComponent(physics));
    Physics::PhysicsEngine& engine = physics->GetPhysicsEngine();

    // Lose energy on impact so drops settle. Engine default is 1.0.
    engine.SetRestitution(0.2f);

    // Heavier than earth gravity so bodies fall with weight, not floaty.
    engine.SetGravity(Vector3f(0, -15, 0));

    // Let settled piles sleep so they stop costing solver time. Contact or
    // reset wakes them.
    engine.SetSleepingEnabled(true);

    // Physics floor is an infinite plane. The visual is one big tiled quad,
    // double wound so it draws no matter which way backface culling runs.
    std::size_t floorIdx =
        engine.AddObject(Physics::PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));
    // Floor needs its own friction: it combines per contact, so a 0 side slides.
    engine.GetObject(floorIdx).SetFriction(0.6f);
    {
        const float S = 500.0f;       // half extent, gives a 1000x1000 floor
        const float R = 250.0f;       // UV tiling, about 4 unit checker squares
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
                floor.AddNormal(Vector3f(0, 1, 0));  // always lit as up
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

    // Spawn a dynamic body plus its rendered entity, wired by stable index.
    // Damping lets spun bodies bleed energy and settle; angular does most of it.
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

    // Friction ramp at -x: a static tilted box. One ball rolls, one slides.
    const Vector3f rampCenter(-8, 2.5f, 0);
    const float rampHalf = 3.0f;
    const Quaternion rampRot(Vector3f(0, 0, 1), ToRadians(20.0f));
    std::size_t rampIdx = engine.AddObject(Physics::PhysicsObject::OrientedBox(
        rampCenter, Vector3f(rampHalf, rampHalf, rampHalf), rampRot,
        Vector3f(0, 0, 0), 0.0f));  // invMass 0 is static
    engine.GetObject(rampIdx).SetFriction(0.7f);
    AddToScene((new Entity(rampCenter, rampRot, rampHalf))
                   ->AddComponent(
                       new MeshRenderer(Mesh("cube.obj"), Material("metal"))));
    addSphere(Vector3f(-8, 10, -1.5f), "bricks", 0.9f);  // rolls
    addSphere(Vector3f(-8, 10, 1.5f), "bricks", 0.0f);   // slides

    // Stable stack at +x: five axis aligned boxes. BOX has no spin inertia, so
    // the solver holds the tower upright.
    for (int i = 0; i < 5; ++i)
        addBox(Vector3f(8, 1.0f + 2.0f * i, 0), "bricks2", 0.6f);

    // Ball drop in the centre.
    addSphere(Vector3f(0, 1, 0), "bricks", 0.5f);  // resting on the floor
    addSphere(Vector3f(0, 8, 0), "bricks", 0.5f);  // drops onto the resting one

    // SPACE throws a ball from the camera aim; R resets.
    Entity* spawner = new Entity();
    // Slower launch than the default 20 so balls spin up less on impact.
    spawner->AddComponent(new BallSpawnerComponent(
        &engine, spawner, cameraEntity->GetTransform(), Mesh("sphere.obj"),
        Material("bricks"), 1.0f /*radius*/, 12.0f /*launchSpeed*/));
    AddToScene(spawner);
}

int main() {
    // No HUD renderer, so print the controls to the console.
    std::cout << "\n=== Physics Sandbox controls ===\n"
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
