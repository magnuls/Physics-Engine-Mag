#include <cmath>
#include <iostream>
#include <string>

#include "3DEngine.h"
#include "components/ballSpawnerComponent.h"
#include "components/freeLook.h"
#include "components/freeMove.h"
#include "components/physicsEngineComponent.h"
#include "components/physicsObjectComponent.h"
#include "physics/physicsObject.h"

// Guided linear walkthrough: the player starts at z 55 and walks down a
// straight path along -z. Stations flank the path left at +x and right at -x,
// each with a floating text label, and the far end has a ledge finale.
// Labels are RGBA pngs with a transparent background hung on quads; the
// shaders discard clear texels so only the letters render. A controls HUD
// quad is parented to the camera so it rides the lower right of the view.
// SPACE throws a ball, UP/DOWN tune speed, Q/E dial spin, R resets and also
// hides every thrown ball. sphere.obj is unit radius, cube.obj is [-1,1].
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
    // One flat color per body kind so every object reads at a glance.
    // Each material is kept alive by a MeshRenderer or the spawner below.
    Material floorMat("floor", Texture("floor_light.png"), 0.05f, 4);
    Material red("red", Texture("flat_red.png"), 0.2f, 8);
    Material blue("blue", Texture("flat_blue.png"), 0.2f, 8);
    Material green("green", Texture("flat_green.png"), 0.2f, 8);
    Material orange("orange", Texture("flat_orange.png"), 0.2f, 8);
    Material yellow("yellow", Texture("flat_yellow.png"), 0.2f, 8);
    Material purple("purple", Texture("flat_purple.png"), 0.2f, 8);
    Material grey("grey", Texture("flat_grey.png"), 0.3f, 8);
    Material bullseye("bullseye", Texture("bullseye.png"), 0.2f, 4);
    Material hud("hud", Texture("hud_controls.png"), 0.0f, 0);

    // One material per label png, kept alive so the runtime meshes find them.
    Material signFriction("signFriction", Texture("sign_friction.png"), 0.0f, 0);
    Material signStack("signStack", Texture("sign_stack.png"), 0.0f, 0);
    Material signKnock("signKnock", Texture("sign_knock.png"), 0.0f, 0);
    Material signTumble("signTumble", Texture("sign_tumble.png"), 0.0f, 0);
    Material signTargets("signTargets", Texture("sign_targets.png"), 0.0f, 0);
    Material signDominoes("signDominoes", Texture("sign_dominoes.png"), 0.0f, 0);
    Material signFinale("signFinale", Texture("sign_finale.png"), 0.0f, 0);

    // Vertical quad mesh, double wound so it draws under either cull winding.
    // World signs face +z toward the walk-in direction; looking down -z screen
    // right is world -x, so u starts at +x and the text is not mirrored.
    // HUD quads flip that: the camera looks down its local +z, screen right is
    // local +x, so u starts at -x and the normal points back at the camera.
    int quadSerial = 0;
    auto makeQuadMesh = [&](float w, float h, bool hudFacing) -> Mesh {
        const float hw = w * 0.5f;
        const float hh = h * 0.5f;
        const float xs = hudFacing ? -1.0f : 1.0f;
        const Vector3f n(0, 0, hudFacing ? -1.0f : 1.0f);
        // Corners as seen on screen: TL, TR, BL, BR.
        const Vector3f corners[4] = {
            Vector3f(xs * hw, hh, 0), Vector3f(-xs * hw, hh, 0),
            Vector3f(xs * hw, -hh, 0), Vector3f(-xs * hw, -hh, 0)};
        const Vector2f uvs[4] = {Vector2f(0, 0), Vector2f(1, 0), Vector2f(0, 1),
                                 Vector2f(1, 1)};
        IndexedModel quad;
        for (int side = 0; side < 2; ++side) {  // two coincident windings
            unsigned int base = side * 4;
            for (int i = 0; i < 4; ++i) {
                quad.AddVertex(corners[i]);
                quad.AddTexCoord(uvs[i]);
                quad.AddNormal(n);
            }
            if (side == 0) {
                quad.AddFace(base + 0, base + 1, base + 2);
                quad.AddFace(base + 2, base + 1, base + 3);
            } else {
                quad.AddFace(base + 2, base + 1, base + 0);
                quad.AddFace(base + 3, base + 1, base + 2);
            }
        }
        quad.CalcTangents();
        std::string name = "quad" + std::to_string(quadSerial++);
        return Mesh(name.c_str(), quad.Finalize());
    };

    // Camera starts at the near end of the path, eye height-ish, looking
    // straight down it toward the finale. Captured so the ball spawner reads
    // its aim.
    Entity* cameraEntity =
        (new Entity(Vector3f(0, 6, 58)))
            ->AddComponent(new CameraComponent(Matrix4f().InitPerspective(
                ToRadians(70.0f), window.GetAspect(), 0.1f, 1000.0f)))
            ->AddComponent(new FreeLook(window.GetCenter()))
            ->AddComponent(new FreeMove(15.0f));
    // Aim down the path on load; FreeLook takes over on mouse movement.
    cameraEntity->GetTransform()->LookAt(Vector3f(0, 3, -10),
                                         Vector3f(0, 1, 0));

    // Controls HUD: a quad parented to the camera entity, so it inherits the
    // camera transform every frame and stays pinned to the lower right of the
    // view. Placement from the 70 deg vertical FOV and window aspect: at
    // depth d the half view height is d * tan 35 and half width is that times
    // aspect. The panel hugs the corner with a small margin. The png is
    // transparent outside the letters, so it never blocks the scene.
    {
        const float hudDist = 2.0f;
        const float halfH = hudDist * tanf(ToRadians(35.0f));
        const float halfW = halfH * window.GetAspect();
        const float hudW = 1.3f;
        const float hudH = 0.9f;
        const float margin = 0.12f;
        const Vector3f hudPos(halfW - margin - hudW * 0.5f,
                              -(halfH - margin - hudH * 0.5f), hudDist);
        Entity* hudPanel = new Entity(hudPos);
        hudPanel->AddComponent(
            new MeshRenderer(makeQuadMesh(hudW, hudH, true), Material("hud")));
        cameraEntity->AddChild(hudPanel);
    }
    AddToScene(cameraEntity);

    // Bright, even lighting: overhead point light, a shadowed sun, and a
    // shadowless fill aimed down the path so object fronts are never dark.
    AddToScene((new Entity(Vector3f(0, 20, -5)))
                   ->AddComponent(new PointLight(Vector3f(1, 1, 1), 80,
                                                 Attenuation(0, 0, 1))));
    // Shadows on the sun: 10 = 1024^2 shadow map, 140 covers the long path.
    AddToScene(
        (new Entity(Vector3f(0, 0, 0),
                    Quaternion(Vector3f(1, 0, 0), ToRadians(50.0f))))
            ->AddComponent(new DirectionalLight(Vector3f(1.0f, 1.0f, 0.96f),
                                                0.8f, 10, 140.0f)));
    // Fill light: rotating 160 deg about x sends its forward down and -z, so
    // it lights the faces the player walks toward. No shadow map.
    AddToScene(
        (new Entity(Vector3f(0, 0, 0),
                    Quaternion(Vector3f(1, 0, 0), ToRadians(160.0f))))
            ->AddComponent(
                new DirectionalLight(Vector3f(1.0f, 1.0f, 1.0f), 0.35f)));

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
        const float R = 125.0f;       // UV tiling, calm 4 unit checker squares
        IndexedModel floor;
        const Vector3f corners[4] = {Vector3f(-S, 0, -S), Vector3f(S, 0, -S),
                                     Vector3f(-S, 0, S), Vector3f(S, 0, S)};
        const Vector2f uvs[4] = {Vector2f(0, 0), Vector2f(R, 0), Vector2f(0, R),
                                 Vector2f(R, R)};
        for (int side = 0; side < 2; ++side) {  // two coincident windings
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
                           Material("floor"))));
    }

    // A floating text label hung in the air, facing +z toward the player.
    auto addSign = [&](const Vector3f& center, float w, float h,
                       const char* materialName) {
        AddToScene((new Entity(center))
                       ->AddComponent(new MeshRenderer(makeQuadMesh(w, h, false),
                                                       Material(materialName))));
    };

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
    // Dynamic unit box, always invMass 1 so a thrown ball can knock it.
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

    // Cuboid mesh with arbitrary half extents, since Transform scale is a
    // single scalar and cube.obj can only render cubes. Double wound like the
    // floor so it draws under either cull winding.
    int boxSerial = 0;
    auto makeBoxMesh = [&](const Vector3f& half) -> Mesh {
        const Vector3f axes[3] = {Vector3f(1, 0, 0), Vector3f(0, 1, 0),
                                  Vector3f(0, 0, 1)};
        const float he[3] = {half.GetX(), half.GetY(), half.GetZ()};
        IndexedModel model;
        unsigned int base = 0;
        for (int a = 0; a < 3; ++a) {
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                const Vector3f n = axes[a] * (float)sgn;
                const Vector3f cn = axes[a] * (he[a] * (float)sgn);
                const Vector3f cu = axes[(a + 1) % 3] * he[(a + 1) % 3];
                const Vector3f cv = axes[(a + 2) % 3] * he[(a + 2) % 3];
                const Vector3f corners[4] = {cn + cu + cv, cn - cu + cv,
                                             cn + cu - cv, cn - cu - cv};
                const Vector2f uvs[4] = {Vector2f(0, 0), Vector2f(1, 0),
                                         Vector2f(0, 1), Vector2f(1, 1)};
                for (int side = 0; side < 2; ++side) {
                    for (int i = 0; i < 4; ++i) {
                        model.AddVertex(corners[i]);
                        model.AddTexCoord(uvs[i]);
                        model.AddNormal(n);
                    }
                    if (side == 0) {
                        model.AddFace(base + 0, base + 1, base + 2);
                        model.AddFace(base + 2, base + 1, base + 3);
                    } else {
                        model.AddFace(base + 2, base + 1, base + 0);
                        model.AddFace(base + 3, base + 1, base + 2);
                    }
                    base += 4;
                }
            }
        }
        model.CalcTangents();
        std::string name = "boxMesh" + std::to_string(boxSerial++);
        return Mesh(name.c_str(), model.Finalize());
    };

    // Oriented box body plus its rendered cuboid. invMass 0 makes it static
    // scenery, anything else is dynamic and syncs pos + rot every frame.
    auto addObbBody = [&](const Vector3f& center, const Vector3f& half,
                          const Quaternion& rot, const char* material,
                          float friction, float invMass) {
        std::size_t index = engine.AddObject(Physics::PhysicsObject::OrientedBox(
            center, half, rot, Vector3f(0, 0, 0), invMass));
        engine.GetObject(index).SetFriction(friction);
        Entity* e = new Entity(center, rot);
        e->AddComponent(new MeshRenderer(makeBoxMesh(half), Material(material)));
        if (invMass > 0.0f) {
            engine.GetObject(index).SetAngularDamping(kAngularDamping);
            engine.GetObject(index).SetLinearDamping(kLinearDamping);
            e->AddComponent(new PhysicsObjectComponent(&engine, index));
        }
        AddToScene(e);
    };

    // A round target: a bullseye quad whose png is transparent outside the
    // outer ring, backed by a dynamic thin oriented box sized to the disc.
    // Rotated -90 deg about y so the face points at the walking path, and it
    // rests on its edge like a standing archery target until a ball hits it.
    const float kDiscR = 1.8f;
    auto addTargetDisc = [&](const Vector3f& center) {
        const Quaternion faceRot(Vector3f(0, 1, 0), ToRadians(-90.0f));
        const Vector3f half(kDiscR, kDiscR, 0.25f);
        std::size_t index = engine.AddObject(Physics::PhysicsObject::OrientedBox(
            center, half, faceRot, Vector3f(0, 0, 0), 1.0f));
        engine.GetObject(index).SetFriction(0.6f);
        engine.GetObject(index).SetAngularDamping(kAngularDamping);
        engine.GetObject(index).SetLinearDamping(kLinearDamping);
        AddToScene(
            (new Entity(center, faceRot))
                ->AddComponent(new MeshRenderer(
                    makeQuadMesh(kDiscR * 2.0f, kDiscR * 2.0f, false),
                    Material("bullseye")))
                ->AddComponent(new PhysicsObjectComponent(&engine, index)));
    };

    // ---- STATION 1, LEFT +x z 35: FRICTION, roll vs slide ----
    // A static slab ramp tilted about z, high side near the path so both
    // balls start where the player can see them and roll away outward.
    // Rotation about z by a negative angle lifts the -x side.
    {
        const float tilt = ToRadians(-20.0f);
        const Vector3f rampC(14, 1.8f, 35);
        addObbBody(rampC, Vector3f(5, 0.5f, 3.5f), Quaternion(Vector3f(0, 0, 1), tilt),
                   "grey", 0.7f, 0.0f);
        // Ball centers sit on the top face near the high edge: local x -3.5,
        // local y 0.5 + 1 for the radius, rotated by the tilt.
        const float c = cosf(tilt);
        const float s = sinf(tilt);
        const float lx = -3.5f;
        const float ly = 1.5f;
        const float bx = rampC.GetX() + lx * c - ly * s;
        const float by = rampC.GetY() + lx * s + ly * c;
        addSphere(Vector3f(bx, by, 33.2f), "red", 0.9f);   // grips, rolls
        addSphere(Vector3f(bx, by, 36.8f), "blue", 0.0f);  // no grip, slides
    }
    addSign(Vector3f(14, 9, 35), 10.0f, 4.0f, "signFriction");

    // ---- STATION 2, RIGHT -x z 22: STABLE STACK ----
    // Six dynamic boxes standing under the sequential impulse solver.
    for (int i = 0; i < 6; ++i)
        addBox(Vector3f(-13, 1.0f + 2.0f * i, 22), "green", 0.6f);
    addSign(Vector3f(-13, 16, 22), 10.0f, 4.0f, "signStack");

    // ---- STATION 3, LEFT +x z 8: KNOCK IT DOWN ----
    // A five box dynamic tower begging for a thrown ball.
    for (int i = 0; i < 5; ++i)
        addBox(Vector3f(13, 1.0f + 2.0f * i, 8), "orange", 0.6f);
    addSign(Vector3f(13, 14, 8), 10.0f, 4.0f, "signKnock");

    // ---- STATION 4, RIGHT -x z -6: TUMBLING BOXES ----
    // Tilted dynamic OBBs dropped from a bit of height: they land on corners
    // and edges, pick up torque, and tumble to rest. Shows angular dynamics.
    addObbBody(Vector3f(-13, 3.0f, -6), Vector3f(1, 1, 1),
               Quaternion(Vector3f(0, 0, 1), ToRadians(30.0f)), "purple", 0.6f, 1.0f);
    addObbBody(Vector3f(-16, 4.5f, -7.5f), Vector3f(1, 1, 1),
               Quaternion(Vector3f(1, 0, 0), ToRadians(35.0f)), "purple", 0.6f, 1.0f);
    addObbBody(Vector3f(-10.5f, 5.5f, -7.5f), Vector3f(1, 1, 1),
               Quaternion(Vector3f(0, 0, 1), ToRadians(-25.0f)), "purple", 0.6f, 1.0f);
    addSign(Vector3f(-13, 10, -6), 10.0f, 4.0f, "signTumble");

    // ---- STATION 5, LEFT +x z -22: HIT THE TARGETS ----
    // Three round bullseye discs facing the path, each a dynamic thin body,
    // so one clean hit knocks a target flying.
    addTargetDisc(Vector3f(9.0f, kDiscR + 0.05f, -19.5f));
    addTargetDisc(Vector3f(13.5f, kDiscR + 0.05f, -22.0f));
    addTargetDisc(Vector3f(18.0f, kDiscR + 0.05f, -24.5f));
    addSign(Vector3f(13, 10, -22), 10.0f, 4.0f, "signTargets");

    // ---- STATION 6, RIGHT -x z -36: DOMINOES ----
    // Tall thin OBBs in a row along x, spaced so one push cascades the rest.
    // The nearest one is closest to the path, so throw at that.
    for (int i = 0; i < 6; ++i)
        addObbBody(Vector3f(-8.5f - 2.6f * i, 2.2f, -36), Vector3f(0.4f, 2.2f, 1.2f),
                   Quaternion(0, 0, 0, 1), "blue", 0.6f, 1.0f);
    addSign(Vector3f(-15, 9, -36), 10.0f, 4.0f, "signDominoes");

    // ---- FINALE, CENTER z -64: SHOOT THE BALLS OFF THE LEDGE ----
    // A static flat platform with five balls resting on top, and a static
    // ramp running from the platform's front edge down to the floor. Knocked
    // balls drop over the edge and roll or tumble down the ramp.
    {
        const Vector3f platC(0, 2, -64);
        const Vector3f platHalf(7, 2, 5);  // top face at y 4, front edge z -59
        addObbBody(platC, platHalf, Quaternion(0, 0, 0, 1), "grey", 0.6f, 0.0f);

        // Ramp slab: top surface runs from the front edge top down to the
        // floor over a 10 unit run. Tilt about x tips the top normal toward
        // +z, so the surface descends toward the player. Center = incline
        // midpoint minus half a thickness along the surface normal.
        const float drop = 4.0f;
        const float run = 10.0f;
        const float theta = atan2f(drop, run);
        const float slope = sqrtf(drop * drop + run * run);
        const float ct = cosf(theta);
        const float st = sinf(theta);
        const Vector3f rampC(0, drop * 0.5f - 0.5f * ct,
                             -59.0f + run * 0.5f + 0.5f * st);
        addObbBody(rampC, Vector3f(7, 0.5f, slope * 0.5f),
                   Quaternion(Vector3f(1, 0, 0), theta), "grey", 0.6f, 0.0f);

        // Five balls resting on the flat top, just behind the front edge.
        // Flat surface, so they sit still until something hits them.
        for (int i = 0; i < 5; ++i)
            addSphere(Vector3f(-5.0f + 2.5f * i, 5.0f, -61), "red", 0.6f);
    }
    addSign(Vector3f(0, 10, -55), 16.0f, 6.0f, "signFinale");

    // SPACE throws a ball from the camera aim; UP/DOWN speed, Q/E spin, R
    // resets the stations and hides every thrown ball. Thrown balls are
    // yellow so they read apart from everything else.
    Entity* spawner = new Entity();
    spawner->AddComponent(new BallSpawnerComponent(
        &engine, spawner, cameraEntity->GetTransform(), Mesh("sphere.obj"),
        Material("yellow"), 1.0f /*radius*/, 16.0f /*launchSpeed*/));
    AddToScene(spawner);
}

int main() {
    // The in-game HUD shows the controls too; the full tour lives here.
    std::cout << "\n=== Physics Walkthrough ===\n"
                 "  A straight path runs ahead of you. Walk it front to back;\n"
                 "  stations line the left and right, the finale waits at the\n"
                 "  end. Controls stay on the HUD in the lower right corner.\n"
                 "\n"
                 "  Controls:\n"
                 "  W A S D    walk\n"
                 "  Mouse      look\n"
                 "  SPACE      throw a ball from the camera aim\n"
                 "  UP/DOWN    launch speed +/- 2, range 5 to 40\n"
                 "  E / Q      spin toward topspin / backspin, +/- 4 per press\n"
                 "  R          reset the stations and clear thrown balls\n"
                 "  ESC        release / recapture the mouse\n"
                 "\n"
                 "  Stations, in walking order:\n"
                 "    FRICTION    left:  one ball rolls, one slides down a ramp\n"
                 "    STACK       right: a six box tower held up by the solver\n"
                 "    KNOCK       left:  a tower to throw balls at\n"
                 "    TUMBLING    right: tilted boxes tip and tumble\n"
                 "    TARGETS     left:  round bullseye discs, knock them over\n"
                 "    DOMINOES    right: topple the near one, watch the cascade\n"
                 "    FINALE      end:   shoot the resting balls off the ledge\n"
                 "                       and watch them tumble down the ramp\n"
                 "===========================\n\n";

    TestGame game;
    Window window(1000, 600, "3D Engine Visual");

    RenderingEngine renderer(window);
    // Bright sky blue so the walkthrough reads like an outdoor sandbox, and
    // a raised ambient floor so no face is ever murky dark.
    renderer.SetClearColor(Vector3f(0.53f, 0.81f, 0.92f));
    renderer.SetVector3f("ambient", Vector3f(0.42f, 0.42f, 0.44f));

    CoreEngine engine(60, &window, &renderer, &game);
    engine.Start();

    return 0;
}
