#include <gtest/gtest.h>

#include "../src/components/physicsEngineComponent.h"
#include "../src/components/physicsObjectComponent.h"
#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// --- Integration ------------------------------------------------------------

// Semi-implicit Euler: v += g*dt first, then pos += v*dt.
TEST(PhysicsEngineTest, GravityIntegration) {
    PhysicsObject o = PhysicsObject::Sphere(Vector3f(0, 10, 0), 1.0f);
    o.Integrate(0.5f, Vector3f(0, -10, 0));
    // v = 0 + (-10)(0.5) = -5 ;  y = 10 + (-5)(0.5) = 7.5
    EXPECT_NEAR(o.GetVelocity().GetY(), -5.0f, 1e-4f);
    EXPECT_NEAR(o.GetPosition().GetY(), 7.5f, 1e-4f);
}

// A static body (invMass 0) is never integrated.
TEST(PhysicsEngineTest, StaticBodyDoesNotMove) {
    PhysicsObject plane = PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f);
    plane.Integrate(1.0f, Vector3f(0, -10, 0));
    EXPECT_TRUE(plane.IsStatic());
    EXPECT_NEAR(plane.GetVelocity().GetY(), 0.0f, 1e-6f);

    PhysicsObject box = PhysicsObject::Box(Vector3f(-1, -1, -1),
                                           Vector3f(1, 1, 1),
                                           Vector3f(0, 0, 0), /*invMass=*/0.0f);
    box.Integrate(1.0f, Vector3f(0, -10, 0));
    EXPECT_TRUE(box.IsStatic());
    EXPECT_NEAR(box.GetPosition().GetY(), 0.0f, 1e-6f);
}

// Engine::Simulate integrates every dynamic body and leaves static ones put.
TEST(PhysicsEngineTest, SimulateAdvancesOnlyDynamicBodies) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, -10, 0));
    std::size_t sphere = engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 20, 0), 1.0f));            // dynamic
    std::size_t floor =
        engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.Simulate(1.0f);
    EXPECT_LT(engine.GetObject(sphere).GetPosition().GetY(), 20.0f);  // fell
    EXPECT_TRUE(engine.GetObject(floor).IsStatic());
}

// --- Response ---------------------------------------------------------------

// A sphere sinking into a static floor plane has its velocity reflected upward.
TEST(PhysicsEngineTest, SphereBouncesOffStaticPlane) {
    PhysicsEngine engine;
    // Sphere (index 0) overlapping the plane y=0: center y=0.5, r=1, falling.
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f, Vector3f(0, -5, 0)));
    // Static floor (index 1), plane y = 0, normal up.
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.HandleCollisions();

    const Vector3f& v = engine.GetObject(0).GetVelocity();
    EXPECT_GT(v.GetY(), 0.0f);          // now moving up
    EXPECT_NEAR(v.GetY(), 5.0f, 1e-3f);  // speed preserved (restitution = 1)
}

// Two dynamic spheres approaching head-on both reverse.
TEST(PhysicsEngineTest, TwoSpheresReflectHeadOn) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(-0.5f, 0, 0), 1.0f, Vector3f(3, 0, 0)));
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0.5f, 0, 0), 1.0f, Vector3f(-3, 0, 0)));

    engine.HandleCollisions();

    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), -3.0f, 1e-3f);
    EXPECT_NEAR(engine.GetObject(1).GetVelocity().GetX(), 3.0f, 1e-3f);
}

// Overlapping but SEPARATING spheres are not re-reflected (no jitter).
TEST(PhysicsEngineTest, SeparatingSpheresAreNotReflected) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(-0.5f, 0, 0), 1.0f, Vector3f(-3, 0, 0)));
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0.5f, 0, 0), 1.0f, Vector3f(3, 0, 0)));

    engine.HandleCollisions();

    // Velocities are unchanged because the bodies are already moving apart.
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), -3.0f, 1e-3f);
    EXPECT_NEAR(engine.GetObject(1).GetVelocity().GetX(), 3.0f, 1e-3f);
}

// The Entity/Component bridges construct and expose their engine. (Driving
// Update() needs a live Entity parent; here we just verify the wiring compiles
// and the owned engine is reachable for setup.)
TEST(PhysicsEngineTest, ComponentBridgesWireUp) {
    PhysicsEngineComponent engineComp;
    std::size_t idx = engineComp.GetPhysicsEngine().AddObject(
        PhysicsObject::Sphere(Vector3f(0, 5, 0), 1.0f));
    EXPECT_EQ(engineComp.GetPhysicsEngine().GetNumObjects(), 1u);

    PhysicsObjectComponent objComp(&engineComp.GetPhysicsEngine(), idx);
    (void)objComp;
    EXPECT_NEAR(
        engineComp.GetPhysicsEngine().GetObject(idx).GetPosition().GetY(), 5.0f,
        1e-6f);
}

// A far-away body is left untouched while a near pair resolves (whole pipeline).
TEST(PhysicsEngineTest, DistantBodyUnaffected) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f, Vector3f(0, -5, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));
    // Far dynamic sphere, nowhere near the floor contact.
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(100, 100, 100), 1.0f, Vector3f(1, 2, 3)));

    engine.HandleCollisions();

    EXPECT_GT(engine.GetObject(0).GetVelocity().GetY(), 0.0f);  // bounced
    const Vector3f& far = engine.GetObject(2).GetVelocity();
    EXPECT_NEAR(far.GetX(), 1.0f, 1e-6f);  // untouched
    EXPECT_NEAR(far.GetY(), 2.0f, 1e-6f);
    EXPECT_NEAR(far.GetZ(), 3.0f, 1e-6f);
}
