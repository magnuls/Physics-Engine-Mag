#include <gtest/gtest.h>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Per-body linear/angular damping: v *= 1/(1 + d*dt) after the velocity update
// in Integrate(). Defaults 0 keep every existing trajectory bit-identical.

namespace {
constexpr float kDt = 1.0f / 60.0f;
}

// A spun body's |w| decays exponentially: after 2 s at d=0.5 the per-step
// product (1 + 0.5*dt)^-120 ~ e^-1 ~ 0.369 of the initial rate remains.
TEST(DampingTest, SpunBodyAngularVelocityDecays) {
    PhysicsObject box = PhysicsObject::OrientedBox(
        Vector3f(0, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1));
    box.SetAngularVelocity(Vector3f(0, 0, 10.0f));
    box.SetAngularDamping(0.5f);

    for (int i = 0; i < 120; ++i) box.Integrate(kDt, Vector3f(0, 0, 0));

    EXPECT_NEAR(box.GetAngularVelocity().GetZ(), 10.0f * 0.3694f, 0.1f);
}

// Linear damping decays translation the same way.
TEST(DampingTest, LinearDampingDecaysVelocity) {
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 1.0f, Vector3f(10, 0, 0));
    ball.SetLinearDamping(0.5f);

    for (int i = 0; i < 120; ++i) ball.Integrate(kDt, Vector3f(0, 0, 0));

    EXPECT_NEAR(ball.GetVelocity().GetX(), 10.0f * 0.3694f, 0.1f);
}

// Default damping 0 is an EXACT no-op (multiply by 1/(1+0) == 1.0f), so every
// pre-existing trajectory in the suite is bit-identical.
TEST(DampingTest, ZeroDampingIsExactNoOp) {
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 1.0f, Vector3f(5, 0, 0));
    PhysicsObject box = PhysicsObject::OrientedBox(
        Vector3f(0, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1));
    box.SetAngularVelocity(Vector3f(0, 0, 3.0f));

    for (int i = 0; i < 120; ++i) {
        ball.Integrate(kDt, Vector3f(0, 0, 0));
        box.Integrate(kDt, Vector3f(0, 0, 0));
    }

    EXPECT_EQ(ball.GetVelocity().GetX(), 5.0f);
    EXPECT_EQ(box.GetAngularVelocity().GetZ(), 3.0f);
}

// A damped ball on a frictional floor spins up toward rolling, then bleeds
// energy, comes to rest and — with sleeping enabled — falls asleep. Without
// damping this exact setup rolls forever.
TEST(DampingTest, DampedRollerComesToRestAndSleeps) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.SetSleepingEnabled(true);
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0.995f, 0), 1.0f, Vector3f(5, 0, 0));
    ball.SetFriction(0.5f);
    ball.SetLinearDamping(1.0f);
    ball.SetAngularDamping(2.0f);
    engine.AddObject(ball);
    engine.AddObject(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.5f));

    for (int i = 0; i < 600; ++i) {  // 10 s — rest is reached in ~4 s
        engine.Simulate(kDt);
        engine.HandleCollisions();
    }

    EXPECT_FALSE(engine.GetObject(0).IsAwake());  // rolled to rest, then slept
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().Length(), 0.0f, 1e-6f);
    EXPECT_NEAR(engine.GetObject(0).GetAngularVelocity().Length(), 0.0f, 1e-6f);
}
