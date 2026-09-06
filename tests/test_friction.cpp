#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Friction and contact solver: Coulomb friction, sequential impulse iteration
// with warm starting, and support points that let boxes tip on corners.

namespace {

// A box resting exactly on the floor plane y=0, sliding along +x.
PhysicsObject slidingBox(float vx, float mu) {
    PhysicsObject box = PhysicsObject::Box(
        Vector3f(-1, -0.005f, -1), Vector3f(1, 1.995f, 1), Vector3f(vx, 0, 0));
    box.SetFriction(mu);
    return box;
}

PhysicsObject frictionFloor(float mu) {
    PhysicsObject floor = PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f);
    floor.SetFriction(mu);
    return floor;
}

}  // namespace

// Friction defaults to 0, so a sliding body is not decelerated.
TEST(FrictionTest, DefaultZeroFrictionKeepsSliding) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.AddObject(slidingBox(5.0f, 0.0f));
    engine.AddObject(frictionFloor(0.0f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {  // 1 s under gravity, in contact throughout
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 5.0f, 1e-3f);
}

// Coulomb friction decelerates a sliding box to rest at |dv/dt| = mu * g, so a
// 5 m/s slide at mu = 0.5 stops after ~1 s and stays stopped.
TEST(FrictionTest, SlidingBoxDeceleratesToRestAtMuG) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.AddObject(slidingBox(5.0f, 0.5f));
    engine.AddObject(frictionFloor(0.5f));  // combined sqrt(.5*.5) = 0.5

    const float dt = 1.0f / 60.0f;
    // After 0.5 s the box should have lost ~ mu*g*t = 2.45 m/s.
    for (int i = 0; i < 30; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 5.0f - 2.4525f,
                0.15f);

    // After 2 s it is at rest, and friction never reverses the motion.
    for (int i = 0; i < 90; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 0.0f, 0.05f);
    EXPECT_GE(engine.GetObject(0).GetVelocity().GetX(), -1e-3f);
}

// The friction impulse is clamped to the Coulomb cone |Jt| <= mu * Jn, so a
// fast slide with a gentle impact loses only a little tangent speed, not all.
TEST(FrictionTest, FrictionImpulseClampedToCoulombCone) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    // No gravity: a single collision, so the impulses are exactly analyzable.
    engine.SetGravity(Vector3f(0, 0, 0));
    PhysicsObject box =
        PhysicsObject::Box(Vector3f(-1, -0.05f, -1), Vector3f(1, 1.95f, 1),
                           Vector3f(10.0f, -1.0f, 0));  // fast slide, soft hit
    box.SetFriction(0.5f);
    engine.AddObject(box);
    engine.AddObject(frictionFloor(0.5f));

    engine.HandleCollisions();

    // Normal impulse stops vy; friction removes at most mu * Jn = 0.5 m/s of
    // the 10 m/s slide.
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetY(), 0.0f, 1e-3f);
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 9.5f, 1e-3f);
}

// Friction spins a sphere up until it rolls without slipping instead of
// stopping it.
TEST(FrictionTest, SphereSpinsUpToRolling) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0.995f, 0), 1.0f, Vector3f(5, 0, 0));
    ball.SetFriction(0.5f);
    engine.AddObject(ball);
    engine.AddObject(frictionFloor(0.5f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
    }

    const PhysicsObject& b = engine.GetObject(0);
    float vx = b.GetVelocity().GetX();
    float wz = b.GetAngularVelocity().GetZ();
    // Rolling without slipping on r=1: slip = vx + wz*r = 0, wz goes negative,
    // and vx settles to 5/7 of its start by conservation about the contact.
    EXPECT_LT(wz, -1.0f);
    EXPECT_NEAR(vx + wz, 0.0f, 0.1f);
    EXPECT_NEAR(vx, 5.0f * 5.0f / 7.0f, 0.2f);
}

// The solver holds a two box stack: both boxes settle and stay put.
TEST(FrictionTest, TwoBoxStackSettles) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    PhysicsObject bottom = PhysicsObject::Box(
        Vector3f(-1, 0.02f, -1), Vector3f(1, 2.02f, 1));  // dropped 2 cm up
    PhysicsObject top =
        PhysicsObject::Box(Vector3f(-1, 2.1f, -1), Vector3f(1, 4.1f, 1));
    bottom.SetFriction(0.5f);
    top.SetFriction(0.5f);
    engine.AddObject(bottom);
    engine.AddObject(top);
    engine.AddObject(frictionFloor(0.5f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {  // ~10 s
        engine.Simulate(dt);
        engine.HandleCollisions();
    }

    // Bottom box rests with center ~1 above the floor, top box ~1 above that.
    EXPECT_NEAR(engine.GetObject(0).GetPosition().GetY(), 1.0f, 0.1f);
    EXPECT_NEAR(engine.GetObject(1).GetPosition().GetY(), 3.0f, 0.15f);
    EXPECT_LT(std::fabs(engine.GetObject(0).GetVelocity().GetY()), 0.1f);
    EXPECT_LT(std::fabs(engine.GetObject(1).GetVelocity().GetY()), 0.1f);
}

// Friction is a pair property combined as sqrt(muA*muB), so one frictionless
// side zeroes the pair and the box slides freely.
TEST(FrictionTest, OneSidedZeroFrictionSlidesFreely) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.AddObject(slidingBox(5.0f, 0.5f));  // box HAS friction...
    engine.AddObject(frictionFloor(0.0f));     // ...floor has none -> pair 0

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 5.0f, 1e-3f);
}

// StaticPlane can take friction at construction; the short form stays
// frictionless.
TEST(FrictionTest, StaticPlaneFrictionParam) {
    EXPECT_NEAR(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.6f).GetFriction(),
        0.6f, 1e-6f);
    EXPECT_NEAR(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f).GetFriction(), 0.0f,
        1e-6f);
}

// Floor mu=0.6 with body mu=0.5 gives pair ~0.55, bringing a 5 m/s slider to
// rest in ~1 s.
TEST(FrictionTest, FrictionalFloorStopsSliderDemoSpec) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.AddObject(slidingBox(5.0f, 0.5f));
    engine.AddObject(frictionFloor(0.6f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; ++i) {  // 2 s >> v/(mu*g) ~ 0.93 s
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 0.0f, 0.05f);
}

// An OBB landing tilted contacts at a corner, whose off center lever arm makes
// it start tipping.
TEST(FrictionTest, TiltedObbLandingOnPlaneStartsTipping) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    // Rotate 30 deg about z: the lowest corner is NOT below the center.
    Quaternion tilt(Vector3f(0, 0, 1), ToRadians(30.0f));
    engine.AddObject(PhysicsObject::OrientedBox(
        Vector3f(0, 1.3f, 0), Vector3f(1, 1, 1), tilt, Vector3f(0, -4, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.HandleCollisions();

    Vector3f w = engine.GetObject(0).GetAngularVelocity();
    EXPECT_GT(std::fabs(w.GetZ()), 0.1f);  // corner impact induces tipping spin
}
