#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// P0 — friction + robust contact solver: Coulomb friction (tangent impulses
// clamped to the friction cone), sequential-impulse iteration with warm
// starting, and the A2 support-point contacts that let boxes tip on corners.

namespace {

// A box resting exactly on the floor plane y=0, sliding along +x.
PhysicsObject slidingBox(float vx, float mu) {
    PhysicsObject box =
        PhysicsObject::Box(Vector3f(-1, -0.005f, -1), Vector3f(1, 1.995f, 1),
                           Vector3f(vx, 0, 0));
    box.SetFriction(mu);
    return box;
}

PhysicsObject frictionFloor(float mu) {
    PhysicsObject floor = PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f);
    floor.SetFriction(mu);
    return floor;
}

}  // namespace

// Friction defaults to 0: a sliding body is NOT decelerated (the engine stays
// physically neutral; scenes opt in).
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

// Coulomb friction decelerates a sliding box to rest: |dv/dt| = mu * g, so
// with mu = 0.5 a 5 m/s slide stops after ~1.02 s — and STAYS stopped.
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

    // After 2 s total it is at rest — and friction never reverses the motion.
    for (int i = 0; i < 90; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
    }
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 0.0f, 0.05f);
    EXPECT_GE(engine.GetObject(0).GetVelocity().GetX(), -1e-3f);
}

// The friction impulse is clamped to the Coulomb cone |Jt| <= mu * Jn: a fast
// slide with a gentle impact loses at most mu * (normal impulse) of tangent
// momentum in that contact — it must NOT stop dead (that would need |Jt| >> cone).
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

    // Normal impulse stops vy (= 1.0 * m); friction may remove at most
    // mu * Jn = 0.5 m/s of the 10 m/s slide.
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetY(), 0.0f, 1e-3f);
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 9.5f, 1e-3f);
}

// Friction at the contact spins a sphere up until it ROLLS without slipping
// (slip velocity v + w x r -> 0 at the contact), rather than stopping it.
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
    // Rolling without slipping on r=1: contact slip = vx + wz*r = 0, with
    // backspin torque making wz negative for +x travel; sliding-to-rolling
    // conserves angular momentum about the contact: vx -> 5 * 5/7 ~ 3.57.
    EXPECT_LT(wz, -1.0f);
    EXPECT_NEAR(vx + wz, 0.0f, 0.1f);
    EXPECT_NEAR(vx, 5.0f * 5.0f / 7.0f, 0.2f);
}

// The sequential-impulse solver holds a two-box STACK: both boxes settle and
// stay put (the mid-stack box satisfies its floor and its top contact at once).
TEST(FrictionTest, TwoBoxStackSettles) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    PhysicsObject bottom = PhysicsObject::Box(
        Vector3f(-1, 0.02f, -1), Vector3f(1, 2.02f, 1));  // dropped 2 cm up
    PhysicsObject top = PhysicsObject::Box(Vector3f(-1, 2.1f, -1),
                                           Vector3f(1, 4.1f, 1));
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

// PHYS-FEEL regression (Round 4): friction is a PAIR property combined as
// sqrt(muA*muB) — one frictionless side zeroes the pair. This is the exact
// mechanism behind the "demo friction isn't working" report (the demo floor
// had mu=0): a frictional box on a FRICTIONLESS floor slides freely. Pinned
// here so the combine semantics can't drift silently.
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

// PHYS-FEEL fix validation: the spec'd demo values (floor mu=0.6 vs body
// mu=0.5 -> pair sqrt(0.30)~0.55) bring a 5 m/s slider to rest in ~1 s.
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

// A2 payoff (support-point contacts in obbCollision.cpp): an OBB landing on
// the floor TILTED contacts at its corner, whose off-center lever arm produces
// torque — the box starts tipping. (Center-projected contacts gave zero torque
// here, which is exactly the limitation A1 logged.)
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
