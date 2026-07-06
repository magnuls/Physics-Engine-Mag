#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// V3 — impulse-based collision response: inverse mass, restitution, and
// positional correction. (Default restitution is 1.0, which reproduces the
// original elastic reflection — see PhysicsEngineTest.)

// Restitution 0 is fully inelastic: the approaching normal velocity is removed.
TEST(ResponseTest, RestitutionZeroStops) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f, Vector3f(0, -5, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.HandleCollisions();
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetY(), 0.0f, 1e-4f);
}

// Restitution 0.5 returns half the impact speed.
TEST(ResponseTest, RestitutionHalfBounce) {
    PhysicsEngine engine;
    engine.SetRestitution(0.5f);
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f, Vector3f(0, -5, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.HandleCollisions();
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetY(), 2.5f, 1e-3f);
}

// Inverse mass matters: an infinite-mass (invMass 0) body acts like a wall —
// it does not move, and the light body carries all the rebound.
TEST(ResponseTest, InfiniteMassActsLikeWall) {
    PhysicsEngine engine;  // default restitution 1.0
    engine.AddObject(PhysicsObject::Sphere(Vector3f(0, 0, 0), 1.0f,
                                           Vector3f(0, 0, 0), /*invMass=*/0.0f));
    engine.AddObject(PhysicsObject::Sphere(Vector3f(1.5f, 0, 0), 1.0f,
                                           Vector3f(-4, 0, 0), /*invMass=*/1.0f));

    engine.HandleCollisions();
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 0.0f, 1e-4f);  // wall
    EXPECT_NEAR(engine.GetObject(1).GetVelocity().GetX(), 4.0f, 1e-3f);  // rebounds
}

// Positional correction pushes an overlapping body out along the contact normal.
TEST(ResponseTest, PositionalCorrectionReducesPenetration) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f));  // sunk 0.5 into floor
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    float before = engine.GetObject(0).GetPosition().GetY();
    engine.HandleCollisions();
    float after = engine.GetObject(0).GetPosition().GetY();
    EXPECT_GT(after, before);        // pushed up out of the floor
    EXPECT_LE(after, 1.0f + 1e-4f);  // but not overshot past resting height
}

// The payoff: a ball dropped under gravity SETTLES on the floor (comes to rest
// near its radius above the plane) instead of sinking through or bouncing
// forever. This is what makes the sandbox look believable.
TEST(ResponseTest, BallSettlesOnFloorUnderGravity) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.2f);
    engine.AddObject(PhysicsObject::Sphere(Vector3f(0, 5, 0), 1.0f));  // r=1
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {  // ~10 s of simulation
        engine.Simulate(dt);
        engine.HandleCollisions();
    }

    float y = engine.GetObject(0).GetPosition().GetY();
    float vy = engine.GetObject(0).GetVelocity().GetY();
    EXPECT_NEAR(y, 1.0f, 0.15f);          // resting ~radius above the plane
    EXPECT_LT(std::fabs(vy), 0.5f);        // essentially at rest (not bouncing)
    EXPECT_GT(y, 0.0f);                     // did NOT sink through the floor
}
