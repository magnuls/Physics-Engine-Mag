#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Continuous collision detection via speculative contacts. Off by default,
// enabled per body via SetContinuous or globally via CcdSpeedThreshold.

namespace {

constexpr float kDt = 1.0f / 60.0f;

// A fast sphere flying +x at 200 u/s toward a thin static wall at x ~ 5. One
// step jumps clean over it.
void buildTunnelScene(PhysicsEngine& engine) {
    engine.SetGravity(Vector3f(0, 0, 0));
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 0.5f, Vector3f(200, 0, 0)));
    engine.AddObject(PhysicsObject::Box(Vector3f(4.8f, -3, -3),
                                        Vector3f(5.2f, 3, 3), Vector3f(0, 0, 0),
                                        /*invMass=*/0.0f));
}

void step(PhysicsEngine& engine, int n) {
    for (int i = 0; i < n; ++i) {
        engine.Simulate(kDt);
        engine.HandleCollisions();
    }
}

}  // namespace

// Without CCD the sphere tunnels straight through the wall.
TEST(CcdTest, FastSphereTunnelsWithoutCcd) {
    PhysicsEngine engine;
    buildTunnelScene(engine);

    step(engine, 6);  // ~20 units of travel

    EXPECT_GT(engine.GetObject(0).GetPosition().GetX(), 6.0f);  // past the wall
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 200.0f, 1e-3f);
}

// With SetContinuous the sphere does not tunnel; it lands on the wall and
// rebounds.
TEST(CcdTest, ContinuousSphereHitsThinWall) {
    PhysicsEngine engine;
    buildTunnelScene(engine);
    engine.GetObject(0).SetContinuous(true);

    for (int i = 0; i < 10; ++i) {
        step(engine, 1);
        // Never crosses the wall's near face at any step.
        EXPECT_LT(engine.GetObject(0).GetPosition().GetX(), 4.81f);
    }
    EXPECT_LT(engine.GetObject(0).GetVelocity().GetX(), 0.0f);  // rebounded
}

// With a speed threshold set, fast bodies are treated as continuous with no
// per body opt in.
TEST(CcdTest, SpeedThresholdAutoCatchesFastBody) {
    PhysicsEngine engine;
    buildTunnelScene(engine);
    engine.SetCcdSpeedThreshold(1.0f);  // 200 u/s * dt = 3.33 u/step > 1.0

    step(engine, 10);

    EXPECT_LT(engine.GetObject(0).GetPosition().GetX(), 4.81f);  // stopped
    EXPECT_LT(engine.GetObject(0).GetVelocity().GetX(), 0.0f);   // rebounded
}

// A slow body stays below the threshold and flies untouched until it really
// contacts.
TEST(CcdTest, SlowBodyUnaffectedByThreshold) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, 0, 0));
    engine.SetCcdSpeedThreshold(1.0f);  // 2 u/s * dt = 0.033 u/step < 1.0
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 0.5f, Vector3f(2, 0, 0)));
    engine.AddObject(PhysicsObject::Box(Vector3f(4.8f, -3, -3),
                                        Vector3f(5.2f, 3, 3), Vector3f(0, 0, 0),
                                        /*invMass=*/0.0f));

    step(engine, 30);  // 0.5 s -> reaches x=1, still ~3.3 from the wall face

    EXPECT_NEAR(engine.GetObject(0).GetPosition().GetX(), 1.0f, 1e-3f);
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 2.0f, 1e-4f);
}

// A thrown continuous ball landing on a frictional floor must never gain energy
// across bounces.
TEST(CcdTest, ContinuousLandingInjectsNoEnergy) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.3f);  // bouncy and frictional
    PhysicsObject ball = PhysicsObject::Sphere(Vector3f(0, 5, 0), 1.0f,
                                               Vector3f(20, -5, 0));  // thrown
    ball.SetFriction(0.5f);
    ball.SetContinuous(true);
    engine.AddObject(ball);
    engine.AddObject(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.5f));

    // |v| can never exceed sqrt(|v0|^2 + 2*g*h0) ~ 22.9 and friction bounds |w|.
    // Checked every step across many bounces.
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
        EXPECT_LT(engine.GetObject(0).GetVelocity().Length(), 24.0f);
        EXPECT_LT(engine.GetObject(0).GetAngularVelocity().Length(), 35.0f);
    }
    // And it ends on the floor, not launched into orbit.
    EXPECT_LT(engine.GetObject(0).GetPosition().GetY(), 3.0f);
    EXPECT_GT(engine.GetObject(0).GetPosition().GetY(), 0.5f);
}

// A continuous ball dropped under gravity still settles on the floor normally.
TEST(CcdTest, ContinuousBallStillSettlesOnFloor) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.2f);
    engine.AddObject(PhysicsObject::Sphere(Vector3f(0, 5, 0), 1.0f));
    engine.GetObject(0).SetContinuous(true);
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    step(engine, 600);  // ~10 s

    EXPECT_NEAR(engine.GetObject(0).GetPosition().GetY(), 1.0f, 0.15f);
    EXPECT_LT(std::fabs(engine.GetObject(0).GetVelocity().GetY()), 0.5f);
}
