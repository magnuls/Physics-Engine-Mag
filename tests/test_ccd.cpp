#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// CCD-1 — continuous collision detection via speculative contacts (contract in
// API-REFERENCE.md §6). Default OFF: nothing changes unless a body opts in via
// SetContinuous(true) or the engine's CcdSpeedThreshold is set.

namespace {

constexpr float kDt = 1.0f / 60.0f;

// A fast sphere (r=0.5) flying +x at 200 u/s (~3.33 u/step) toward a thin
// static wall: an AABB slab 0.4 thick at x ~ 5. One integration step jumps
// clean over it.
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

// Baseline (documents the problem): without CCD the sphere tunnels straight
// through the wall — it ends up on the far side with its velocity untouched.
TEST(CcdTest, FastSphereTunnelsWithoutCcd) {
    PhysicsEngine engine;
    buildTunnelScene(engine);

    step(engine, 6);  // ~20 units of travel

    EXPECT_GT(engine.GetObject(0).GetPosition().GetX(), 6.0f);  // past the wall
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().GetX(), 200.0f, 1e-3f);
}

// The contract case: the SAME scene with SetContinuous(true) does NOT tunnel —
// the speculative contact caps the approach at gap/dt, the sphere lands on the
// wall the next step and the real contact rebounds it.
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

// The engine-level knob: with a speed threshold set (distance per step), fast
// bodies are auto-treated as continuous without any per-body opt-in.
TEST(CcdTest, SpeedThresholdAutoCatchesFastBody) {
    PhysicsEngine engine;
    buildTunnelScene(engine);
    engine.SetCcdSpeedThreshold(1.0f);  // 200 u/s * dt = 3.33 u/step > 1.0

    step(engine, 10);

    EXPECT_LT(engine.GetObject(0).GetPosition().GetX(), 4.81f);  // stopped
    EXPECT_LT(engine.GetObject(0).GetVelocity().GetX(), 0.0f);   // rebounded
}

// A slow body stays below the threshold: its approach is NOT speculatively
// braked — it flies untouched until it really contacts, exactly as before.
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

// Energy-injection regression (Agent 3's repro): a thrown CONTINUOUS ball
// landing on a frictional floor under gravity must never GAIN energy. The bug:
// warm-starting a speculative contact re-applied the previous real landing's
// cached friction impulses with no speculative friction solve to correct them
// — a free tangential kick at a long lever arm that pumped |v| ~20 -> ~90 and
// |w| -> ~300 (sign-reversing) across bounces.
TEST(CcdTest, ContinuousLandingInjectsNoEnergy) {
    PhysicsEngine engine;
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.3f);  // demo-like: bouncy + frictional
    PhysicsObject ball = PhysicsObject::Sphere(Vector3f(0, 5, 0), 1.0f,
                                               Vector3f(20, -5, 0));  // thrown
    ball.SetFriction(0.5f);
    ball.SetContinuous(true);
    engine.AddObject(ball);
    engine.AddObject(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.5f));

    // Energy bound: |v| can never exceed sqrt(|v0|^2 + 2*g*h0) ~ 22.9 (all
    // potential energy converted, nothing added). Friction bounds |w| by the
    // cone; ~35 rad/s is generous. Check EVERY step, across many bounces.
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {
        engine.Simulate(dt);
        engine.HandleCollisions();
        EXPECT_LT(engine.GetObject(0).GetVelocity().Length(), 24.0f);
        EXPECT_LT(engine.GetObject(0).GetAngularVelocity().Length(), 35.0f);
    }
    // And it ends up ON the floor, not launched into orbit.
    EXPECT_LT(engine.GetObject(0).GetPosition().GetY(), 3.0f);
    EXPECT_GT(engine.GetObject(0).GetPosition().GetY(), 0.5f);
}

// Regression guard: CCD must not destabilize ordinary behavior — a continuous
// ball dropped under gravity still settles on the floor like any other body.
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
