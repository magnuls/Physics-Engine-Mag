#include <gtest/gtest.h>

// Extra CCD checks: speculative contacts, opt-in via SetContinuous(bool)
// (default off) plus an optional CcdSpeedThreshold.
#define PHYSICS_CCD_LANDED 1

#if PHYSICS_CCD_LANDED

#include <algorithm>
#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

namespace {

// A fast ball aimed straight at a STATIC target sphere across a gap wider than
// one step. With dt = 1/60 and v = 360 m/s the ball advances 6 m/step; the
// target sits at x = 15 (r = 1) and the ball has r = 0.5, so the collision
// window for the ball centre is x in [13.5, 16.5]. Step positions 0,6,12,18,...
// straddle that window (12 -> 18) — i.e. WITHOUT CCD the ball tunnels clean
// through in a single step.
constexpr float kDt = 1.0f / 60.0f;

PhysicsObject staticTarget() {
    return PhysicsObject::Sphere(Vector3f(15, 0, 0), 1.0f, Vector3f(0, 0, 0),
                                 0.0f);  // invMass 0 => static
}
PhysicsObject fastBall() {
    return PhysicsObject::Sphere(Vector3f(0, 0, 0), 0.5f, Vector3f(360, 0, 0));
}
void run(PhysicsEngine& e, int steps) {
    for (int i = 0; i < steps; ++i) {
        e.Simulate(kDt);
        e.HandleCollisions();
    }
}

}  // namespace

// Continuous is off by default.
TEST(CcdVerify, DefaultsToOff) { EXPECT_FALSE(fastBall().IsContinuous()); }

// A fast ball with CCD off tunnels clean through the static target — ends up
// past the far side, never having collided.
TEST(CcdVerify, FastSphereTunnelsWithoutCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    e.AddObject(fastBall());  // index 0, CCD off (default)
    e.AddObject(staticTarget());
    run(e, 4);

    EXPECT_GT(e.GetObject(0).GetPosition().GetX(), 16.5f);   // past the target
    EXPECT_GT(e.GetObject(0).GetVelocity().GetX(), 300.0f);  // never collided
}

// Same setup with SetContinuous(true) — the ball stops at the contact instead
// of passing through.
TEST(CcdVerify, FastSphereStoppedWithCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    PhysicsObject ball = fastBall();
    ball.SetContinuous(true);
    e.AddObject(ball);
    e.AddObject(staticTarget());
    run(e, 4);

    // The speculative contact clamps the ball to the contact surface exactly —
    // x = 15 - 1 - 0.5 = 13.5, vx = 0.
    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f,
                0.2f);  // stops at contact
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()),
              1.0f);  // no pass-through
}

// Optional engine knob: a body moving faster than the CCD speed threshold is
// auto-treated as continuous even without an explicit SetContinuous(true).
TEST(CcdVerify, SpeedThresholdAutoEnablesCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    e.SetCcdSpeedThreshold(1.0f);  // 1 m/step; the ball does 6 m/step
    e.AddObject(fastBall());       // CCD off, but over threshold
    e.AddObject(staticTarget());
    run(e, 4);

    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f,
                0.2f);  // auto-CCD stops at contact
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()), 1.0f);
}

// Guard against over-reach: with CCD ON, a NORMAL (slow) approach must still
// resolve exactly like a discrete contact — the ball rolls up and rests at the
// contact surface (x = 15 - 1 - 0.5 = 13.5), no spurious early stop.
TEST(CcdVerify, CcdOnLeavesNormalCollisionUnchanged) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 0.5f, Vector3f(6, 0, 0));
    ball.SetContinuous(true);
    e.AddObject(ball);
    e.AddObject(staticTarget());
    run(e, 200);  // 0.1 m/step -> reaches the target, then rests

    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f, 0.5f);
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()), 0.5f);
}

// Regression for the CCD energy-injection bug (speculative contacts must be
// cold-started and skip friction). Only reproduces with the full recipe: CCD on
// + friction > 0 on both ball and floor + restitution > 0 + several bounces. A
// fast continuous frictional ball on a bouncy frictional floor must keep |v|
// and |w| bounded across many bounces — the bug drove |v| ~20 -> ~90 m/s and
// |w| -> ~300 rad/s.
TEST(CcdVerify, ContinuousFrictionalBallOnBouncyFloorStaysBounded) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, -9.81f, 0));
    e.SetRestitution(0.5f);  // bouncy -> many bounces (needed to trip the bug)

    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 8, 0), 1.0f, Vector3f(18, -6, 0));
    ball.SetContinuous(true);  // CCD on — the trigger
    ball.SetFriction(0.6f);    // friction on the ball ...
    e.AddObject(ball);
    e.AddObject(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.6f));  // ... and floor

    float peakV = 0.0f, peakW = 0.0f;
    for (int i = 0; i < 600; ++i) {  // 10 s — many bounces
        e.Simulate(kDt);
        e.HandleCollisions();
        const PhysicsObject& b = e.GetObject(0);
        peakV = std::max(peakV, b.GetVelocity().Length());
        peakW = std::max(peakW, b.GetAngularVelocity().Length());
        ASSERT_FALSE(std::isnan(peakV) || std::isnan(peakW)) << "NaN at step " << i;
    }
    // Bounds sit well above healthy motion (impact ~20 m/s, roll ~20 rad/s) yet
    // far below the bug's runaway (|v| ~90, |w| ~300). A regression trips these.
    EXPECT_LT(peakV, 45.0f) << "linear velocity ran away";
    EXPECT_LT(peakW, 90.0f) << "angular velocity ran away";
}

#else

// Placeholder so this file compiles and the suite stays green until CCD exists.
TEST(CcdVerify, PendingCcdLanding) {
    GTEST_SKIP()
        << "Continuous collision detection not built yet; set "
           "PHYSICS_CCD_LANDED to 1 to activate the tunnel/no-tunnel tests.";
}

#endif
