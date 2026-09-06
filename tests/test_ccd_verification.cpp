#include <gtest/gtest.h>

// Extra CCD checks. Speculative contacts, off by default, enabled per body via
// SetContinuous or globally via CcdSpeedThreshold.
#define PHYSICS_CCD_LANDED 1

#if PHYSICS_CCD_LANDED

#include <algorithm>
#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

namespace {

// Fast ball aimed at a static target across a gap wider than one step. At
// v = 360 m/s it moves 6 m per step, so without CCD it skips the target at x = 15.
constexpr float kDt = 1.0f / 60.0f;

PhysicsObject staticTarget() {
    return PhysicsObject::Sphere(Vector3f(15, 0, 0), 1.0f, Vector3f(0, 0, 0),
                                 0.0f);  // invMass 0 is static
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

// A fast ball with CCD off tunnels clean through the static target.
TEST(CcdVerify, FastSphereTunnelsWithoutCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    e.AddObject(fastBall());  // CCD off by default
    e.AddObject(staticTarget());
    run(e, 4);

    EXPECT_GT(e.GetObject(0).GetPosition().GetX(), 16.5f);   // past the target
    EXPECT_GT(e.GetObject(0).GetVelocity().GetX(), 300.0f);  // never collided
}

// With SetContinuous the ball stops at the contact instead of passing through.
TEST(CcdVerify, FastSphereStoppedWithCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    PhysicsObject ball = fastBall();
    ball.SetContinuous(true);
    e.AddObject(ball);
    e.AddObject(staticTarget());
    run(e, 4);

    // Ball clamped exactly to the contact surface: x = 13.5, vx = 0.
    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f,
                0.2f);  // stops at contact
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()),
              1.0f);  // no pass through
}

// A body faster than the CCD speed threshold is treated as continuous with no
// explicit opt in.
TEST(CcdVerify, SpeedThresholdAutoEnablesCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    e.SetCcdSpeedThreshold(1.0f);  // 1 m/step; the ball does 6 m/step
    e.AddObject(fastBall());       // CCD off but over threshold
    e.AddObject(staticTarget());
    run(e, 4);

    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f,
                0.2f);  // auto CCD stops at contact
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()), 1.0f);
}

// With CCD on, a slow approach still resolves like a normal contact and rests
// at the surface with no early stop.
TEST(CcdVerify, CcdOnLeavesNormalCollisionUnchanged) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 0, 0), 0.5f, Vector3f(6, 0, 0));
    ball.SetContinuous(true);
    e.AddObject(ball);
    e.AddObject(staticTarget());
    run(e, 200);  // 0.1 m/step, reaches the target then rests

    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f, 0.5f);
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()), 0.5f);
}

// Regression for the CCD energy pump. A fast continuous frictional ball on a
// bouncy frictional floor must keep |v| and |w| bounded across many bounces.
TEST(CcdVerify, ContinuousFrictionalBallOnBouncyFloorStaysBounded) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, -9.81f, 0));
    e.SetRestitution(0.5f);  // bouncy, many bounces

    PhysicsObject ball =
        PhysicsObject::Sphere(Vector3f(0, 8, 0), 1.0f, Vector3f(18, -6, 0));
    ball.SetContinuous(true);  // CCD on
    ball.SetFriction(0.6f);    // friction on the ball
    e.AddObject(ball);
    e.AddObject(
        PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f, 0.6f));  // and the floor

    float peakV = 0.0f, peakW = 0.0f;
    for (int i = 0; i < 600; ++i) {  // 10 s
        e.Simulate(kDt);
        e.HandleCollisions();
        const PhysicsObject& b = e.GetObject(0);
        peakV = std::max(peakV, b.GetVelocity().Length());
        peakW = std::max(peakW, b.GetAngularVelocity().Length());
        ASSERT_FALSE(std::isnan(peakV) || std::isnan(peakW)) << "NaN at step " << i;
    }
    // Bounds sit above healthy motion but well below the bug's runaway.
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
