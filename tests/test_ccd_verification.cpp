#include <gtest/gtest.h>

// CCD-1 VERIFICATION (Agent 3) — written to the LOCKED API-REFERENCE.md §6
// "CCD-1" contract BEFORE the implementation lands (interface-first workflow:
// build the verification against the contract so it's ready the moment CCD ships).
//
// CCD is owned by Agent 1 (Fable 5): speculative contacts inside
// HandleCollisions(), opt-in via PhysicsObject::SetContinuous(bool) (default
// off) + optional PhysicsEngine::SetCcdSpeedThreshold(float). No IntersectData /
// collision<>() change.
//
// ACTIVATION: until PhysicsObject::SetContinuous exists this file compiles as one
// SKIPPED placeholder so `./unit_tests` stays green. When Agent 1 lands CCD-1,
// flip PHYSICS_CCD_LANDED to 1 (and drop the #else placeholder) to activate the
// tunnel/no-tunnel tests. (These bodies are written to the contract but have not
// been compiled yet — expect to reconcile against the real symbols on activation.)
//
// ACTIVATED 2026-07-06 (Agent 3): CCD-1 landed (Agent 1) and the signatures match
// the contract verbatim, so the real tests below are now live.
#define PHYSICS_CCD_LANDED 1

#if PHYSICS_CCD_LANDED

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

// Contract: continuous is OFF by default (so the whole existing suite is
// unaffected).
TEST(CcdVerify, DefaultsToOff) {
    EXPECT_FALSE(fastBall().IsContinuous());
}

// Baseline (the bug CCD fixes): a fast ball with CCD OFF tunnels clean through
// the static target — it ends up past the far side, never having collided.
TEST(CcdVerify, FastSphereTunnelsWithoutCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    e.AddObject(fastBall());     // index 0, CCD off (default)
    e.AddObject(staticTarget());
    run(e, 4);

    EXPECT_GT(e.GetObject(0).GetPosition().GetX(), 16.5f);  // past the target
    EXPECT_GT(e.GetObject(0).GetVelocity().GetX(), 300.0f);  // never collided
}

// The fix: same setup with SetContinuous(true) — the ball is stopped at the
// contact instead of passing through.
TEST(CcdVerify, FastSphereStoppedWithCcd) {
    PhysicsEngine e;
    e.SetGravity(Vector3f(0, 0, 0));
    e.SetRestitution(0.0f);
    PhysicsObject ball = fastBall();
    ball.SetContinuous(true);
    e.AddObject(ball);
    e.AddObject(staticTarget());
    run(e, 4);

    // Measured on activation (2026-07-06): the speculative contact clamps the
    // ball to the contact surface exactly — x = 15 - 1 - 0.5 = 13.5, vx = 0.
    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f, 0.2f);  // stops at contact
    EXPECT_LT(std::fabs(e.GetObject(0).GetVelocity().GetX()), 1.0f);  // no pass-through
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

    EXPECT_NEAR(e.GetObject(0).GetPosition().GetX(), 13.5f, 0.2f);  // auto-CCD stops at contact
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

#else

// Placeholder so this file compiles and the suite stays green until CCD lands.
TEST(CcdVerify, PendingCcdLanding) {
    GTEST_SKIP() << "CCD-1 not landed (PhysicsObject::SetContinuous absent). "
                    "Flip PHYSICS_CCD_LANDED to 1 in this file when Agent 1 "
                    "lands CCD-1 to activate the tunnel/no-tunnel verification.";
}

#endif
