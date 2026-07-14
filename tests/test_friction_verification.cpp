#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Extra checks for the friction + sequential-impulse solver: solver-safety
// invariants and the warm-start / restitution interplay. Public API only.

namespace {

PhysicsObject floorPlane(float mu = 0.0f) {
    PhysicsObject f = PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f);
    f.SetFriction(mu);
    return f;
}

void step(PhysicsEngine& e, int n, float dt = 1.0f / 60.0f) {
    for (int i = 0; i < n; ++i) {
        e.Simulate(dt);
        e.HandleCollisions();
    }
}

}  // namespace

// Warm-start / restitution safety: the solver must never inject energy. An
// elastic (e=1) ball bouncing on the floor must never rebound higher than it
// started — stale warm-start impulses self-correct, they don't accumulate.
TEST(FrictionVerify, ElasticBounceNeverGainsEnergy) {
    PhysicsEngine engine;
    engine.SetRestitution(1.0f);  // perfectly elastic — worst case for energy gain
    const float startY = 4.0f;
    engine.AddObject(PhysicsObject::Sphere(Vector3f(0, startY, 0), 1.0f));
    engine.AddObject(floorPlane(0.0f));

    float maxY = startY;
    float maxYAfterContact = 0.0f;
    bool contacted = false;
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {  // ~10 s, many bounces
        engine.Simulate(dt);
        engine.HandleCollisions();
        float y = engine.GetObject(0).GetPosition().GetY();
        if (y <= 1.05f) contacted = true;  // near the floor (center y ~ radius)
        maxY = std::max(maxY, y);
        if (contacted) maxYAfterContact = std::max(maxYAfterContact, y);
        ASSERT_FALSE(std::isnan(y)) << "position went NaN at step " << i;
    }
    // No energy gain: never rises meaningfully above where it began.
    EXPECT_LT(maxY, startY + 0.2f);
    // But it did bounce back up substantially — rules out the trivial "it just
    // stuck to the floor".
    EXPECT_GT(maxYAfterContact, 3.0f);
}

// Restitution threshold: approach speeds below 0.5 m/s are treated as e=0 to
// kill endless micro-bouncing. A ball released just above the floor settles, a
// fast one still bounces.
TEST(FrictionVerify, RestitutionThresholdSettlesSlowContacts) {
    // Slow: dropped from ~2 cm up (impact << 0.5 m/s over the first steps) with
    // full elasticity set — the threshold must still bring it to rest.
    PhysicsEngine slow;
    slow.SetRestitution(1.0f);
    slow.AddObject(PhysicsObject::Sphere(Vector3f(0, 1.02f, 0), 1.0f));
    slow.AddObject(floorPlane(0.0f));
    step(slow, 300);  // 5 s
    EXPECT_NEAR(slow.GetObject(0).GetPosition().GetY(), 1.0f, 0.02f);
    EXPECT_LT(std::fabs(slow.GetObject(0).GetVelocity().GetY()), 0.05f);

    // Fast: a real impact (>0.5 m/s) with e=1 must rebound upward.
    PhysicsEngine fast;
    fast.SetRestitution(1.0f);
    fast.SetGravity(Vector3f(0, 0, 0));  // isolate the bounce
    fast.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 1.0f, 0), 1.0f, Vector3f(0, -4, 0)));
    fast.AddObject(floorPlane(0.0f));
    fast.HandleCollisions();
    EXPECT_GT(fast.GetObject(0).GetVelocity().GetY(), 3.0f);  // ~+4 elastic
}

// A resting box neither sinks through the floor (penetration stays within the
// solver's slop) nor creeps, over a long run.
TEST(FrictionVerify, RestingBoxDoesNotSinkOrCreep) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    PhysicsObject box =
        PhysicsObject::Box(Vector3f(-1, 0.01f, -1), Vector3f(1, 2.01f, 1));
    box.SetFriction(0.5f);
    engine.AddObject(box);
    engine.AddObject(floorPlane(0.5f));

    step(engine, 300);  // let it settle
    float settled = engine.GetObject(0).GetPosition().GetY();
    // Bottom face should sit at the floor: center y ~ 1, within slop (5 mm).
    EXPECT_NEAR(settled, 1.0f, 0.01f);

    step(engine, 300);  // 5 s more — must not drift
    float later = engine.GetObject(0).GetPosition().GetY();
    EXPECT_NEAR(later, settled, 1e-3f);  // no creep up or sink down
    EXPECT_LT(std::fabs(engine.GetObject(0).GetVelocity().GetY()), 1e-2f);
}

// Three boxes stress the solver's convergence more than two — all three settle
// at their resting heights and stop moving.
TEST(FrictionVerify, ThreeBoxStackSettles) {
    PhysicsEngine engine;
    engine.SetRestitution(0.0f);
    for (int k = 0; k < 3; ++k) {
        float base = 0.05f + k * 2.05f;  // small gaps so they drop together
        PhysicsObject b = PhysicsObject::Box(Vector3f(-1, base, -1),
                                             Vector3f(1, base + 2, 1));
        b.SetFriction(0.6f);
        engine.AddObject(b);
    }
    engine.AddObject(floorPlane(0.6f));

    step(engine, 900);  // 15 s
    EXPECT_NEAR(engine.GetObject(0).GetPosition().GetY(), 1.0f, 0.05f);
    EXPECT_NEAR(engine.GetObject(1).GetPosition().GetY(), 3.0f, 0.10f);
    EXPECT_NEAR(engine.GetObject(2).GetPosition().GetY(), 5.0f, 0.20f);
    for (int i = 0; i < 3; ++i)
        EXPECT_LT(std::fabs(engine.GetObject(i).GetVelocity().GetY()), 0.1f)
            << "box " << i << " still moving";
}

// A messy overlapping pile with friction and restitution stays bounded — no
// NaN, no absurd velocities, nothing launched or tunnelled below the floor.
TEST(FrictionVerify, OverlappingPileStaysBounded) {
    PhysicsEngine engine;
    engine.SetRestitution(0.3f);
    // Deliberately overlapping spheres and boxes jammed together above the
    // floor.
    for (int k = 0; k < 5; ++k) {
        PhysicsObject s = PhysicsObject::Sphere(
            Vector3f(0.3f * k, 2.0f + 0.7f * k, 0.0f), 1.0f);
        s.SetFriction(0.4f);
        engine.AddObject(s);
    }
    engine.AddObject(floorPlane(0.4f));

    step(engine, 300);
    for (std::size_t i = 0; i < engine.GetNumObjects(); ++i) {
        const PhysicsObject& o = engine.GetObject(i);
        const Vector3f p = o.GetPosition();
        const Vector3f v = o.GetVelocity();
        ASSERT_FALSE(std::isnan(p.GetY()) || std::isnan(v.GetY()));
        EXPECT_LT(v.Length(), 30.0f) << "body " << i << " launched";
        EXPECT_GT(p.GetY(), -2.0f) << "body " << i << " tunnelled the floor";
    }
}

// The solver (incl. the index-keyed warm-start cache) is deterministic —
// identical setups produce bit-identical trajectories.
TEST(FrictionVerify, SolverIsDeterministic) {
    auto build = [](PhysicsEngine& e) {
        e.SetRestitution(0.3f);
        for (int k = 0; k < 4; ++k) {
            PhysicsObject b =
                PhysicsObject::Box(Vector3f(-1 + 0.1f * k, 0.5f + 2.1f * k, -1),
                                   Vector3f(1 + 0.1f * k, 2.5f + 2.1f * k, 1),
                                   Vector3f(0.5f * k, 0, 0));
            b.SetFriction(0.5f);
            e.AddObject(b);
        }
        e.AddObject(floorPlane(0.5f));
    };

    PhysicsEngine a, b;
    build(a);
    build(b);
    step(a, 200);
    step(b, 200);

    for (std::size_t i = 0; i < a.GetNumObjects(); ++i) {
        EXPECT_FLOAT_EQ(a.GetObject(i).GetPosition().GetX(),
                        b.GetObject(i).GetPosition().GetX());
        EXPECT_FLOAT_EQ(a.GetObject(i).GetPosition().GetY(),
                        b.GetObject(i).GetPosition().GetY());
        EXPECT_FLOAT_EQ(a.GetObject(i).GetPosition().GetZ(),
                        b.GetObject(i).GetPosition().GetZ());
    }
}
