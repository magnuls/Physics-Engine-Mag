#include <gtest/gtest.h>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Sleeping / islands. Master switch defaults off: with sleeping disabled
// nothing changes anywhere.

namespace {

constexpr float kDt = 1.0f / 60.0f;

void step(PhysicsEngine& engine, int n) {
    for (int i = 0; i < n; ++i) {
        engine.Simulate(kDt);
        engine.HandleCollisions();
    }
}

// A ball dropped onto the floor plane; settles within a few seconds at e=0.2.
void buildRestingBall(PhysicsEngine& engine) {
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.2f);
    engine.AddObject(PhysicsObject::Sphere(Vector3f(0, 3, 0), 1.0f));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));
}

}  // namespace

// With sleeping off, a fully settled body still reports awake.
TEST(SleepTest, DisabledByDefaultNothingSleeps) {
    PhysicsEngine engine;
    buildRestingBall(engine);

    step(engine, 600);  // ~10 s, long settled

    EXPECT_TRUE(engine.GetObject(0).IsAwake());
}

// Enabled, a settled body falls asleep (motion frozen, resting height kept).
TEST(SleepTest, SettledBodySleeps) {
    PhysicsEngine engine;
    engine.SetSleepingEnabled(true);
    buildRestingBall(engine);

    step(engine, 600);

    EXPECT_FALSE(engine.GetObject(0).IsAwake());
    EXPECT_NEAR(engine.GetObject(0).GetVelocity().Length(), 0.0f, 1e-6f);
    EXPECT_NEAR(engine.GetObject(0).GetPosition().GetY(), 1.0f, 0.15f);
}

// A sleeping body under gravity does NOT sink or drift: its position is
// bit-frozen while asleep (integration is skipped entirely).
TEST(SleepTest, SleepingBodyDoesNotSink) {
    PhysicsEngine engine;
    engine.SetSleepingEnabled(true);
    buildRestingBall(engine);
    step(engine, 600);
    ASSERT_FALSE(engine.GetObject(0).IsAwake());
    const float yAsleep = engine.GetObject(0).GetPosition().GetY();

    step(engine, 300);  // 5 more seconds of gravity

    EXPECT_FALSE(engine.GetObject(0).IsAwake());
    EXPECT_EQ(engine.GetObject(0).GetPosition().GetY(), yAsleep);
}

// External velocity change wakes a sleeping body and it responds normally.
TEST(SleepTest, SetVelocityWakesAndBodyMoves) {
    PhysicsEngine engine;
    engine.SetSleepingEnabled(true);
    buildRestingBall(engine);
    step(engine, 600);
    ASSERT_FALSE(engine.GetObject(0).IsAwake());

    engine.GetObject(0).SetVelocity(Vector3f(5, 0, 0));
    EXPECT_TRUE(engine.GetObject(0).IsAwake());

    step(engine, 30);                                           // 0.5 s
    EXPECT_GT(engine.GetObject(0).GetPosition().GetX(), 1.0f);  // rolling away
}

// A new collision wakes a sleeping body: a second ball dropped onto the
// sleeper flips it awake on contact.
TEST(SleepTest, NewCollisionWakesSleepingBody) {
    PhysicsEngine engine;
    engine.SetSleepingEnabled(true);
    buildRestingBall(engine);
    step(engine, 600);
    ASSERT_FALSE(engine.GetObject(0).IsAwake());

    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0.5f, 6, 0), 1.0f));  // incoming
    bool wokeUp = false;
    for (int i = 0; i < 120 && !wokeUp; ++i) {  // up to 2 s to fall + touch
        step(engine, 1);
        wokeUp = engine.GetObject(0).IsAwake();
    }
    EXPECT_TRUE(wokeUp);
}

// Islands: two stacked boxes sleep together, and a thrown ball hitting the top
// box wakes both (the wake propagates across the contact island).
TEST(SleepTest, IslandSleepsAndWakesTogether) {
    PhysicsEngine engine;
    engine.SetSleepingEnabled(true);
    engine.SetGravity(Vector3f(0, -9.81f, 0));
    engine.SetRestitution(0.0f);
    PhysicsObject bottom =
        PhysicsObject::Box(Vector3f(-1, 0.02f, -1), Vector3f(1, 2.02f, 1));
    PhysicsObject top =
        PhysicsObject::Box(Vector3f(-1, 2.1f, -1), Vector3f(1, 4.1f, 1));
    bottom.SetFriction(0.5f);
    top.SetFriction(0.5f);
    engine.AddObject(bottom);
    engine.AddObject(top);
    PhysicsObject floor = PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f);
    floor.SetFriction(0.5f);
    engine.AddObject(floor);

    step(engine, 600);  // stack settles, island sleeps as one
    ASSERT_FALSE(engine.GetObject(0).IsAwake());
    ASSERT_FALSE(engine.GetObject(1).IsAwake());

    // Throw a ball at the TOP box only.
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(6, 3, 0), 0.5f, Vector3f(-20, 0, 0)));
    bool bothAwake = false;
    for (int i = 0; i < 60 && !bothAwake; ++i) {
        step(engine, 1);
        bothAwake =
            engine.GetObject(0).IsAwake() && engine.GetObject(1).IsAwake();
    }
    EXPECT_TRUE(bothAwake);  // bottom box woke via the island, not the ball
}
