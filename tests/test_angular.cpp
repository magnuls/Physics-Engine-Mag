#include <gtest/gtest.h>

#include <cmath>

#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// Angular dynamics: bodies gain angular velocity from off centre impulses and
// integrate their orientation over time.

// Orientation advances by integrating a constant angular velocity.
TEST(AngularTest, OrientationIntegratesFromAngularVelocity) {
    PhysicsObject o = PhysicsObject::OrientedBox(
        Vector3f(0, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1));
    o.SetAngularVelocity(Vector3f(0, 0, 1));  // 1 rad/s about +z

    for (int i = 0; i < 1000; ++i)
        o.Integrate(0.001f, Vector3f(0, 0, 0));  // 1 second, no gravity

    // The box's local +x axis has swung ~1 rad about +z.
    Vector3f right = o.GetOrientation().GetRight();
    EXPECT_NEAR(right.GetX(), std::cos(1.0f), 0.02f);
    EXPECT_NEAR(right.GetY(), std::sin(1.0f), 0.02f);
    EXPECT_NEAR(right.GetZ(), 0.0f, 0.02f);
}

// An impulse through the center of mass imparts no spin; an off centre one
// does.
TEST(AngularTest, OffCenterCollisionInducesSpin) {
    // Centered: falling box directly above the static box -> lever arm parallel
    // to the normal -> no torque.
    {
        PhysicsEngine engine;
        engine.AddObject(PhysicsObject::OrientedBox(
            Vector3f(0, 1.5f, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
            Vector3f(0, -4, 0)));
        engine.AddObject(PhysicsObject::OrientedBox(
            Vector3f(0, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
            Vector3f(0, 0, 0), /*invMass=*/0.0f));
        engine.HandleCollisions();
        EXPECT_NEAR(engine.GetObject(0).GetAngularVelocity().Length(), 0.0f,
                    1e-4f);
    }
    // Offset horizontally: contact is off to one side -> the box gains spin.
    {
        PhysicsEngine engine;
        engine.AddObject(PhysicsObject::OrientedBox(
            Vector3f(0, 1.5f, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
            Vector3f(0, -4, 0)));
        engine.AddObject(PhysicsObject::OrientedBox(
            Vector3f(1.0f, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
            Vector3f(0, 0, 0), /*invMass=*/0.0f));
        engine.HandleCollisions();
        Vector3f w = engine.GetObject(0).GetAngularVelocity();
        EXPECT_GT(w.Length(), 0.5f);           // spin was induced
        EXPECT_GT(std::fabs(w.GetZ()), 0.5f);  // about z from the in plane offset
    }
}

// A static body absorbs the collision but never rotates.
TEST(AngularTest, StaticBodyStaysUnrotated) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::OrientedBox(Vector3f(0, 1.5f, 0), Vector3f(1, 1, 1),
                                   Quaternion(0, 0, 0, 1), Vector3f(0, -4, 0)));
    engine.AddObject(PhysicsObject::OrientedBox(
        Vector3f(1.0f, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
        Vector3f(0, 0, 0), /*invMass=*/0.0f));
    engine.HandleCollisions();
    EXPECT_NEAR(engine.GetObject(1).GetAngularVelocity().Length(), 0.0f, 1e-6f);
}

// A frictionless sphere hitting a plane head on gets no spin; the contact lever
// arm is parallel to the normal.
TEST(AngularTest, SphereHeadOnNoSpin) {
    PhysicsEngine engine;
    engine.AddObject(
        PhysicsObject::Sphere(Vector3f(0, 0.5f, 0), 1.0f, Vector3f(0, -5, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));
    engine.HandleCollisions();
    EXPECT_NEAR(engine.GetObject(0).GetAngularVelocity().Length(), 0.0f, 1e-4f);
    EXPECT_GT(engine.GetObject(0).GetVelocity().GetY(), 0.0f);  // still bounces
}
