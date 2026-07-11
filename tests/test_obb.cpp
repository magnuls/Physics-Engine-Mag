#include <gtest/gtest.h>

#include "../src/physics/obb.h"
#include "../src/physics/obbCollision.h"
#include "../src/physics/physicsEngine.h"
#include "../src/physics/physicsObject.h"

using namespace Physics;

// A quaternion rotating `deg` degrees about the +Z axis.
static Quaternion rotZ(float deg) {
    return Quaternion(Vector3f(0, 0, 1), ToRadians(deg));
}

// --- OBB vs OBB (SAT) --------------------------------------------------------

// With identity orientation an OBB behaves exactly like an AABB.
TEST(OBBTest, AxisAlignedMissAndHit) {
    OBB a(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    OBB far(Vector3f(3, 0, 0), Vector3f(1, 1, 1));      // gap 3-(1+1)=1
    OBB near(Vector3f(1.5f, 0, 0), Vector3f(1, 1, 1));  // pen 2-1.5=0.5

    IntersectData miss{collision<OBB, OBB>(a, far)};
    EXPECT_FALSE(miss.m_doesIntersect);
    EXPECT_NEAR(miss.distance, 1.0f, 1e-3f);
    EXPECT_NEAR(miss.m_normal.GetX(), 1.0f, 1e-3f);  // points a -> b (+x)

    IntersectData hit{collision<OBB, OBB>(a, near)};
    EXPECT_TRUE(hit.m_doesIntersect);
    EXPECT_NEAR(hit.distance, 0.5f, 1e-3f);  // penetration depth
    EXPECT_NEAR(hit.m_normal.GetX(), 1.0f, 1e-3f);
}

// A box rotated 45 deg about Z reaches sqrt(2) ~ 1.414 along X, so orientation
// changes the result — this is what makes it an OBB and not an AABB.
TEST(OBBTest, RotatedReachChangesOverlap) {
    OBB a(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    // Rotated box's X-reach is ~1.414. At center x=3 it is a clear miss...
    OBB rMiss(Vector3f(3, 0, 0), Vector3f(1, 1, 1), rotZ(45));
    IntersectData miss{collision<OBB, OBB>(a, rMiss)};
    EXPECT_FALSE(miss.m_doesIntersect);
    EXPECT_NEAR(miss.distance, 3.0f - (1.0f + 1.41421f), 0.02f);

    // ...but pulled in to x=2.3 the diagonal corner pokes into `a` (hit),
    // whereas an axis-aligned box (reach 1) at x=2.3 would still miss.
    OBB rHit(Vector3f(2.3f, 0, 0), Vector3f(1, 1, 1), rotZ(45));
    IntersectData hit{collision<OBB, OBB>(a, rHit)};
    EXPECT_TRUE(hit.m_doesIntersect);
    EXPECT_NEAR(hit.distance, (1.0f + 1.41421f) - 2.3f, 0.02f);
    EXPECT_NEAR(hit.m_normal.GetX(), 1.0f, 0.02f);
}

// --- OBB vs Sphere -----------------------------------------------------------

TEST(OBBTest, SphereClosestPoint) {
    OBB box(Vector3f(0, 0, 0), Vector3f(1, 1, 1));

    BoundingSphere miss(Vector3f(2, 0, 0), 0.5);  // closest face at x=1, gap 1
    IntersectData r1{collision<OBB, BoundingSphere>(box, miss)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 1.0f, 1e-3f);
    EXPECT_NEAR(r1.m_normal.GetX(), 1.0f, 1e-3f);  // box -> sphere

    BoundingSphere hit(Vector3f(1.3f, 0, 0), 0.5);
    IntersectData r2{collision<OBB, BoundingSphere>(box, hit)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.3f, 1e-3f);

    // Rotated box: its +x corner sits at ~ (1.414, 0, 0); a sphere at x=1.6
    // r0.3 just reaches it.
    OBB rot(Vector3f(0, 0, 0), Vector3f(1, 1, 1), rotZ(45));
    BoundingSphere s(Vector3f(1.6f, 0, 0), 0.3);
    IntersectData r3{collision<OBB, BoundingSphere>(rot, s)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 1.6f - 1.41421f, 0.02f);
}

// Swapped ordering flips the normal (A->B convention preserved).
TEST(OBBTest, SphereSwapFlipsNormal) {
    OBB box(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    BoundingSphere s(Vector3f(1.3f, 0, 0), 0.5);

    IntersectData ob{collision<OBB, BoundingSphere>(box, s)};
    IntersectData bo{collision<BoundingSphere, OBB>(s, box)};
    EXPECT_EQ(ob.m_doesIntersect, bo.m_doesIntersect);
    EXPECT_NEAR(ob.m_normal.GetX(), -bo.m_normal.GetX(), 1e-3f);
}

// --- OBB vs Plane ------------------------------------------------------------

TEST(OBBTest, PlaneProjectedRadius) {
    Plane floor(Vector3f(0, 1, 0), 0);

    OBB above(Vector3f(0, 3, 0), Vector3f(1, 1, 1));  // r=1, dist=3 -> miss
    IntersectData r1{collision<OBB, Plane>(above, floor)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 3.0f, 1e-3f);
    EXPECT_NEAR(r1.m_normal.GetY(), -1.0f, 1e-3f);  // box -> plane (down)

    OBB touching(Vector3f(0, 0.5f, 0), Vector3f(1, 1, 1));  // dist .5 <= r 1
    IntersectData r2{collision<OBB, Plane>(touching, floor)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.5f, 1e-3f);

    // Rotated 45 about Z: projected radius onto the floor normal grows to
    // ~1.414, so a box whose center is 1.3 above the floor now touches it.
    OBB rot(Vector3f(0, 1.3f, 0), Vector3f(1, 1, 1), rotZ(45));
    IntersectData r3{collision<OBB, Plane>(rot, floor)};
    EXPECT_TRUE(r3.m_doesIntersect);  // 1.3 <= 1.414
}

// --- OBB vs AABB -------------------------------------------------------------

TEST(OBBTest, VersusAABB) {
    OBB obb(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    AABB box(Vector3f(1.5f, -1, -1), Vector3f(3.5f, 1, 1));  // center (2.5,0,0)
    IntersectData r{collision<OBB, AABB>(obb, box)};
    // gap along x = 2.5 - (1 + 1) = 0.5 -> miss
    EXPECT_FALSE(r.m_doesIntersect);
    EXPECT_NEAR(r.distance, 0.5f, 1e-3f);
}

// --- Simulation (droppable OBBs) --------------------------------------------

// An oriented box falling onto the static floor bounces (velocity reflected).
TEST(OBBTest, DropsAndBouncesOffPlane) {
    PhysicsEngine engine;
    engine.AddObject(PhysicsObject::OrientedBox(
        Vector3f(0, 0.5f, 0), Vector3f(1, 1, 1), rotZ(30), Vector3f(0, -5, 0)));
    engine.AddObject(PhysicsObject::StaticPlane(Vector3f(0, 1, 0), 0.0f));

    engine.HandleCollisions();
    EXPECT_GT(engine.GetObject(0).GetVelocity().GetY(), 0.0f);  // bounced up
}

// A dynamic box dropped onto a static box bounces off it (OBB-vs-OBB in the
// full broadphase -> narrowphase -> response pipeline).
TEST(OBBTest, StacksOnStaticBox) {
    PhysicsEngine engine;
    engine.AddObject(PhysicsObject::OrientedBox(
        Vector3f(0, 1.5f, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
        Vector3f(0, -4, 0)));  // falling box
    engine.AddObject(PhysicsObject::OrientedBox(
        Vector3f(0, 0, 0), Vector3f(1, 1, 1), Quaternion(0, 0, 0, 1),
        Vector3f(0, 0, 0), /*invMass=*/0.0f));  // static base

    engine.HandleCollisions();
    EXPECT_GT(engine.GetObject(0).GetVelocity().GetY(), 0.0f);  // top rebounds
    EXPECT_NEAR(engine.GetObject(1).GetVelocity().GetY(), 0.0f,
                1e-6f);  // base still
}
