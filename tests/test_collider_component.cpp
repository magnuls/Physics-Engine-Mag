// Tests for colliders-as-components: a collider attached to an Entity must
// follow that entity's Transform (translation, scale, rotation) instead of
// being a static shape. These exercise the new Physics::*Collider components.

#include <gtest/gtest.h>

#include "../src/core/entity.h"
#include "../src/core/math3d.h"
#include "../src/core/transform.h"
#include "../src/physics/colliderComponent.h"

using Physics::AABBCollider;
using Physics::BoundingSphere;
using Physics::OBBCollider;
using Physics::PlaneCollider;
using Physics::SphereCollider;

// A sphere collider's world center tracks its entity's position, and its
// radius scales with the entity's scale.
TEST(ColliderComponentTest, SphereFollowsTransform) {
    Entity entity(Vector3f(5, 0, 0));
    SphereCollider* sphere = new SphereCollider(2.0f);
    entity.AddComponent(sphere);  // entity takes ownership

    BoundingSphere s0 = sphere->getWorldSphere();
    EXPECT_NEAR(s0.getCenter().GetX(), 5.0f, 0.01f);
    EXPECT_NEAR(s0.getCenter().GetY(), 0.0f, 0.01f);
    EXPECT_NEAR(s0.getCenter().GetZ(), 0.0f, 0.01f);
    EXPECT_NEAR(s0.getRadius(), 2.0f, 0.01f);

    // Move + scale the entity; the collider must move with it.
    entity.GetTransform()->SetPos(Vector3f(10, 1, -2));
    entity.GetTransform()->SetScale(3.0f);

    BoundingSphere s1 = sphere->getWorldSphere();
    EXPECT_NEAR(s1.getCenter().GetX(), 10.0f, 0.01f);
    EXPECT_NEAR(s1.getCenter().GetY(), 1.0f, 0.01f);
    EXPECT_NEAR(s1.getCenter().GetZ(), -2.0f, 0.01f);
    EXPECT_NEAR(s1.getRadius(), 6.0f, 0.01f);  // 2 * scale 3
}

// A local sphere offset is placed relative to the entity origin.
TEST(ColliderComponentTest, SphereHonorsLocalOffset) {
    Entity entity(Vector3f(0, 0, 0));
    SphereCollider* sphere = new SphereCollider(1.0f, Vector3f(3, 0, 0));
    entity.AddComponent(sphere);

    entity.GetTransform()->SetPos(Vector3f(0, 10, 0));
    BoundingSphere s = sphere->getWorldSphere();
    EXPECT_NEAR(s.getCenter().GetX(), 3.0f, 0.01f);
    EXPECT_NEAR(s.getCenter().GetY(), 10.0f, 0.01f);
}

// An AABB collider translates and scales with its entity while remaining
// axis-aligned.
TEST(ColliderComponentTest, AABBFollowsTransform) {
    Entity entity(Vector3f(10, 0, 0));
    AABBCollider* box =
        new AABBCollider(Vector3f(-1, -1, -1), Vector3f(1, 1, 1));
    entity.AddComponent(box);

    Physics::AABB b0 = box->getWorldAABB();
    EXPECT_NEAR(b0.getMin().GetX(), 9.0f, 0.01f);
    EXPECT_NEAR(b0.getMax().GetX(), 11.0f, 0.01f);
    EXPECT_NEAR(b0.getMin().GetY(), -1.0f, 0.01f);
    EXPECT_NEAR(b0.getMax().GetY(), 1.0f, 0.01f);

    // Uniform scale doubles the half-extents around the (moved) center.
    entity.GetTransform()->SetPos(Vector3f(0, 0, 0));
    entity.GetTransform()->SetScale(2.0f);
    Physics::AABB b1 = box->getWorldAABB();
    EXPECT_NEAR(b1.getMin().GetX(), -2.0f, 0.01f);
    EXPECT_NEAR(b1.getMax().GetX(), 2.0f, 0.01f);
    EXPECT_NEAR(b1.getMin().GetZ(), -2.0f, 0.01f);
    EXPECT_NEAR(b1.getMax().GetZ(), 2.0f, 0.01f);
}

// A 90-degree rotation about Z keeps the box axis-aligned (refit) and swaps
// which local extent maps to which world axis; extents stay symmetric here.
TEST(ColliderComponentTest, AABBRefitsUnderRotation) {
    Entity entity(Vector3f(0, 0, 0));
    AABBCollider* box =
        new AABBCollider(Vector3f(-2, -1, -1), Vector3f(2, 1, 1));
    entity.AddComponent(box);

    entity.GetTransform()->Rotate(Vector3f(0, 0, 1), ToRadians(90.0f));
    Physics::AABB b = box->getWorldAABB();
    // The length-4 (x) extent rotates onto the world Y axis and vice-versa.
    EXPECT_NEAR(b.getMax().GetX(), 1.0f, 0.01f);
    EXPECT_NEAR(b.getMax().GetY(), 2.0f, 0.01f);
    EXPECT_NEAR(b.getMin().GetX(), -1.0f, 0.01f);
    EXPECT_NEAR(b.getMin().GetY(), -2.0f, 0.01f);
}

// A plane collider translates along its normal with the entity and keeps a
// unit normal after rotation.
TEST(ColliderComponentTest, PlaneFollowsTransform) {
    Entity entity(Vector3f(0, 0, 0));
    // Local plane: normal +Y through the origin.
    PlaneCollider* plane = new PlaneCollider(Vector3f(0, 1, 0), 0.0f);
    entity.AddComponent(plane);

    entity.GetTransform()->SetPos(Vector3f(0, 5, 0));
    Physics::Plane p = plane->getWorldPlane();
    EXPECT_NEAR(p.getNorm().GetY(), 1.0f, 0.01f);
    // Plane now sits at y = 5, so signed distance to origin is 5.
    EXPECT_NEAR(p.getScaler(), 5.0f, 0.01f);

    // Normal must remain unit length under rotation.
    entity.GetTransform()->Rotate(Vector3f(1, 0, 0), ToRadians(90.0f));
    Physics::Plane pr = plane->getWorldPlane();
    float len = pr.getNorm().Length();
    EXPECT_NEAR(len, 1.0f, 0.01f);
}

// An OBB collider's center/half-extents follow the entity's position and scale.
TEST(ColliderComponentTest, OBBFollowsTransformAndScale) {
    Entity entity(Vector3f(5, 0, 0));
    OBBCollider* box = new OBBCollider(Vector3f(1, 2, 3));
    entity.AddComponent(box);

    Physics::OBB o0 = box->getWorldOBB();
    EXPECT_NEAR(o0.getCenter().GetX(), 5.0f, 0.01f);
    EXPECT_NEAR(o0.getHalfExtents().GetX(), 1.0f, 0.01f);
    EXPECT_NEAR(o0.getHalfExtents().GetY(), 2.0f, 0.01f);
    EXPECT_NEAR(o0.getHalfExtents().GetZ(), 3.0f, 0.01f);
    // Identity orientation => the box's local +x axis is world +x.
    EXPECT_NEAR(o0.axisX().GetX(), 1.0f, 0.01f);

    entity.GetTransform()->SetPos(Vector3f(0, 10, 0));
    entity.GetTransform()->SetScale(2.0f);
    Physics::OBB o1 = box->getWorldOBB();
    EXPECT_NEAR(o1.getCenter().GetY(), 10.0f, 0.01f);
    EXPECT_NEAR(o1.getHalfExtents().GetX(), 2.0f, 0.01f);  // 1 * scale 2
    EXPECT_NEAR(o1.getHalfExtents().GetZ(), 6.0f, 0.01f);  // 3 * scale 2
}

// Unlike AABBCollider (which refits axis-aligned), the OBB actually rotates:
// a 90-degree turn about Z carries its local +x axis onto the world Y axis.
TEST(ColliderComponentTest, OBBOrientationTracksRotation) {
    Entity entity(Vector3f(0, 0, 0));
    OBBCollider* box = new OBBCollider(Vector3f(1, 1, 1));
    entity.AddComponent(box);

    entity.GetTransform()->Rotate(Vector3f(0, 0, 1), ToRadians(90.0f));
    Vector3f ax = box->getWorldOBB().axisX();
    EXPECT_NEAR(ax.Length(), 1.0f, 0.01f);  // still a unit axis
    EXPECT_NEAR(ax.GetX(), 0.0f, 0.01f);     // rotated off world +x...
    EXPECT_NEAR(ax.GetZ(), 0.0f, 0.01f);     // ...onto the world Y axis (+/-Y)
}
