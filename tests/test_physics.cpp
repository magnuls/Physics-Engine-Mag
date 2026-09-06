#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "../src/physics/aabb.h"
#include "../src/physics/boundingSphere.h"
#include "../src/physics/broadphase.h"
#include "../src/physics/collisionDispatch.h"
#include "../src/physics/contactPoints.h"
#include "../src/physics/intersectData.h"
#include "../src/physics/plane.h"

using namespace Physics;

TEST(PhysicsTest, BoundingSphere) {
    BoundingSphere sphere1(Vector3f(0, 0, 0), 1);  // red
    BoundingSphere sphere2(Vector3f(1, 1, 1), 1);  // blue
    BoundingSphere sphere3(Vector3f(4, 3, 2), 1);  // purple
    BoundingSphere sphere4(Vector3f(0, 0, 2), 1);  // white
                                                   //
                                                   //
    IntersectData s1_s2{collision(sphere1, sphere2)};
    IntersectData s2_s1{collision(sphere2, sphere1)};
    IntersectData s3_s4{collision(sphere3, sphere4)};
    IntersectData s4_s1{collision(sphere4, sphere1)};

    EXPECT_TRUE(s1_s2.m_doesIntersect);
    EXPECT_TRUE(s2_s1.m_doesIntersect);
    EXPECT_FALSE(s3_s4.m_doesIntersect);
    EXPECT_TRUE(s4_s1.m_doesIntersect);
}

int nthdigit(float& digit, const int n) {
    return std::trunc(std::pow(10, n) * digit);
}

TEST(PhysicsTest, AxisAlignedBoundingBox) {
    AABB box0(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    // This test below will throw a runtime error since second vector has an
    // attribute < first
    //  AABB box1(Vector3f(4, 4, 4), Vector3f(5, 8, 0));
    AABB box2(Vector3f(4, 2, 3), Vector3f(7, 6, 5));
    AABB box3(Vector3f(-399, -493, 233), Vector3f(9, -9, 300));
    AABB box4(Vector3f(0, 0, 0), Vector3f(5, 5, 5));
    AABB box5(Vector3f(3, 2, 4), Vector3f(8, 7, 9));
    AABB box6(Vector3f(10, 10, 10), Vector3f(15, 15, 15));
    AABB box7(Vector3f(0, 0, 0), Vector3f(3, 3, 3));

    // IntersectData b0_b1{collision(box0, box1)};
    IntersectData b0_b2{collision(box0, box2)};
    IntersectData b2_b3{collision(box2, box3)};
    // IntersectData b3_b0{collision(box3, box0)};
    IntersectData b4_b5{collision(box4, box5)};
    IntersectData b6_b7{collision(box6, box7)};

    EXPECT_FALSE(b6_b7.m_doesIntersect);
    EXPECT_NEAR(b6_b7.distance, 12.12, 0.01f);
    EXPECT_FALSE(b0_b2.m_doesIntersect);
    EXPECT_FALSE(b2_b3.m_doesIntersect);
    EXPECT_TRUE(b4_b5.m_doesIntersect);
    EXPECT_NEAR(b0_b2.distance, 3.74, 0.01f);
    EXPECT_NEAR(b2_b3.distance, 228.31, 0.01f);
}

TEST(PhysicsTest, AABB_vs_BS) {
    AABB box(Vector3f(0, 0, 0), Vector3f(5, 5, 5));

    // Sphere center inside the box
    BoundingSphere s_inside(Vector3f(2.5, 2.5, 2.5), 0.5);
    IntersectData r1{collision(box, s_inside)};
    EXPECT_TRUE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 0.0, 0.01f);

    // Sphere overlaps a face
    BoundingSphere s_face(Vector3f(6, 2.5, 2.5), 1.5);
    IntersectData r2{collision(box, s_face)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 1.0, 0.01f);

    BoundingSphere s_tangent(Vector3f(6, 2.5, 2.5), 1.0);
    IntersectData r3{collision(box, s_tangent)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 1.0, 0.01f);

    // Sphere near a corner intersecting
    BoundingSphere s_corner_hit(Vector3f(5.5, 5.5, 5.5), 1.0);
    IntersectData r4{collision(box, s_corner_hit)};
    EXPECT_TRUE(r4.m_doesIntersect);
    EXPECT_NEAR(r4.distance, 0.87, 0.01f);

    // Sphere near a corner  missing
    BoundingSphere s_corner_miss(Vector3f(6, 6, 6), 1.0);
    IntersectData r5{collision(box, s_corner_miss)};
    EXPECT_FALSE(r5.m_doesIntersect);
    EXPECT_NEAR(r5.distance, 1.73, 0.01f);

    // Sphere near an edge missing
    BoundingSphere s_edge_miss(Vector3f(6, 6, 2.5), 1.0);
    IntersectData r6{collision(box, s_edge_miss)};
    EXPECT_FALSE(r6.m_doesIntersect);
    EXPECT_NEAR(r6.distance, 1.41, 0.01f);

    // Sphere far away
    BoundingSphere s_far(Vector3f(10, 10, 10), 1.0);
    IntersectData r7{collision(box, s_far)};
    EXPECT_FALSE(r7.m_doesIntersect);
    EXPECT_NEAR(r7.distance, 8.66, 0.01f);

    IntersectData r8{collision(s_face, box)};
    EXPECT_TRUE(r8.m_doesIntersect);
    EXPECT_NEAR(r8.distance, 1.0, 0.01f);

    IntersectData r9{collision(s_corner_miss, box)};
    EXPECT_FALSE(r9.m_doesIntersect);
    EXPECT_NEAR(r9.distance, 1.73, 0.01f);

    // Sphere approaching from neg side
    AABB box2(Vector3f(-2, -2, -2), Vector3f(2, 2, 2));
    BoundingSphere s_neg(Vector3f(-3, 0, 0), 1.5);
    IntersectData r10{collision(box2, s_neg)};
    EXPECT_TRUE(r10.m_doesIntersect);
    EXPECT_NEAR(r10.distance, 1.0, 0.01f);
}
TEST(PhysicsTest, Plane_Collision_Tests) {
    // 1. Box well above axis-aligned floor, no intersection
    Plane floor(Vector3f(0, 1, 0), 0);
    AABB box_above(Vector3f(0, 5, 0), Vector3f(2, 7, 2));
    // center=(1,6,1), half=(1,1,1), r=1, dist=6
    IntersectData r1{collision(floor, box_above)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 6.0, 0.01f);

    // 2. Box straddling plane, center on plane
    AABB box_straddle(Vector3f(-1, -1, -1), Vector3f(1, 1, 1));
    // center=(0,0,0), r=1, dist=0
    IntersectData r2{collision(floor, box_straddle)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.0, 0.01f);

    // 3. Box tangent, bottom face just touches plane
    AABB box_tangent(Vector3f(-1, 0, -1), Vector3f(1, 2, 1));
    // center=(0,1,0), r=1, dist=1, 1<=1
    IntersectData r3{collision(floor, box_tangent)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 1.0, 0.01f);

    // 4. Near miss, bottom face just above plane
    AABB box_near_miss(Vector3f(-1, 0.1, -1), Vector3f(1, 2.1, 1));
    // center=(0,1.1,0), r=1, dist=1.1, 1.1>1
    IntersectData r4{collision(floor, box_near_miss)};
    EXPECT_FALSE(r4.m_doesIntersect);
    EXPECT_NEAR(r4.distance, 1.1, 0.01f);

    // 5. Diagonal plane through origin, box at origin
    Plane diag(Vector3f(1, 1, 1), 0);
    // normalized norm = (1/√3, 1/√3, 1/√3)
    // r = 3/√3 = √3 ≈ 1.73, dist = 0
    IntersectData r5{collision(diag, box_straddle)};
    EXPECT_TRUE(r5.m_doesIntersect);
    EXPECT_NEAR(r5.distance, 0.0, 0.01f);

    // 6. Diagonal plane, box far away
    AABB box_far(Vector3f(5, 5, 5), Vector3f(7, 7, 7));
    // center=(6,6,6), r=√3≈1.73, dist=6√3≈10.39
    IntersectData r6{collision(diag, box_far)};
    EXPECT_FALSE(r6.m_doesIntersect);
    EXPECT_NEAR(r6.distance, 10.39, 0.01f);

    // 7. Flipped normal, same plane, same result
    Plane floor_flipped(Vector3f(0, -1, 0), 0);
    IntersectData r7{collision(floor_flipped, box_straddle)};
    EXPECT_TRUE(r7.m_doesIntersect);
    EXPECT_NEAR(r7.distance, 0.0, 0.01f);

    // 8. Box below an elevated plane
    Plane elevated(Vector3f(0, 1, 0), 10);
    AABB box_below(Vector3f(0, 0, 0), Vector3f(2, 2, 2));
    // center=(1,1,1), r=1, dist=|1-10|=9
    IntersectData r8{collision(elevated, box_below)};
    EXPECT_FALSE(r8.m_doesIntersect);
    EXPECT_NEAR(r8.distance, 9.0, 0.01f);

    // 9. Non-unit normal, tests normalization
    Plane scaled_floor(Vector3f(0, 3, 0), 0);
    IntersectData r9{collision(scaled_floor, box_tangent)};
    EXPECT_TRUE(r9.m_doesIntersect);
    EXPECT_NEAR(r9.distance, 1.0, 0.01f);

    // 10. Both orderings
    IntersectData r10{collision(box_straddle, floor)};
    EXPECT_TRUE(r10.m_doesIntersect);
    EXPECT_NEAR(r10.distance, 0.0, 0.01f);

    IntersectData r11{collision(box_far, diag)};
    EXPECT_FALSE(r11.m_doesIntersect);
    EXPECT_NEAR(r11.distance, 10.39, 0.01f);
}

TEST(PhysicsTest, Plane_vs_BoundingSphere) {
    // 1. Sphere well above axis-aligned floor, miss
    Plane floor(Vector3f(0, 1, 0), 0);
    BoundingSphere s_above(Vector3f(0, 5, 0), 1.0);
    IntersectData r1{collision(floor, s_above)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 5.0, 0.01f);

    // 2. Sphere partially through floor, hit
    BoundingSphere s_partial(Vector3f(0, 0.5, 0), 1.0);
    IntersectData r2{collision(floor, s_partial)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.5, 0.01f);

    // 3. Sphere tangent to floor, hit
    BoundingSphere s_tangent(Vector3f(0, 1, 0), 1.0);
    IntersectData r3{collision(floor, s_tangent)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 1.0, 0.01f);

    // 4. Near miss, just above
    BoundingSphere s_near(Vector3f(0, 1.5, 0), 1.0);
    IntersectData r4{collision(floor, s_near)};
    EXPECT_FALSE(r4.m_doesIntersect);
    EXPECT_NEAR(r4.distance, 1.5, 0.01f);

    // 5. Center on plane, hit, distance 0
    BoundingSphere s_on(Vector3f(3, 0, 3), 2.0);
    IntersectData r5{collision(floor, s_on)};
    EXPECT_TRUE(r5.m_doesIntersect);
    EXPECT_NEAR(r5.distance, 0.0, 0.01f);

    // 6. Sphere below plane, miss
    BoundingSphere s_below(Vector3f(0, -3, 0), 1.0);
    IntersectData r6{collision(floor, s_below)};
    EXPECT_FALSE(r6.m_doesIntersect);
    EXPECT_NEAR(r6.distance, 3.0, 0.01f);

    // 7. Sphere from negative side, hit
    BoundingSphere s_neg_hit(Vector3f(0, -0.5, 0), 1.0);
    IntersectData r7{collision(floor, s_neg_hit)};
    EXPECT_TRUE(r7.m_doesIntersect);
    EXPECT_NEAR(r7.distance, 0.5, 0.01f);

    // 8. Elevated plane, sphere below it, miss
    Plane elevated(Vector3f(0, 1, 0), 10);
    BoundingSphere s_low(Vector3f(0, 8, 0), 1.0);
    IntersectData r8{collision(elevated, s_low)};
    EXPECT_FALSE(r8.m_doesIntersect);
    EXPECT_NEAR(r8.distance, 2.0, 0.01f);

    // 9. Non-unit normal, tests normalization
    Plane scaled(Vector3f(0, 5, 0), 0);
    IntersectData r9{collision(scaled, s_tangent)};
    EXPECT_TRUE(r9.m_doesIntersect);
    EXPECT_NEAR(r9.distance, 1.0, 0.01f);

    // 10. Both orderings
    IntersectData r10{collision(s_above, floor)};
    EXPECT_FALSE(r10.m_doesIntersect);
    EXPECT_NEAR(r10.distance, 5.0, 0.01f);

    IntersectData r11{collision(s_partial, floor)};
    EXPECT_TRUE(r11.m_doesIntersect);
    EXPECT_NEAR(r11.distance, 0.5, 0.01f);
}

TEST(PhysicsTest, Plane_vs_Plane) {
    // 1. Two parallel planes, same normal, distance 5
    Plane p1(Vector3f(0, 1, 0), 0);
    Plane p2(Vector3f(0, 1, 0), 5);
    IntersectData r1{collision(p1, p2)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 5.0, 0.01f);

    // 2. Same plane, distance 0
    Plane p3(Vector3f(0, 1, 0), 3);
    Plane p4(Vector3f(0, 1, 0), 3);
    IntersectData r2{collision(p3, p4)};
    EXPECT_FALSE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.0, 0.01f);

    // 3. Perpendicular planes, intersect
    Plane p5(Vector3f(0, 1, 0), 0);
    Plane p6(Vector3f(1, 0, 0), 0);
    IntersectData r3{collision(p5, p6)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 0.0, 0.01f);

    // 4. Angled planes, intersect
    Plane p7(Vector3f(0, 1, 0), 0);
    Plane p8(Vector3f(1, 1, 0), 0);
    IntersectData r4{collision(p7, p8)};
    EXPECT_TRUE(r4.m_doesIntersect);
    EXPECT_NEAR(r4.distance, 0.0, 0.01f);

    // 5. Non-unit parallel normals, tests normalization
    Plane p9(Vector3f(0, 3, 0), 0);
    Plane p10(Vector3f(0, 7, 0), 14);
    // After normalization: (0,1,0) d=0 and (0,1,0) d=2
    IntersectData r5{collision(p9, p10)};
    EXPECT_FALSE(r5.m_doesIntersect);
    EXPECT_NEAR(r5.distance, 2.0, 0.01f);

    // 6. Parallel along diagonal
    Plane p11(Vector3f(1, 1, 1), 0);
    Plane p12(Vector3f(1, 1, 1), 3);
    // After normalization: d1=0, d2=3/√3=√3≈1.73
    IntersectData r6{collision(p11, p12)};
    EXPECT_FALSE(r6.m_doesIntersect);
    EXPECT_NEAR(r6.distance, 1.73, 0.01f);

    // 7. Both orderings
    IntersectData r7{collision(p2, p1)};
    EXPECT_FALSE(r7.m_doesIntersect);
    EXPECT_NEAR(r7.distance, 5.0, 0.01f);

    IntersectData r8{collision(p6, p5)};
    EXPECT_TRUE(r8.m_doesIntersect);
    EXPECT_NEAR(r8.distance, 0.0, 0.01f);
}

TEST(PhysicsTest, Plane_vs_AABB) {
    // 1. Box well above axis-aligned floor, miss
    Plane floor(Vector3f(0, 1, 0), 0);
    AABB box_above(Vector3f(0, 5, 0), Vector3f(2, 7, 2));
    // center=(1,6,1), half=(1,1,1), r=1, dist=6
    IntersectData r1{collision(floor, box_above)};
    EXPECT_FALSE(r1.m_doesIntersect);
    EXPECT_NEAR(r1.distance, 6.0, 0.01f);

    // 2. Box straddling plane, center on plane
    AABB box_straddle(Vector3f(-1, -1, -1), Vector3f(1, 1, 1));
    // center=(0,0,0), r=1, dist=0
    IntersectData r2{collision(floor, box_straddle)};
    EXPECT_TRUE(r2.m_doesIntersect);
    EXPECT_NEAR(r2.distance, 0.0, 0.01f);

    // 3. Box tangent, bottom face just touches
    AABB box_tangent(Vector3f(-1, 0, -1), Vector3f(1, 2, 1));
    // center=(0,1,0), r=1, dist=1, 1<=1
    IntersectData r3{collision(floor, box_tangent)};
    EXPECT_TRUE(r3.m_doesIntersect);
    EXPECT_NEAR(r3.distance, 1.0, 0.01f);

    // 4. Near miss, bottom face just above
    AABB box_near_miss(Vector3f(-1, 1.1, -1), Vector3f(1, 3.1, 1));
    // center=(0,2.1,0), r=1, dist=2.1
    IntersectData r4{collision(floor, box_near_miss)};
    EXPECT_FALSE(r4.m_doesIntersect);
    EXPECT_NEAR(r4.distance, 2.1, 0.01f);
}

// Contact manifold tests: collision normal A to B unit length, plus contact
// point on IntersectData, without disturbing distance or hit.
static void EXPECT_VEC_NEAR(const Vector3f& v, float x, float y, float z,
                            float tol = 0.01f) {
    EXPECT_NEAR(v.GetX(), x, tol);
    EXPECT_NEAR(v.GetY(), y, tol);
    EXPECT_NEAR(v.GetZ(), z, tol);
}

TEST(ContactManifold, SphereSphere) {
    BoundingSphere a(Vector3f(0, 0, 0), 1.0);
    BoundingSphere b(Vector3f(2, 0, 0), 1.5);
    IntersectData ab{collision(a, b)};
    EXPECT_TRUE(ab.m_doesIntersect);
    // normal points from A toward B: +x. contact sits on A's surface.
    EXPECT_VEC_NEAR(ab.m_normal, 1, 0, 0);
    EXPECT_VEC_NEAR(ab.m_contactPoint, 1, 0, 0);
    EXPECT_NEAR(ab.m_normal.Length(), 1.0, 0.01f);

    // Swapped order flips the normal.
    IntersectData ba{collision(b, a)};
    EXPECT_VEC_NEAR(ba.m_normal, -1, 0, 0);
}

TEST(ContactManifold, AABBSphereFace) {
    AABB box(Vector3f(0, 0, 0), Vector3f(5, 5, 5));
    BoundingSphere s(Vector3f(6, 2.5, 2.5), 1.5);
    IntersectData r{collision(box, s)};  // A=box, B=sphere
    EXPECT_TRUE(r.m_doesIntersect);
    EXPECT_VEC_NEAR(r.m_normal, 1, 0, 0);  // box -> sphere
    EXPECT_VEC_NEAR(r.m_contactPoint, 5, 2.5, 2.5);

    IntersectData rs{collision(s, box)};  // A=sphere, B=box
    EXPECT_VEC_NEAR(rs.m_normal, -1, 0, 0);
}

TEST(ContactManifold, SphereInsideAABB) {
    AABB box(Vector3f(0, 0, 0), Vector3f(5, 5, 5));
    // Center inside, nearest face is the -x face.
    BoundingSphere s(Vector3f(1, 2.5, 2.5), 0.5);
    IntersectData r{collision(box, s)};
    EXPECT_TRUE(r.m_doesIntersect);
    EXPECT_VEC_NEAR(r.m_normal, -1, 0, 0);
}

TEST(ContactManifold, PlaneSphere) {
    Plane floor(Vector3f(0, 1, 0), 0);
    BoundingSphere s(Vector3f(0, 0.5, 0), 1.0);  // partially through floor
    IntersectData sp{collision(s, floor)};       // A=sphere, B=plane
    EXPECT_TRUE(sp.m_doesIntersect);
    EXPECT_VEC_NEAR(sp.m_normal, 0, -1, 0);       // sphere -> plane (down)
    EXPECT_VEC_NEAR(sp.m_contactPoint, 0, 0, 0);  // projected onto plane

    IntersectData ps{collision(floor, s)};  // A=plane, B=sphere
    EXPECT_VEC_NEAR(ps.m_normal, 0, 1, 0);  // plane -> sphere (up)
}

TEST(ContactManifold, PlaneAABB) {
    Plane floor(Vector3f(0, 1, 0), 0);
    AABB box(Vector3f(-1, -1, -1), Vector3f(1, 1, 1));  // straddles plane
    IntersectData pa{collision(floor, box)};            // A=plane, B=box
    EXPECT_TRUE(pa.m_doesIntersect);
    EXPECT_VEC_NEAR(pa.m_normal, 0, 1, 0);
    // Contact is the box's support point on the surface toward the plane at
    // y=-1, not the box centroid.
    EXPECT_VEC_NEAR(pa.m_contactPoint, 0, -1, 0);

    IntersectData ap{collision(box, floor)};  // A=box, B=plane
    EXPECT_VEC_NEAR(ap.m_normal, 0, -1, 0);
    EXPECT_VEC_NEAR(ap.m_contactPoint, 0, -1,
                    0);  // contact carries through swap
}

// The box vs plane contact point sits on the contact face and tracks the box's
// x/z, so an offset box gives a torque producing lever arm.
TEST(ContactManifold, PlaneAABBSupportPoint) {
    Plane floor(Vector3f(0, 1, 0), 0);

    // Box resting so its bottom face is exactly on the plane -> face center.
    AABB resting(Vector3f(-1, 0, -1), Vector3f(1, 2, 1));  // center (0,1,0)
    EXPECT_VEC_NEAR(collision(floor, resting).m_contactPoint, 0, 0, 0);

    // Box offset in x/z -> support point tracks it (bottom-face center at the
    // box's x/z), so r = contact - center is purely vertical here (flat face)
    // but positioned under the box, not at the world origin.
    AABB offset(Vector3f(4, 0, 6), Vector3f(6, 2, 8));  // center (5,1,7)
    EXPECT_VEC_NEAR(collision(floor, offset).m_contactPoint, 5, 0, 7);
}

// The reusable support-point helpers.
TEST(ContactPoints, SupportHelpers) {
    AABB box(Vector3f(0, 0, 0), Vector3f(2, 4, 6));
    // Toward -y -> bottom face; x,z perpendicular -> face-centered.
    EXPECT_VEC_NEAR(Physics::aabbSupportPoint(box, Vector3f(0, -1, 0)), 1, 0,
                    3);
    // Diagonal dir -> the (max,max,max) corner.
    EXPECT_VEC_NEAR(Physics::aabbSupportPoint(box, Vector3f(1, 1, 1)), 2, 4, 6);

    // Oriented box rotated 90deg about Y maps local +x onto world -z; asking
    // for the support toward world +x picks the local axis now aligned with +x.
    const float s = std::sqrt(2.0f) / 2.0f;  // sin/cos of 45deg
    Quaternion yaw(Vector3f(0, 1, 0), ToRadians(90.0f));
    Vector3f ax = yaw.GetRight();    // local x in world
    Vector3f ay = yaw.GetUp();       // local y in world
    Vector3f az = yaw.GetForward();  // local z in world
    Vector3f p = Physics::obbSupportPoint(Vector3f(0, 0, 0), Vector3f(1, 1, 3),
                                          ax, ay, az, Vector3f(0, 1, 0));
    (void)s;
    EXPECT_NEAR(p.GetY(), 1.0f, 1e-4f);  // +y half-extent along world up
}

// Broad-phase tests: sweep and prune must reproduce the O(N^2) brute force
// candidate set exactly and cull non-overlapping pairs.
static std::vector<std::pair<int, int>> sortedPairs(
    std::vector<std::pair<int, int>> v) {
    std::ranges::sort(v);
    return v;
}

TEST(Broadphase, OverlapPredicate) {
    AABB a(Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    AABB touching(Vector3f(1, 0, 0), Vector3f(2, 1, 1));  // shares a face
    AABB apart(Vector3f(2, 0, 0), Vector3f(3, 1, 1));     // gap on x
    AABB yDisjoint(Vector3f(0, 5, 0), Vector3f(1, 6, 1));

    EXPECT_TRUE(Broadphase::overlaps(a, touching));
    EXPECT_FALSE(Broadphase::overlaps(a, apart));
    EXPECT_FALSE(Broadphase::overlaps(a, yDisjoint));
}

TEST(Broadphase, KnownPairs) {
    std::vector<AABB> boxes{
        AABB(Vector3f(0, 0, 0), Vector3f(1, 1, 1)),        // 0
        AABB(Vector3f(0.5, 0.5, 0.5), Vector3f(2, 2, 2)),  // 1 overlaps 0
        AABB(Vector3f(5, 5, 5), Vector3f(6, 6, 6)),        // 2 isolated
        AABB(Vector3f(1.5, 1.5, 1.5), Vector3f(3, 3, 3)),  // 3 overlaps 1
    };
    std::vector<std::pair<int, int>> expected{{0, 1}, {1, 3}};
    EXPECT_EQ(sortedPairs(Broadphase::sweepAndPrune(boxes)),
              sortedPairs(Broadphase::bruteForcePairs(boxes)));
    EXPECT_EQ(sortedPairs(Broadphase::sweepAndPrune(boxes)), expected);
}

TEST(Broadphase, MatchesBruteForce) {
    std::vector<AABB> boxes;
    // A dense row of unit boxes stepped by 0.5, each overlaps its neighbours.
    for (int i = 0; i < 20; ++i) {
        float x = i * 0.5f;
        boxes.emplace_back(Vector3f(x, 0, 0), Vector3f(x + 1, 1, 1));
    }
    // A box with a wide x-interval but disjoint on y: stays "active" through
    // the whole sweep, so it stresses the 3-axis confirm (must match zero of
    // them).
    boxes.emplace_back(Vector3f(0, 10, 0), Vector3f(30, 11, 1));
    // A separate cluster far along x.
    boxes.emplace_back(Vector3f(100, 0, 0), Vector3f(101, 1, 1));
    boxes.emplace_back(Vector3f(100.5, 0, 0), Vector3f(101.5, 1, 1));

    EXPECT_EQ(sortedPairs(Broadphase::sweepAndPrune(boxes)),
              sortedPairs(Broadphase::bruteForcePairs(boxes)));
}

// 5. Diagonal plane through origin,
/*
Type of Assertions
 EXPECT_EQ(a, b);      // a == b
 EXPECT_NE(a, b);      // a != b
 EXPECT_TRUE(expr);    // expr is true
 EXPECT_FALSE(expr);   // expr is false
 EXPECT_LT(a, b);      // a < b
 EXPECT_GT(a, b);      // a > b
 EXPECT_THROW(expr, ExceptionType);  // expr throws that exception
*/
