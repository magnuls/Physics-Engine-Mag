#include <gtest/gtest.h>

#include <cmath>

#include "../src/core/math3d.h"
#include "../src/core/transform.h"

// Headless unit tests for the core math and Transform, no GL or SDL context
// needed. Rendering code that needs a live GL context is out of scope.

namespace {
// Takes the base Vector<float,3> so it binds both Vector3f and the raw
// Vector<float,3> that Matrix4f::Transform returns.
void ExpectVec3(const Vector<float, 3>& v, float x, float y, float z,
                float tol = 1e-4f) {
    EXPECT_NEAR(v[0], x, tol);
    EXPECT_NEAR(v[1], y, tol);
    EXPECT_NEAR(v[2], z, tol);
}
}  // namespace

// Vector3f
TEST(Math3D_Vector3f, DotCrossLength) {
    Vector3f a(1, 2, 3), b(4, 5, 6);
    EXPECT_NEAR(a.Dot(b), 32.0f, 1e-4f);  // 4 + 10 + 18
    ExpectVec3(Vector3f(1, 0, 0).Cross(Vector3f(0, 1, 0)), 0, 0, 1);
    ExpectVec3(Vector3f(0, 1, 0).Cross(Vector3f(1, 0, 0)), 0, 0, -1);
    EXPECT_NEAR(Vector3f(3, 4, 0).Length(), 5.0f, 1e-4f);
    EXPECT_NEAR(Vector3f(1, 2, 2).Length(), 3.0f, 1e-4f);
}

TEST(Math3D_Vector3f, Normalized) {
    Vector3f n = Vector3f(0, 3, 4).Normalized();
    ExpectVec3(n, 0.0f, 0.6f, 0.8f);
    EXPECT_NEAR(n.Length(), 1.0f, 1e-4f);
}

TEST(Math3D_Vector3f, Operators) {
    ExpectVec3(Vector3f(1, 2, 3) + Vector3f(4, 5, 6), 5, 7, 9);
    ExpectVec3(Vector3f(4, 5, 6) - Vector3f(1, 2, 3), 3, 3, 3);
    ExpectVec3(Vector3f(1, 2, 3) * 2.0f, 2, 4, 6);
    ExpectVec3(Vector3f(2, 4, 6) / 2.0f, 1, 2, 3);

    Vector3f v(1, 1, 1);
    v += Vector3f(1, 2, 3);
    ExpectVec3(v, 2, 3, 4);
    v -= Vector3f(0, 1, 2);
    ExpectVec3(v, 2, 2, 2);
    v *= 3.0f;
    ExpectVec3(v, 6, 6, 6);
    v /= 2.0f;
    ExpectVec3(v, 3, 3, 3);

    EXPECT_TRUE(Vector3f(1, 2, 3) == Vector3f(1, 2, 3));
    EXPECT_TRUE(Vector3f(1, 2, 3) != Vector3f(1, 2, 4));
}

TEST(Math3D_Vector3f, RotateAboutAxis) {
    // Rotate 1,0,0 by 90 deg about +Z. The engine's Rotate is left handed with
    // sin of negative angle, so assert the axis plane behaviour it actually has.
    Vector3f r = Vector3f(1, 0, 0).Rotate(ToRadians(90.0f), Vector3f(0, 0, 1));
    EXPECT_NEAR(r.Length(), 1.0f, 1e-4f);
    EXPECT_NEAR(r.GetZ(), 0.0f, 1e-4f);            // stays in the XY plane
    EXPECT_NEAR(std::abs(r.GetY()), 1.0f, 1e-4f);  // maps onto +/- Y
    EXPECT_NEAR(r.GetX(), 0.0f, 1e-4f);
}

// Vector2f
TEST(Math3D_Vector2f, DotCross) {
    Vector2f a(1, 2), b(3, 4);
    EXPECT_NEAR(a.Dot(b), 11.0f, 1e-4f);  // 3 + 8
    EXPECT_NEAR(Vector2f(1, 0).Cross(Vector2f(0, 1)), 1.0f, 1e-4f);
    EXPECT_NEAR(Vector2f(0, 1).Cross(Vector2f(1, 0)), -1.0f, 1e-4f);
}

// Helpers: Clamp / ToRadians / ToDegrees / square
TEST(Math3D_Helpers, ClampRadiansSquare) {
    EXPECT_EQ(Clamp(5, 0, 3), 3);
    EXPECT_EQ(Clamp(-1, 0, 3), 0);
    EXPECT_EQ(Clamp(2, 0, 3), 2);

    EXPECT_NEAR(ToRadians(180.0f), static_cast<float>(MATH_PI), 1e-4f);
    EXPECT_NEAR(ToDegrees(static_cast<float>(MATH_PI)), 180.0f, 1e-3f);

    EXPECT_EQ(square(3), 9);
    EXPECT_NEAR(square(1.5f), 2.25f, 1e-5f);
}

// Matrix4f
TEST(Math3D_Matrix4f, IdentityTransform) {
    Matrix4f id;
    id.InitIdentity();
    // Identity leaves the 3 vector unchanged.
    auto p = id.Transform(Vector3f(1, 2, 3));
    ExpectVec3(p, 1, 2, 3);
}

TEST(Math3D_Matrix4f, TranslationAndScale) {
    Matrix4f t;
    t.InitTranslation(Vector3f(10, 20, 30));
    ExpectVec3(t.Transform(Vector3f(1, 2, 3)), 11, 22, 33);

    Matrix4f s;
    s.InitScale(Vector3f(2, 3, 4));
    ExpectVec3(s.Transform(Vector3f(1, 1, 1)), 2, 3, 4);
}

TEST(Math3D_Matrix4f, IdentityIsMultiplicativeUnit) {
    Matrix4f id, t;
    id.InitIdentity();
    t.InitTranslation(Vector3f(5, -2, 7));
    // I*T == T == T*I, verified through the vector action.
    ExpectVec3((id * t).Transform(Vector3f(0, 0, 0)), 5, -2, 7);
    ExpectVec3((t * id).Transform(Vector3f(0, 0, 0)), 5, -2, 7);
}

// Quaternion
TEST(Math3D_Quaternion, Conjugate) {
    Quaternion q(1, 2, 3, 4);
    Quaternion c = q.Conjugate();
    EXPECT_NEAR(c.GetX(), -1.0f, 1e-4f);
    EXPECT_NEAR(c.GetY(), -2.0f, 1e-4f);
    EXPECT_NEAR(c.GetZ(), -3.0f, 1e-4f);
    EXPECT_NEAR(c.GetW(), 4.0f, 1e-4f);
}

TEST(Math3D_Quaternion, HamiltonProduct_IJequalsK) {
    Quaternion i(1, 0, 0, 0), j(0, 1, 0, 0);
    Quaternion k = i * j;  // i*j = k
    EXPECT_NEAR(k.GetX(), 0.0f, 1e-4f);
    EXPECT_NEAR(k.GetY(), 0.0f, 1e-4f);
    EXPECT_NEAR(k.GetZ(), 1.0f, 1e-4f);
    EXPECT_NEAR(k.GetW(), 0.0f, 1e-4f);
}

TEST(Math3D_Quaternion, AxisAngleConstruction) {
    // 180 deg about +Z gives 0,0,sin(90),cos(90) = 0,0,1,0.
    Quaternion q(Vector3f(0, 0, 1), ToRadians(180.0f));
    EXPECT_NEAR(q.GetX(), 0.0f, 1e-4f);
    EXPECT_NEAR(q.GetY(), 0.0f, 1e-4f);
    EXPECT_NEAR(q.GetZ(), 1.0f, 1e-4f);
    EXPECT_NEAR(q.GetW(), 0.0f, 1e-4f);
}

TEST(Math3D_Quaternion, IdentityBasisVectors) {
    Quaternion idq;  // identity 0,0,0,1
    ExpectVec3(idq.GetForward(), 0, 0, 1);
    ExpectVec3(idq.GetUp(), 0, 1, 0);
    ExpectVec3(idq.GetRight(), 1, 0, 0);
}

// Transform scene graph node, headless, no rendering
TEST(Transform, DefaultsAndSetters) {
    Transform tr;
    // Non const GetPos returns Vector3f*, the const overload returns a ref.
    ExpectVec3(*tr.GetPos(), 0, 0, 0);
    EXPECT_NEAR(tr.GetScale(), 1.0f, 1e-4f);

    tr.SetPos(Vector3f(4, 5, 6));
    ExpectVec3(*tr.GetPos(), 4, 5, 6);
    // No parent, so transformed pos equals local pos under the identity parent.
    ExpectVec3(tr.GetTransformedPos(), 4, 5, 6);
}

TEST(Transform, ParentTranslationComposes) {
    Transform parent(Vector3f(10, 0, 0));
    Transform child(Vector3f(1, 0, 0));
    child.SetParent(&parent);
    // Child's world pos = parent transform applied to the child's local pos.
    ExpectVec3(child.GetTransformedPos(), 11, 0, 0);
}

TEST(Transform, GetTransformationTranslatesPoint) {
    Transform tr(Vector3f(2, 3, 4));  // identity rot, unit scale
    Matrix4f m = tr.GetTransformation();
    ExpectVec3(m.Transform(Vector3f(0, 0, 0)), 2, 3, 4);
    ExpectVec3(m.Transform(Vector3f(1, 1, 1)), 3, 4, 5);
}
