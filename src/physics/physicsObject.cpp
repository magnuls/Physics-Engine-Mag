#include "physicsObject.h"

#include <cmath>

namespace Physics {

PhysicsObject PhysicsObject::Sphere(const Vector3f& center, float radius,
                                    const Vector3f& velocity, float invMass) {
    PhysicsObject o;
    o.m_kind = ShapeKind::SPHERE;
    o.m_position = center;
    o.m_velocity = velocity;
    o.m_invMass = invMass;
    o.m_radius = radius;
    o.InitInertia();
    return o;
}

PhysicsObject PhysicsObject::Box(const Vector3f& min, const Vector3f& max,
                                 const Vector3f& velocity, float invMass) {
    PhysicsObject o;
    o.m_kind = ShapeKind::BOX;
    o.m_position = (min + max) / 2.0f;
    o.m_velocity = velocity;
    o.m_invMass = invMass;
    o.m_halfExtents = (max - min) / 2.0f;
    o.InitInertia();
    return o;
}

PhysicsObject PhysicsObject::StaticPlane(const Vector3f& normal, float scaler,
                                         float friction) {
    PhysicsObject o;
    o.m_kind = ShapeKind::PLANE;
    o.m_invMass = 0.0f;  // planes are always immovable
    o.m_planeNormal = normal;
    o.m_planeScaler = scaler;
    o.m_friction = friction;
    o.InitInertia();  // static -> zero inverse inertia
    return o;
}

PhysicsObject PhysicsObject::OrientedBox(const Vector3f& center,
                                         const Vector3f& halfExtents,
                                         const Quaternion& orientation,
                                         const Vector3f& velocity,
                                         float invMass) {
    PhysicsObject o;
    o.m_kind = ShapeKind::OBB;
    o.m_position = center;
    o.m_velocity = velocity;
    o.m_invMass = invMass;
    o.m_halfExtents = halfExtents;
    o.m_orientation = orientation;
    o.InitInertia();
    return o;
}

float PhysicsObject::SupportRadius(const Vector3f& n) const {
    switch (m_kind) {
        case ShapeKind::SPHERE:
            return m_radius;
        case ShapeKind::BOX:
            return m_halfExtents.GetX() * std::abs(n.GetX()) +
                   m_halfExtents.GetY() * std::abs(n.GetY()) +
                   m_halfExtents.GetZ() * std::abs(n.GetZ());
        case ShapeKind::OBB: {
            Vector3f ax{m_orientation.GetRight()};
            Vector3f ay{m_orientation.GetUp()};
            Vector3f az{m_orientation.GetForward()};
            return m_halfExtents.GetX() * std::abs(ax.Dot(n)) +
                   m_halfExtents.GetY() * std::abs(ay.Dot(n)) +
                   m_halfExtents.GetZ() * std::abs(az.Dot(n));
        }
        case ShapeKind::PLANE:
        default:
            return 0.0f;  // plane is static and positionally fixed
    }
}

void PhysicsObject::InitInertia() {
    if (m_invMass == 0.0f) {  // static -> cannot rotate
        m_invInertiaLocal = Vector3f(0, 0, 0);
        return;
    }
    switch (m_kind) {
        case ShapeKind::SPHERE: {
            // Solid sphere: I = (2/5) m r^2  ->  Iinv = 5 * invMass / (2 r^2).
            float s = (m_radius > 0.0f)
                          ? 5.0f * m_invMass / (2.0f * m_radius * m_radius)
                          : 0.0f;
            m_invInertiaLocal = Vector3f(s, s, s);  // isotropic
            break;
        }
        case ShapeKind::OBB: {
            // Solid box: Ixx = (1/3) m (hy^2 + hz^2), so
            // Iinv_x = 3 * invMass / (hy^2 + hz^2), etc.
            float hx = m_halfExtents.GetX(), hy = m_halfExtents.GetY(),
                  hz = m_halfExtents.GetZ();
            m_invInertiaLocal =
                Vector3f(3.0f * m_invMass / (hy * hy + hz * hz),
                         3.0f * m_invMass / (hx * hx + hz * hz),
                         3.0f * m_invMass / (hx * hx + hy * hy));
            break;
        }
        case ShapeKind::BOX:  // axis-aligned box stays unrotated by design
        case ShapeKind::PLANE:
        default:
            m_invInertiaLocal = Vector3f(0, 0, 0);
            break;
    }
}

Vector3f PhysicsObject::ApplyInverseInertia(const Vector3f& worldVec) const {
    // World tensor = R * Iinv_local * R^T. Rotate the vector into the body's
    // local frame, scale by the principal inverse inertia, rotate back. For an
    // isotropic sphere the rotation cancels; for an OBB the tensor tracks its
    // orientation. Zero inverse inertia (static / BOX / plane) yields zero.
    Vector3f local{worldVec.Rotate(m_orientation.Conjugate())};
    Vector3f scaled(local.GetX() * m_invInertiaLocal.GetX(),
                    local.GetY() * m_invInertiaLocal.GetY(),
                    local.GetZ() * m_invInertiaLocal.GetZ());
    return scaled.Rotate(m_orientation);
}

void PhysicsObject::Integrate(float delta, const Vector3f& gravity) {
    if (IsStatic()) return;
    // Linear: semi-implicit Euler. Gravity acts at the center of mass, so it
    // applies no torque. Damping (DAMP-1) right after the velocity update, in
    // the Bullet form 1/(1+d*dt) — unconditionally stable (never negates the
    // velocity however large d*dt gets), and an exact no-op at d=0.
    m_velocity += gravity * delta;
    m_velocity *= 1.0f / (1.0f + m_linearDamping * delta);
    m_angularVelocity *= 1.0f / (1.0f + m_angularDamping * delta);
    m_position += m_velocity * delta;

    // Angular: integrate the orientation from the angular velocity via the
    // quaternion rate  dq/dt = 0.5 * (omega as a pure quaternion) * q, then
    // renormalize to keep it a unit rotation.
    const Vector3f& w = m_angularVelocity;
    if (w.GetX() != 0.0f || w.GetY() != 0.0f || w.GetZ() != 0.0f) {
        Quaternion wq(w.GetX(), w.GetY(), w.GetZ(), 0.0f);
        Quaternion dq{wq * m_orientation};
        float qx = m_orientation.GetX() + 0.5f * dq.GetX() * delta;
        float qy = m_orientation.GetY() + 0.5f * dq.GetY() * delta;
        float qz = m_orientation.GetZ() + 0.5f * dq.GetZ() * delta;
        float qw = m_orientation.GetW() + 0.5f * dq.GetW() * delta;
        float len = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (len > 0.0f)
            m_orientation = Quaternion(qx / len, qy / len, qz / len, qw / len);
    }
}

WorldShape PhysicsObject::GetWorldShape() const {
    switch (m_kind) {
        case ShapeKind::SPHERE:
            return WorldShape(BoundingSphere(m_position, m_radius));
        case ShapeKind::BOX:
            return WorldShape(
                AABB(m_position - m_halfExtents, m_position + m_halfExtents));
        case ShapeKind::OBB:
            return WorldShape(OBB(m_position, m_halfExtents, m_orientation));
        case ShapeKind::PLANE:
        default:
            return WorldShape(Plane(m_planeNormal, m_planeScaler));
    }
}

AABB PhysicsObject::GetWorldAABB() const {
    switch (m_kind) {
        case ShapeKind::SPHERE: {
            Vector3f r(m_radius, m_radius, m_radius);
            return AABB(m_position - r, m_position + r);
        }
        case ShapeKind::BOX:
            return AABB(m_position - m_halfExtents, m_position + m_halfExtents);
        case ShapeKind::OBB: {
            // World bound of a rotated box: project its half-extents onto each
            // world axis via the absolute values of its local axes' components.
            Vector3f ax{m_orientation.GetRight()};
            Vector3f ay{m_orientation.GetUp()};
            Vector3f az{m_orientation.GetForward()};
            Vector3f ext(m_halfExtents.GetX() * std::abs(ax.GetX()) +
                             m_halfExtents.GetY() * std::abs(ay.GetX()) +
                             m_halfExtents.GetZ() * std::abs(az.GetX()),
                         m_halfExtents.GetX() * std::abs(ax.GetY()) +
                             m_halfExtents.GetY() * std::abs(ay.GetY()) +
                             m_halfExtents.GetZ() * std::abs(az.GetY()),
                         m_halfExtents.GetX() * std::abs(ax.GetZ()) +
                             m_halfExtents.GetY() * std::abs(ay.GetZ()) +
                             m_halfExtents.GetZ() * std::abs(az.GetZ()));
            return AABB(m_position - ext, m_position + ext);
        }
        case ShapeKind::PLANE:
        default: {
            // A plane is unbounded; give it a large finite bound so the broad
            // phase always pairs it with finite bodies (conservative).
            const float k = 1.0e6f;
            return AABB(Vector3f(-k, -k, -k), Vector3f(k, k, k));
        }
    }
}

}  // namespace Physics
