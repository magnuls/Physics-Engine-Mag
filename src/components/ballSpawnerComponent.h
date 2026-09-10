#pragma once
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "../core/entity.h"
#include "../core/entityComponent.h"
#include "../core/input.h"
#include "../core/transform.h"
#include "../physics/physicsEngine.h"
#include "../physics/physicsObject.h"
#include "../rendering/material.h"
#include "../rendering/mesh.h"
#include "meshRenderer.h"
#include "physicsObjectComponent.h"

// Spawns balls at runtime and resets the scene. SPACE launches a ball from the
// camera aim; R restores the pre-placed bodies to their start snapshot and
// hides every thrown ball. UP/DOWN tune launch speed, Q/E dial spin. Every key
// is latched so one press = one action. Safe to spawn mid frame: Entity walks
// children by index, so AddChild during ProcessInput is fine.
//
// Hiding: the engine has no remove-body API and component bindings are by
// stable index, so a cleared ball is parked far below the world at y -1000
// with zeroed velocities. The floor plane test is double sided by distance,
// so a parked ball never collides; it is re-pinned every frame so gravity can
// never drag it anywhere visible, and its PhysicsObjectComponent moves the
// rendered mesh out of sight along with it.
class BallSpawnerComponent : public EntityComponent {
   public:
    BallSpawnerComponent(Physics::PhysicsEngine* engine, Entity* spawnParent,
                         const Transform* cameraTransform, const Mesh& ballMesh,
                         const Material& ballMaterial, float radius = 1.0f,
                         float launchSpeed = 20.0f,
                         int spawnKey = Input::KEY_SPACE,
                         int resetKey = Input::KEY_R)
        : m_engine(engine),
          m_spawnParent(spawnParent),
          m_cameraTransform(cameraTransform),
          m_ballMesh(ballMesh),
          m_ballMaterial(ballMaterial),
          m_radius(radius),
          m_launchSpeed(launchSpeed),
          m_spawnKey(spawnKey),
          m_resetKey(resetKey) {}

    virtual void ProcessInput(const Input& input, float delta) {
        (void)delta;
        // Snapshot rest state on the first frame, once all pre-placed bodies
        // exist. Runtime balls are tracked separately in m_thrown.
        if (!m_snapshotted) {
            for (std::size_t i = 0; i < m_engine->GetNumObjects(); ++i)
                m_initial.push_back(m_engine->GetObject(i).GetPosition());
            m_snapshotted = true;
        }

        // Keep parked balls pinned under the world every frame.
        PinParked();

        if (Pressed(input, m_spawnKey, m_spawnWasDown)) Spawn();
        if (Pressed(input, m_resetKey, m_resetWasDown)) Reset();

        // Launch speed dial, one step per press.
        if (Pressed(input, Input::KEY_UP, m_upWasDown)) {
            m_launchSpeed = std::min(m_launchSpeed + kSpeedStep, kSpeedMax);
            std::cout << "Launch speed: " << m_launchSpeed << std::endl;
        }
        if (Pressed(input, Input::KEY_DOWN, m_downWasDown)) {
            m_launchSpeed = std::max(m_launchSpeed - kSpeedStep, kSpeedMin);
            std::cout << "Launch speed: " << m_launchSpeed << std::endl;
        }

        // Spin dial: E toward topspin, Q toward backspin, 0 in the middle.
        // Positive = topspin, negative = backspin, rad/s on the spawned ball.
        if (Pressed(input, Input::KEY_E, m_eWasDown)) {
            m_spin = std::min(m_spin + kSpinStep, kSpinMax);
            PrintSpin();
        }
        if (Pressed(input, Input::KEY_Q, m_qWasDown)) {
            m_spin = std::max(m_spin - kSpinStep, -kSpinMax);
            PrintSpin();
        }
    }

   private:
    // One thrown ball: its stable engine index plus its rendered entity.
    struct ThrownBall {
        std::size_t index;
        Entity* entity;
    };

    // Key latch: fires once on the down edge, then waits for release.
    static bool Pressed(const Input& input, int key, bool& wasDown) {
        const bool down = input.GetKey(key);
        const bool fired = down && !wasDown;
        wasDown = down;
        return fired;
    }

    void PrintSpin() const {
        std::cout << "Spin: " << m_spin
                  << (m_spin > 0 ? " (topspin)"
                                 : m_spin < 0 ? " (backspin)" : " (none)")
                  << std::endl;
    }

    void Spawn() {
        const Vector3f pos = m_cameraTransform->GetPos();
        const Vector3f aim = m_cameraTransform->GetRot().GetForward();
        const Vector3f velocity = aim * m_launchSpeed;

        Physics::PhysicsObject ball =
            Physics::PhysicsObject::Sphere(pos, m_radius, velocity);
        // CCD on so fast balls don't tunnel in one step.
        ball.SetContinuous(true);
        // Ball needs its own friction; it combines as sqrt(fA*fB), so 0 slides.
        ball.SetFriction(0.4f);
        // Enough angular damping to tame landing whirl, low enough that
        // Q/E spin survives the flight and visibly bites on the bounce.
        ball.SetAngularDamping(1.0f);
        ball.SetLinearDamping(0.05f);
        // Spin about the horizontal axis perpendicular to the aim. axis =
        // aim x up points screen-right, and topspin needs the ball's top
        // surface moving forward, which is rotation about MINUS that axis.
        if (m_spin != 0.0f) {
            const Vector3f up(0, 1, 0);
            Vector3f axis = aim.Cross(up);
            if (axis.Length() > 1e-4f) {
                axis = axis.Normalized();
                ball.SetAngularVelocity(axis * -m_spin);
            }
        }
        std::size_t index = m_engine->AddObject(ball);

        // Reuse the held mesh and material so nothing can be evicted.
        Entity* e = (new Entity(pos))
                        ->AddComponent(new MeshRenderer(m_ballMesh, m_ballMaterial))
                        ->AddComponent(new PhysicsObjectComponent(m_engine, index));
        m_spawnParent->AddChild(e);

        // Track it so R can clear it later.
        m_thrown.push_back({index, e});
    }

    void Reset() {
        // Pre-placed bodies go back to their first frame snapshot. Setting
        // velocity also wakes a sleeping body, so piles re-settle cleanly.
        const std::size_t n =
            std::min(m_initial.size(), m_engine->GetNumObjects());
        for (std::size_t i = 0; i < n; ++i) {
            Physics::PhysicsObject& o = m_engine->GetObject(i);
            o.SetPosition(m_initial[i]);
            o.SetVelocity(Vector3f(0, 0, 0));
            o.SetAngularVelocity(Vector3f(0, 0, 0));
        }

        // Every thrown ball disappears below the world.
        for (const ThrownBall& b : m_thrown) m_parked.push_back(b);
        m_thrown.clear();
        PinParked();
    }

    // Freeze parked balls at the graveyard: staggered along x so they never
    // overlap each other, CCD off so they pair with nothing, velocities
    // zeroed so gravity has no frame to frame effect worth seeing.
    void PinParked() {
        for (std::size_t i = 0; i < m_parked.size(); ++i) {
            Physics::PhysicsObject& o = m_engine->GetObject(m_parked[i].index);
            o.SetContinuous(false);
            o.SetPosition(Vector3f(3.0f * (float)i, kParkY, 0));
            o.SetVelocity(Vector3f(0, 0, 0));
            o.SetAngularVelocity(Vector3f(0, 0, 0));
        }
    }

    static constexpr float kSpeedStep = 2.0f;
    static constexpr float kSpeedMin = 5.0f;
    static constexpr float kSpeedMax = 40.0f;
    static constexpr float kSpinStep = 4.0f;
    static constexpr float kSpinMax = 16.0f;
    static constexpr float kParkY = -1000.0f;

    Physics::PhysicsEngine* m_engine;
    Entity* m_spawnParent;
    const Transform* m_cameraTransform;
    Mesh m_ballMesh;          // held by value so its data is never evicted
    Material m_ballMaterial;  // held by value so its data is never evicted
    float m_radius;
    float m_launchSpeed;
    int m_spawnKey;
    int m_resetKey;

    float m_spin{0.0f};  // rad/s; + topspin, - backspin
    bool m_spawnWasDown{false};
    bool m_resetWasDown{false};
    bool m_upWasDown{false};
    bool m_downWasDown{false};
    bool m_qWasDown{false};
    bool m_eWasDown{false};

    bool m_snapshotted{false};
    std::vector<Vector3f> m_initial;    // rest position per pre-placed body
    std::vector<ThrownBall> m_thrown;   // live thrown balls, cleared on R
    std::vector<ThrownBall> m_parked;   // hidden below the world after R
};
