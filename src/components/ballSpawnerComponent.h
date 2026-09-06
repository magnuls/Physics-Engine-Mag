#pragma once
#include <algorithm>
#include <cstddef>
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
// camera aim; R sends every body back to its start. Safe to spawn mid frame:
// Entity walks children by index, so AddChild during ProcessInput is fine.
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
        // Snapshot rest state on the first frame, once all bodies exist.
        if (!m_snapshotted) {
            for (std::size_t i = 0; i < m_engine->GetNumObjects(); ++i)
                m_initial.push_back(m_engine->GetObject(i).GetPosition());
            m_snapshotted = true;
        }

        if (input.GetKeyDown(m_spawnKey)) Spawn();
        if (input.GetKeyDown(m_resetKey)) Reset();
    }

   private:
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
        // Higher angular damping than the scene balls since these land faster.
        ball.SetAngularDamping(2.0f);
        ball.SetLinearDamping(0.05f);
        std::size_t index = m_engine->AddObject(ball);

        // Reuse the held mesh and material so nothing can be evicted.
        m_spawnParent->AddChild(
            (new Entity(pos))
                ->AddComponent(new MeshRenderer(m_ballMesh, m_ballMaterial))
                ->AddComponent(new PhysicsObjectComponent(m_engine, index)));

        // Keep the reset snapshot aligned with the new body's index.
        m_initial.push_back(pos);
    }

    void Reset() {
        const std::size_t n =
            std::min(m_initial.size(), m_engine->GetNumObjects());
        for (std::size_t i = 0; i < n; ++i) {
            Physics::PhysicsObject& o = m_engine->GetObject(i);
            o.SetPosition(m_initial[i]);
            o.SetVelocity(Vector3f(0, 0, 0));
        }
    }

    Physics::PhysicsEngine* m_engine;
    Entity* m_spawnParent;
    const Transform* m_cameraTransform;
    Mesh m_ballMesh;          // held by value so its data is never evicted
    Material m_ballMaterial;  // held by value so its data is never evicted
    float m_radius;
    float m_launchSpeed;
    int m_spawnKey;
    int m_resetKey;

    bool m_snapshotted{false};
    std::vector<Vector3f> m_initial;  // rest position per body index
};
