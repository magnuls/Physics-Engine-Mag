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

// V2 INTERACTIVE CONTROLS (Agent 3): drop/spawn balls at runtime + reset.
//
// Attach this to a scene entity that is ALREADY wired into the graph (so the
// CoreEngine pointer has been propagated to it). On the spawn key it creates a
// new sphere: a Physics::PhysicsObject added to the shared PhysicsEngine and
// LAUNCHED from the camera along its aim, plus a rendered Entity (MeshRenderer +
// PhysicsObjectComponent) AddChild'd under m_spawnParent so it falls and
// collides exactly like the pre-placed objects. The reset key restores every
// body to a captured rest snapshot (initial positions + zero velocity).
//
// Runtime spawning is safe here: Entity walks its children by index
// (for i < size()), so appending during ProcessInput doesn't invalidate the
// traversal, and Entity::AddChild propagates SetEngine/AddToEngine to the new
// entity's components. The body is referenced by its stable engine index, which
// survives the engine growing its internal vector.
//
// Reset note: PhysicsEngine has no remove-body API (and that file is Agent 1's),
// so reset does NOT delete spawned balls — it sends every body (pre-placed and
// spawned) back to where it started and zeroes its velocity, so the whole scene
// falls again. Indices stay aligned because this component is the only thing
// that adds bodies at runtime.
//
// Material/mesh ownership (fixes the "bricks not initialized" eviction crash):
// Mesh and Material data live in static resource maps that are ERASED when the
// last reference drops, and Material("name")/Mesh("file") ASSERT if the entry is
// absent. The spawner therefore holds its ball Mesh + Material BY VALUE
// (constructed once from the live scene setup) rather than re-looking-up a name
// on every SPACE. That keeps a live reference so the entry is never evicted, and
// lets the scene assign the spawn material freely without needing a separate
// persistent MeshRenderer to keep it alive.
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
        // Snapshot the rest state on the first frame, by which point the scene's
        // starting bodies have all been added.
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

        // Thrown balls are fast (launchSpeed ~20 m/s) — opt them into CCD so they
        // can't tunnel through the floor / other bodies in one step (CCD-1).
        Physics::PhysicsObject ball =
            Physics::PhysicsObject::Sphere(pos, m_radius, velocity);
        ball.SetContinuous(true);
        std::size_t index = m_engine->AddObject(ball);

        // Reuse the held mesh/material (ref-counted copies) — no per-spawn name
        // lookup, so nothing can have been evicted out from under us.
        m_spawnParent->AddChild(
            (new Entity(pos))
                ->AddComponent(new MeshRenderer(m_ballMesh, m_ballMaterial))
                ->AddComponent(new PhysicsObjectComponent(m_engine, index)));

        // Keep the reset snapshot index-aligned: this new body's index equals
        // m_initial.size() at snapshot + prior spawns.
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
