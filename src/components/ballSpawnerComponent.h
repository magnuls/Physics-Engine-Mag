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

// Drop/spawn balls at runtime + reset.
//
// Attach to a scene entity that's already wired into the graph (so the
// CoreEngine pointer has been propagated to it). On the spawn key it creates a
// new sphere: a PhysicsObject added to the shared PhysicsEngine and launched
// from the camera along its aim, plus a rendered Entity AddChild'd under
// m_spawnParent so it falls and collides like the pre-placed objects. The reset
// key restores every body to a captured rest snapshot (initial pos + zero vel).
//
// Safe to spawn mid-frame: Entity walks its children by index (for i < size()),
// so appending during ProcessInput doesn't invalidate the traversal, and
// AddChild propagates SetEngine/AddToEngine to the new entity's components. The
// body is referenced by its stable engine index, which survives the engine
// growing its internal vector.
//
// Reset: PhysicsEngine has no remove-body API, so reset doesn't delete spawned
// balls — it sends every body back to where it started and zeroes velocity, so
// the whole scene falls again. Indices stay aligned because this component is
// the only thing that adds bodies at runtime.
//
// Mesh/Material data live in static resource maps that are erased when the last
// reference drops, and Material("name")/Mesh("file") assert if the entry is
// gone. So the spawner holds its ball Mesh + Material by value (built once from
// the live scene) instead of re-looking-up a name on every SPACE — keeps a live
// reference so nothing gets evicted.
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

        Physics::PhysicsObject ball =
            Physics::PhysicsObject::Sphere(pos, m_radius, velocity);
        // Opt thrown balls into CCD so they can't tunnel through thin geometry /
        // other bodies in one step. (This once injected energy on fast floor
        // landings until the speculative-contact solver bug was fixed; speculative
        // contacts are now cold-started and skip friction.)
        ball.SetContinuous(true);
        // Give spawned balls friction so they grip the floor and roll/settle
        // instead of sliding forever. The solver combines friction as
        // sqrt(fA*fB), so a friction-0 ball is frictionless against any surface —
        // the ball must carry its own. Matches the floor/pre-placed value.
        // 0.4: grips enough to roll but the spin-up on impact is gentler.
        ball.SetFriction(0.4f);
        // Angular damping governs how fast the roll-spin bleeds off as the ball
        // slows: 2.0 settles it in ~5 s. Kept higher than the pre-placed 0.4
        // because spawned balls land far faster than dropped ones. Linear damping
        // 0.05 so it still rolls a bit before stopping.
        ball.SetAngularDamping(2.0f);
        ball.SetLinearDamping(0.05f);
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
