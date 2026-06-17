#pragma once
#include <cstddef>

#include "../core/entityComponent.h"
#include "../physics/physicsEngine.h"

// Copies a simulated body's position back onto its Entity's Transform each
// frame, so the rendered entity follows the physics. The body is referenced by
// (engine, index) rather than by pointer, so it survives the engine growing its
// internal std::vector.
class PhysicsObjectComponent : public EntityComponent {
   public:
    PhysicsObjectComponent(Physics::PhysicsEngine* engine, std::size_t index)
        : m_engine(engine), m_index(index) {}

    virtual void Update(float delta) {
        (void)delta;
        const Physics::PhysicsObject& body = m_engine->GetObject(m_index);
        GetTransform()->SetPos(body.GetPosition());
        GetTransform()->SetRot(body.GetOrientation());  // renders tumbling/spin
    }

   private:
    Physics::PhysicsEngine* m_engine;
    std::size_t m_index;
};
