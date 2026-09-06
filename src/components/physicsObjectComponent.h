#pragma once
#include <cstddef>

#include "../core/entityComponent.h"
#include "../physics/physicsEngine.h"

// Copies a simulated body's position and orientation onto its Transform each
// frame so rendering follows the physics. The body is referenced by engine and
// index so it survives the engine's vector growing.
class PhysicsObjectComponent : public EntityComponent {
   public:
    PhysicsObjectComponent(Physics::PhysicsEngine* engine, std::size_t index)
        : m_engine(engine), m_index(index) {}

    virtual void Update(float delta) {
        (void)delta;
        const Physics::PhysicsObject& body = m_engine->GetObject(m_index);
        GetTransform()->SetPos(body.GetPosition());
        GetTransform()->SetRot(body.GetOrientation());  // renders tumbling and spin
    }

   private:
    Physics::PhysicsEngine* m_engine;
    std::size_t m_index;
};
