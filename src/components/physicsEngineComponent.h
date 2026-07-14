#pragma once
#include "../core/entityComponent.h"
#include "../physics/physicsEngine.h"

// Drives a Physics::PhysicsEngine from the component update loop; Update steps
// the sim and resolves collisions. GetPhysicsEngine exposes it to add bodies.
class PhysicsEngineComponent : public EntityComponent {
   public:
    PhysicsEngineComponent() {}

    virtual void Update(float delta) {
        m_physicsEngine.Simulate(delta);
        m_physicsEngine.HandleCollisions();
    }

    Physics::PhysicsEngine& GetPhysicsEngine() { return m_physicsEngine; }

   private:
    Physics::PhysicsEngine m_physicsEngine;
};
