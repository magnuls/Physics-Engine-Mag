#pragma once
#include "../core/entityComponent.h"
#include "../physics/physicsEngine.h"

// Drives a Physics::PhysicsEngine from the Entity/Component update loop. Attach
// one to an Entity; every fixed step CoreEngine -> Game -> Entity calls Update,
// which advances the simulation and resolves collisions.
//
// The engine is owned by value; GetPhysicsEngine() hands out a reference so the
// game can add bodies during setup (and wire PhysicsObjectComponents to them).
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
