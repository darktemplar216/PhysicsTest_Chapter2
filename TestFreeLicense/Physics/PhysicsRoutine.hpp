//
//  PhysicsRoutine.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef PhysicsRoutine_hpp
#define PhysicsRoutine_hpp

#include <stdio.h>
#include "RigidBody.hpp"
#include "ContactReport.hpp"
#include "G_GjkEpa.h"

class Entity;

class PhysicsRoutine
{
private:
    static PhysicsRoutine* instance;
    
    PhysicsRoutine();
    
    btGjkEpaSolver2 gjkEpaSolver;
    
    std::list<Entity*> physicalEntities;
    
    std::list<ContactManifold*> manifolds;
    
public:
    
    ~PhysicsRoutine();
    
    static bool IsValid();
    static PhysicsRoutine* GetInstance();
    static PhysicsRoutine* CreateInstance();
    static void DestroyInstance();
    
    void Init();
    void Uninit();
    
    void RemoveRelatedManifold(Entity* entity);
    void AddOrUpdateManifold(btGjkEpaSolver2::sResults& resf, RigidDataIndex dataIndex, Entity* entityA, Entity* entityB);
    bool FindAllMyManifolds(const RigidBody* me, std::list<const ContactManifold*>& relatedManifolds);
    
public:
    void Update(double deltaTime, long frame);
    
private:
    bool IsEntityAHasCheckedWithEntityB(Entity* a, Entity* b);
    
private:
    void Finalize(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
    void InitRoutine();
    void UpdateVelocities(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    void UpdateManifolds(RigidDataIndex dataIndex);
    void WarmStart(double deltaTime, RigidDataIndex dataIndex);
    void UpdatePosAndRots(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
    void HandleVelocityConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    void HandlePositionConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
private:
    void HandleContactForVelocityConstraints(RigidDataIndex from, RigidDataIndex to, ContactPoint& contact, double deltaTime);
    void HandleContactForPositionConstraints(RigidDataIndex from, RigidDataIndex to, ContactPoint& contact, double deltaTime);
    void WarmContact(RigidDataIndex dataIndex, ContactPoint& contact, double deltaTime);
};


#endif /* PhysicsRoutine_hpp */
