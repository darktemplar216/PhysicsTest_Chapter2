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
#include "ContactManifold.hpp"
#include "G_GjkEpa.h"

class SceneMgr;
class Entity;

class PhysicsRoutine
{
public:
    
    friend class RigidBody;
    
private:


    btGjkEpaSolver2 m_gjkEpaSolver;
    
    std::vector<RigidBody*> m_rigidBodies;
    
    std::map<int, RigidBody*> m_rigidBodyMap;
    
    std::list<ContactManifold*> m_mainifolds;
    
    int m_rigidBodyUniqueIdAccum = 0;
    int FetchRigidBodyUID() { return ++m_rigidBodyUniqueIdAccum; }
    
    Vector3 m_gravityAcc;
    
    float m_simuationTimeRemaining = 0;

public:
    
    void AddRigidBody(RigidBody* rigidBody);
    RigidBody* GetRigidBody(int id);
    void RemoveRigidBody(RigidBody* rigidBody);
    void RemoveRigidBody(int id);
    void RemoveAllRigidBodies();
    const std::vector<RigidBody*>& getRigidBodies() const;
    
public:
    
    PhysicsRoutine();
    virtual ~PhysicsRoutine();
    
    void InitParams(const Vector3& gravityAcc);
    
    void Update(double deltaTime, long frame);

    void AddOrUpdateManifold(btGjkEpaSolver2::sResults& resf, RigidDataIndex dataIndex, Entity* entityA, Entity* entityB);
    
private:
    
    void OnRigidBodyRemoved(RigidBody* rigidBody);
    
private:
    
    void UpdateVelocities(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
    void UpdateManifolds(RigidDataIndex dataIndex);
    
    void WarmStart(double deltaTime, RigidDataIndex dataIndex);
    void WarmContacts(RigidDataIndex dataIndex, ContactManifold* manifold, double deltaTime);
    
    void UpdatePosAndRotsAndCheckDormancy(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
    void HandleVelocityConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to);
    
    bool CheckIfRigidBodyCanBeDormant(RigidBody* rigidBody, RigidDataIndex dataIndex);
    
    void HandleManifoldForVelocityConstraints(RigidDataIndex from, RigidDataIndex to, ContactManifold* manifold, double deltaTime);
    
    bool FindAllMyManifolds(const RigidBody* me, std::vector<const ContactManifold*>& relatedManiforlds);
    void RemoveRelatedManifolds(RigidBody* rigidBody);
    void RemoveAllManifolds();
    bool IsAHasCheckedWithB(RigidBody* a, RigidBody* b);
    void AddOrUpdateManifold(btGjkEpaSolver2::sResults& result, RigidDataIndex dataIndex, RigidBody* rigidBodyA, RigidBody* rigidBodyB);
    
    
};


#endif /* PhysicsRoutine_hpp */
