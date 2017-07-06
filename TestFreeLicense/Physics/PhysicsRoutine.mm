//
//  PhysicsRoutine.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//


#import <Foundation/Foundation.h>
#include "PhysicsRoutine.hpp"
#include "SceneMgr.hpp"
#include "Entity.hpp"
#include "ContactReport.hpp"

//#define LogPhysicsRoutine
//#define LogEntityStatus
//#define LogImpulseSolve


PhysicsRoutine* PhysicsRoutine::instance = 0;

PhysicsRoutine::PhysicsRoutine()
{
    Init();
}

PhysicsRoutine::~PhysicsRoutine()
{
    Uninit();
}

bool PhysicsRoutine::IsValid()
{
    return instance != 0;
}

PhysicsRoutine* PhysicsRoutine::GetInstance()
{
    return instance;
}

PhysicsRoutine* PhysicsRoutine::CreateInstance()
{
    if(instance == 0)
    {
        instance = new PhysicsRoutine();
    }
    return instance;
}

void PhysicsRoutine::DestroyInstance()
{
    if(instance != 0)
    {
        delete instance;
        instance = 0;
    }
}

void PhysicsRoutine::Init()
{
    
}

void PhysicsRoutine::Uninit()
{
    
}

bool RemoveManifoldIfEntityMatches (const ContactManifold* manifold)
{
    return (manifold == 0);
}

void PhysicsRoutine::RemoveRelatedManifold(Entity* entity)
{
    std::list<ContactManifold*>::iterator iterBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = manifolds.end();
    while(iterBegin != iterEnd)
    {
        if((*iterBegin)->entityA == entity || (*iterBegin)->entityB == entity)
        {
            delete *iterBegin;
            *iterBegin = 0;
        }
        iterBegin++;
    }
    
    manifolds.remove_if(RemoveManifoldIfEntityMatches);
}

void PhysicsRoutine::AddOrUpdateManifold(btGjkEpaSolver2::sResults& resf, RigidDataIndex dataIndex, Entity* entityA, Entity* entityB)
{
    ContactManifold* manifold = 0;
    
    std::list<ContactManifold*>::iterator iterBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = manifolds.end();
    while(iterBegin != iterEnd)
    {
        ContactManifold* curManifold = *iterBegin;
        if((curManifold->entityA == entityA && curManifold->entityB == entityB)
           || (curManifold->entityB == entityA && curManifold->entityA == entityB))
        {
            manifold = curManifold;
            break;
        }
        iterBegin++;
    }
    
    if(manifold == 0)
    {
        manifold = new ContactManifold();
        manifold->entityA = entityA;
        manifold->entityB = entityB;
        manifolds.push_back(manifold);
    }
    
    if(manifold != 0)
    {
        manifold->TryToAddNewContact(dataIndex, entityA, entityB, resf);
    }
}

void PhysicsRoutine::Update(double deltaTime, long frame)
{
    if(!SceneMgr::IsValid())
    {
        return;
    }
    
    //初始化物理模拟，把实体的位置同步给刚体//
    InitRoutine();
    
    //这里我们更新物体的受力、速度信息//
    UpdateVelocities(deltaTime, RDI_real, RDI_real);
    
    //更新当前碰撞信息//
    UpdateManifolds(RDI_real);
    
    //用于增加稳定性//
    WarmStart(deltaTime, RDI_real, RDI_real);
    
    //处理所有的碰撞，迭代多次，求得稳定的结果//
    for(int i=0; i < 10; i++)
    {
        HandleVelocityConstraints(deltaTime, RDI_real, RDI_real);
    }
    
    for(int i=0; i < 3; i++)
    {
        HandlePositionConstraints(deltaTime, RDI_real, RDI_real);
    }
    
    //更新物体的位置信息//
    UpdatePosAndRots(deltaTime, RDI_real, RDI_real);
    
    //把物体的位置信息同步给实体，用于绘制//
    Finalize(deltaTime, RDI_real, RDI_real);
}

void PhysicsRoutine::InitRoutine()
{
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    
    //重新整理哪些实体是物理实体，简称物体:)//
    physicalEntities.clear();
    
    //这里把实体的位置信息同步给物理模块, 对于新建的物体起初始化作用//
    std::list<Entity*>::iterator iterBegin = sceneMgr->entityList.begin();
    std::list<Entity*>::iterator iterEnd = sceneMgr->entityList.end();
    while(iterBegin != iterEnd)
    {
        Entity* entity = *iterBegin;
        
        if(entity->rigidBody != 0)
        {
            physicalEntities.push_back(entity);
            
            RigidBody* rigidBody = entity->rigidBody;
            
            rigidBody->datas[RDI_real].position = entity->position;
            rigidBody->datas[RDI_real].rotation = entity->rotation;
            rigidBody->datas[RDI_real].scale = entity->scale;
            
            entity->rigidBody->haveCheckedWithList.clear();
        }
        iterBegin++;
    }
}

void PhysicsRoutine::Finalize(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<Entity*>::iterator iterBegin = physicalEntities.begin();
    std::list<Entity*>::iterator iterEnd = physicalEntities.end();
    while(iterBegin != iterEnd)
    {
        Entity* entity = *iterBegin;
        RigidBody* rigidBody = entity->rigidBody;
        RigidData& fromData = rigidBody->datas[from];
        RigidData& toData = rigidBody->datas[to];
        memcpy(&toData, &fromData, sizeof(RigidData));
        
        entity->position = toData.position;
        entity->rotation = toData.rotation;
        
        iterBegin++;
    }
}

void PhysicsRoutine::HandleContactForVelocityConstraints(RigidDataIndex from, RigidDataIndex to, ContactPoint& contact, double deltaTime)
{
    Entity* entityA = contact.entityA;
    RigidBody* rigidBodyA = entityA->rigidBody;
    RigidData& dataA = rigidBodyA->datas[to];
    
    Entity* entityB = contact.entityB;
    RigidBody* rigidBodyB = entityB->rigidBody;
    RigidData& dataB = rigidBodyB->datas[to];
    
    if(((rigidBodyA->isStatic | dataA.isDormant) & (rigidBodyB->isStatic | dataB.isDormant)) == false)
    {
        //let normal point from a -> b
        Vector3 normal = - contact.GetGlobalNormalB2A(from);
        
        Vector3 relativePosA = contact.globalWittnessPointA - dataA.position;
        Vector3 relativePosB = contact.globalWittnessPointB - dataB.position;
        
        float combinedImpulseCoefficient = 0.00000f;   //rigidBodyA->impulseCoefficient * rigidBodyB->impulseCoefficient;
        float combinedFrictionCoefficient = 0.8f;
        
        Vector3 combinedVelocityA = dataA.velocity + (dataA.angularVel * relativePosA);
        Vector3 combinedVelocityB = dataB.velocity + (dataB.angularVel * relativePosB);
        Vector3 relativeVelocity = combinedVelocityB - combinedVelocityA;
        
        //-- collision impulse begin ----------------------------------------------------------------------------------------------------------------------------------------------------//
        float biasPenetrationDepth = 0.0;
        if ((-contact.penetrationDistance) > PENETRATION_TOLERANCE && deltaTime > 0)
        {
            biasPenetrationDepth = 0.8f * (PENETRATION_VELOCITY_BIAS / deltaTime) * fmax(0.0f, float((-contact.penetrationDistance) - PENETRATION_TOLERANCE));
        }
        
        float collisionImpulseValA = (relativePosA * normal).dot(rigidBodyA->inertiaInverse.transform3x3(relativePosA * normal));
        float collisionImpulseValB = (relativePosB * normal).dot(rigidBodyB->inertiaInverse.transform3x3(relativePosB * normal));
        float collisionImpulseVal = -(1.000f + combinedImpulseCoefficient + biasPenetrationDepth) * relativeVelocity.dot(normal) /
                                        (rigidBodyA->oneDivMass + rigidBodyB->oneDivMass + collisionImpulseValA + collisionImpulseValB);

        float newImpulse = fmax(contact.totalNormalImpulse + collisionImpulseVal, 0.0f);
        collisionImpulseVal = newImpulse - contact.totalNormalImpulse;
        contact.totalNormalImpulse = newImpulse;
        
        Vector3 collisionImpulse = normal * collisionImpulseVal;
        
        if(!rigidBodyA->isStatic)
        {
            dataA.velocity = dataA.velocity - (collisionImpulse * rigidBodyA->oneDivMass);
            dataA.angularVel = dataA.angularVel - (collisionImpulseVal * rigidBodyA->inertiaInverse.transform3x3(relativePosA * normal));
        }
        
        if(!rigidBodyB->isStatic)
        {
            dataB.velocity = dataB.velocity + (collisionImpulse * rigidBodyB->oneDivMass);
            dataB.angularVel = dataB.angularVel + (collisionImpulseVal * rigidBodyB->inertiaInverse.transform3x3(relativePosB * normal));
        }
        
        //-- collision impulse end ----------------------------------------------------------------------------------------------------------------------------------------------------//
        
        //-- friction impulse begin ----------------------------------------------------------------------------------------------------------------------------------------------------//
        Vector3 relativeVelocityTangent = relativeVelocity - (relativeVelocity.dot(normal) * normal);
        float relativeVelocityOnTangent = relativeVelocityTangent.normalizeF();
        contact.tangent = relativeVelocityTangent;
        Vector3 tangent = relativeVelocityTangent;
        
        float frictionImpulseValA = (relativePosA * tangent).dot(rigidBodyA->inertiaInverse.transform3x3(relativePosA * tangent));
        float frictionImpulseValB = (relativePosB * tangent).dot(rigidBodyB->inertiaInverse.transform3x3(relativePosB * tangent));
        float frictionImpulseVal = -1.000f * relativeVelocityOnTangent /
        (rigidBodyA->oneDivMass + rigidBodyB->oneDivMass + frictionImpulseValA + frictionImpulseValB);
        frictionImpulseVal = fmax(fabs(combinedFrictionCoefficient * collisionImpulseVal), fabs(frictionImpulseVal)) * (frictionImpulseVal > 0 ? 1.0f : -1.0f);
        Vector3 frictionImpulse = frictionImpulseVal * tangent;
        
        if(!rigidBodyA->isStatic)
        {
            dataA.velocity = dataA.velocity - frictionImpulse * rigidBodyA->oneDivMass;
            dataA.angularVel = dataA.angularVel - frictionImpulseVal * rigidBodyA->inertiaInverse.transform3x3(relativePosA * tangent);
        }
        
        if(!rigidBodyB->isStatic)
        {
            dataB.velocity = dataB.velocity + frictionImpulse * rigidBodyB->oneDivMass;
            dataB.angularVel = dataB.angularVel + frictionImpulseVal * rigidBodyB->inertiaInverse.transform3x3(relativePosB * tangent);
        }
        
        //-- friction impulse end ----------------------------------------------------------------------------------------------------------------------------------------------------//
        
        if(!rigidBodyA->isStatic) dataA.CheckIfCanbeDormant();
        if(!rigidBodyB->isStatic) dataB.CheckIfCanbeDormant();
    }
}

void PhysicsRoutine::HandleContactForPositionConstraints(RigidDataIndex from, RigidDataIndex to, ContactPoint& contact, double deltaTime)
{
    Entity* entityA = contact.entityA;
    RigidBody* rigidBodyA = entityA->rigidBody;
    RigidData& dataA = rigidBodyA->datas[to];
    
    Entity* entityB = contact.entityB;
    RigidBody* rigidBodyB = entityB->rigidBody;
    RigidData& dataB = rigidBodyB->datas[to];
    
    if(((rigidBodyA->isStatic | dataA.isDormant) & (rigidBodyB->isStatic | dataB.isDormant)) == false)
    {
        //let normal point from a -> b
        //Vector3 normal = contact.negativeNormal;
        
        
        
        
    }
}

void PhysicsRoutine::WarmContact(RigidDataIndex from, RigidDataIndex to, ContactPoint& contact, double deltaTime)
{
    Entity* entityA = contact.entityA;
    RigidBody* rigidBodyA = entityA->rigidBody;
    RigidData& dataA = rigidBodyA->datas[to];
    
    Entity* entityB = contact.entityB;
    RigidBody* rigidBodyB = entityB->rigidBody;
    RigidData& dataB = rigidBodyB->datas[to];
    
    //let normal point from a -> b
    Vector3 normal = - contact.GetGlobalNormalB2A(from);
    
    Vector3 relativePosA = contact.globalWittnessPointA - dataA.position;
    Vector3 relativePosB = contact.globalWittnessPointB - dataB.position;
    
    //-- collision impulse begin ----------------------------------------------------------------------------------------------------------------------------------------------------//
    float collisionImpulseVal = contact.totalNormalImpulse;
    Vector3 collisionImpulse = normal * collisionImpulseVal;
    
    if(!rigidBodyA->isStatic)
    {
        dataA.velocity = dataA.velocity - (collisionImpulse * rigidBodyA->oneDivMass);
        dataA.angularVel = dataA.angularVel - (collisionImpulseVal * rigidBodyA->inertiaInverse.transform3x3(relativePosA * normal));
    }
    
    if(!rigidBodyB->isStatic)
    {
        dataB.velocity = dataB.velocity + (collisionImpulse * rigidBodyB->oneDivMass);
        dataB.angularVel = dataB.angularVel + (collisionImpulseVal * rigidBodyB->inertiaInverse.transform3x3(relativePosB * normal));
    }
    //-- collision impulse end ----------------------------------------------------------------------------------------------------------------------------------------------------//
    
    //-- friction impulse begin ----------------------------------------------------------------------------------------------------------------------------------------------------//
    Vector3 tangent = contact.tangent;
    float frictionImpulseVal = contact.totalTangentImpulse;
    Vector3 frictionImpulse = frictionImpulseVal * tangent;
    
    if(!rigidBodyA->isStatic)
    {
        dataA.velocity = dataA.velocity - frictionImpulse * rigidBodyA->oneDivMass;
        dataA.angularVel = dataA.angularVel - frictionImpulseVal * rigidBodyA->inertiaInverse.transform3x3(relativePosA * tangent);
    }
    
    if(!rigidBodyB->isStatic)
    {
        dataB.velocity = dataB.velocity + frictionImpulse * rigidBodyB->oneDivMass;
        dataB.angularVel = dataB.angularVel + frictionImpulseVal * rigidBodyB->inertiaInverse.transform3x3(relativePosB * tangent);
    }
    //-- friction impulse end ----------------------------------------------------------------------------------------------------------------------------------------------------//
}

bool PhysicsRoutine::IsEntityAHasCheckedWithEntityB(Entity* a, Entity* b)
{
    if(a == b)
    {
        return true;
    }
    
    std::vector<Entity*>::iterator iterBegin = a->rigidBody->haveCheckedWithList.begin();
    std::vector<Entity*>::iterator iteBEnd = a->rigidBody->haveCheckedWithList.end();
    while(iterBegin != iteBEnd)
    {
        if(*iterBegin == b)
        {
            return true;
        }
        iterBegin++;
    }
    return false;
}

void PhysicsRoutine::UpdateVelocities(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<Entity*>::iterator iterABegin = physicalEntities.begin();
    std::list<Entity*>::iterator iterAEnd = physicalEntities.end();
    while(iterABegin != iterAEnd)
    {
        Entity* entity = *iterABegin;
        RigidBody* rigidBody = entity->rigidBody;
        RigidData& fromData = rigidBody->datas[from];
        RigidData& toData = rigidBody->datas[to];
        
        if(!rigidBody->isStatic)
        {
            if(!fromData.isDormant)
            {
                Vector3 combinedForce = fromData.force + (sVec3AxisNegY * rigidBody->mass * G_ACC);
                
                toData.acceleration = combinedForce * rigidBody->oneDivMass;
                
                toData.velocity = fromData.velocity + (toData.acceleration * deltaTime);
                
                toData.angularVel = fromData.angularVel;
            }
            else
            {
                memcpy(&toData, &fromData, sizeof(RigidData));
            }
        }
        else
        {
            memcpy(&toData, &fromData, sizeof(RigidData));
        }
        
        iterABegin++;
    }
}

void PhysicsRoutine::UpdateManifolds(RigidDataIndex dataIndex)
{
    std::list<ContactManifold*>::iterator iterBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = manifolds.end();
    while(iterBegin != iterEnd)
    {
        (*iterBegin)->UpdateContacts(dataIndex);
        iterBegin++;
    }
    
    std::list<Entity*>::iterator iterABegin = physicalEntities.begin();
    std::list<Entity*>::iterator iterAEnd = physicalEntities.end();
    while(iterABegin != iterAEnd)
    {
        Entity* entityA = *iterABegin;
        RigidBody* rigidBodyA = entityA->rigidBody;
        RigidData& dataA = rigidBodyA->datas[dataIndex];
        
        if(!rigidBodyA->isStatic && !dataA.isDormant)
        {
            std::list<Entity*>::iterator iterBBegin = physicalEntities.begin();
            std::list<Entity*>::iterator iterBEnd = physicalEntities.end();
            while(iterBBegin != iterBEnd)
            {
                Entity* entityB = *iterBBegin;
                RigidBody* rigidBodyB = entityB->rigidBody;
                RigidData& dataB = rigidBodyB->datas[dataIndex];
                
                if(!IsEntityAHasCheckedWithEntityB(entityA, entityB))
                {
                    rigidBodyA->haveCheckedWithList.push_back(entityB);
                    rigidBodyB->haveCheckedWithList.push_back(entityA);
                    
                    Matrix4 transMatrA = dataA.MakeTransMatrix();
                    Matrix4 transMatrB = dataB.MakeTransMatrix();
                    
                    btGjkEpaSolver2::sResults resf;
                    memset(&resf, 0, sizeof(btGjkEpaSolver2::sResults));
                    
                    bool isCollisionHappened = gjkEpaSolver.Penetration(rigidBodyA,
                                                                        rigidBodyB,
                                                                        (const Matrix4*) &transMatrA,
                                                                        (const Matrix4*) &transMatrB,
                                                                        dataA.position - dataB.position,
                                                                        resf) && (resf.distance < 0);
                    if(isCollisionHappened)
                    {
                        dataA.isDormant = false;
                        dataB.isDormant = false;
                        
                        AddOrUpdateManifold(resf, dataIndex, entityA, entityB);
                    }
                }
                iterBBegin++;
            }
        }
        iterABegin++;
    }
}

void PhysicsRoutine::WarmStart(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<ContactManifold*>::iterator iterBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = manifolds.end();
    while(iterBegin != iterEnd)
    {
        ContactManifold* manifold = *iterBegin;
        for(int i=0; i<manifold->contactPointCount; i++)
        {
            WarmContact(from, to, manifold->contactPoints[i], deltaTime);
        }
        iterBegin++;
    }
}

void PhysicsRoutine::HandleVelocityConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<ContactManifold*>::iterator iterColRepBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterColRepEnd = manifolds.end();
    while(iterColRepBegin != iterColRepEnd)
    {
        ContactManifold* manifold = *iterColRepBegin;
        
        for(int i=0; i<manifold->contactPointCount; i++)
        {
            HandleContactForVelocityConstraints(from, to, manifold->contactPoints[i], deltaTime);
        }
        
        iterColRepBegin++;
    }
}

void PhysicsRoutine::HandlePositionConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<ContactManifold*>::iterator iterColRepBegin = manifolds.begin();
    std::list<ContactManifold*>::iterator iterColRepEnd = manifolds.end();
    while(iterColRepBegin != iterColRepEnd)
    {
        ContactManifold* manifold = *iterColRepBegin;
        
        for(int i=0; i<manifold->contactPointCount; i++)
        {
            HandleContactForPositionConstraints(from, to, manifold->contactPoints[i], deltaTime);
        }
        
        iterColRepBegin++;
    }
}

void PhysicsRoutine::UpdatePosAndRots(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<Entity*>::iterator iterABegin = physicalEntities.begin();
    std::list<Entity*>::iterator iterAEnd = physicalEntities.end();
    while(iterABegin != iterAEnd)
    {
        Entity* entity = *iterABegin;
        
        RigidBody* rigidBody = entity->rigidBody;
        RigidData& fromData = rigidBody->datas[from];
        RigidData& toData = rigidBody->datas[to];
        
        if(!rigidBody->isStatic)
        {
            if(!fromData.isDormant)
            {
                toData.position = fromData.position + (toData.velocity * deltaTime);
                
                float angularVelVal = toData.angularVel.length();
                float angularDeltaVal = angularVelVal * deltaTime;
                Vector3 angularVelAxis = angularDeltaVal > EPSILON_FLT ? toData.angularVel / angularVelVal : sVec3AxisY;
                Quaternion deltaRotQuat(angularDeltaVal, angularVelAxis);
                toData.rotation = deltaRotQuat * fromData.rotation;
                
                toData.scale = fromData.scale;
            }
        }
        
        iterABegin++;
    }
}










