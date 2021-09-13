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
#include "ContactManifold.hpp"

//#define LogPhysicsRoutine
//#define LogEntityStatus
//#define LogImpulseSolve

PhysicsRoutine::PhysicsRoutine():m_rigidBodyUniqueIdAccum(0)
{
}

PhysicsRoutine::~PhysicsRoutine()
{
    RemoveAllRigidBodies();
}

void PhysicsRoutine::InitParams(const Vector3& gravityAcc)
{
    m_gravityAcc = gravityAcc;
}

void PhysicsRoutine::AddRigidBody(RigidBody* rigidBody)
{
    if(rigidBody == nullptr)
    {
        return;
    }
    
    if(rigidBody->m_routine == nullptr)
    {
        rigidBody->m_routine = this;
    }
    
    // 注意，说明可以多PhysicsRoutine
    if(rigidBody->m_routine != this)
    {
        return;
    }
    
    // 注意RigidBody在这里获得UID
    if(rigidBody->m_uid <= 0)
    {
        rigidBody->m_uid = FetchRigidBodyUID();
    }
    
    std::map<int, RigidBody*>::iterator mapFindIter = m_rigidBodyMap.find(rigidBody->m_uid);
    if(mapFindIter == m_rigidBodyMap.end())
    {
        m_rigidBodyMap.insert(std::pair<int, RigidBody*>(rigidBody->m_uid, rigidBody));
        m_rigidBodies.push_back(rigidBody);
    }
}

RigidBody* PhysicsRoutine::GetRigidBody(int id)
{
    RigidBody* ret = nullptr;
    
    std::map<int, RigidBody*>::iterator mapFindIter = m_rigidBodyMap.find(id);
    if(mapFindIter != m_rigidBodyMap.end())
    {
        ret = mapFindIter->second;
    }
    
    return ret;
}

void PhysicsRoutine::RemoveRigidBody(RigidBody* rigidBody)
{
    if(rigidBody != nullptr)
    {
        // 注意请 参见 ~RigidBody, 里面回调 OnRigidBodyRemoved
        delete rigidBody;
    }
}

void PhysicsRoutine::RemoveRigidBody(int id)
{
    RigidBody* ret = nullptr;
    
    std::map<int, RigidBody*>::iterator mapFindIter = m_rigidBodyMap.find(id);
    if(mapFindIter != m_rigidBodyMap.end())
    {
        ret = mapFindIter->second;
    }
    
    // 注意请 参见 ~RigidBody, 里面回调 OnRigidBodyRemoved
    RemoveRigidBody(ret);
}

void PhysicsRoutine::RemoveAllRigidBodies()
{
    std::vector<RigidBody*>::iterator vecIter = m_rigidBodies.begin();
    while(vecIter != m_rigidBodies.end())
    {
        if(*vecIter != nullptr)
        {
            // 注意 这里断开了 RigidBody和Routine的关系，防止回调OnRigidBodyRemoved
            (*vecIter)->m_routine = nullptr;
            
            delete (*vecIter);
        }

        vecIter++;
    }
    
    m_rigidBodies.clear();
    m_rigidBodyMap.clear();
    
    RemoveAllManifolds();
}

void PhysicsRoutine::OnRigidBodyRemoved(RigidBody* rigidBody)
{
    if(rigidBody == nullptr)
    {
        return;
    }
    
    std::map<int, RigidBody*>::iterator mapFindIter = m_rigidBodyMap.find(rigidBody->getUID());
    if(mapFindIter != m_rigidBodyMap.end())
    {
        m_rigidBodyMap.erase(mapFindIter);
        
        std::vector<RigidBody*>::iterator vecIter = m_rigidBodies.begin();
        while(vecIter != m_rigidBodies.end())
        {
            RigidBody* vecIterRigidBody = *vecIter;
            if(vecIterRigidBody != nullptr && vecIterRigidBody == rigidBody)
            {
                m_rigidBodies.erase(vecIter);
                break;
            }
            vecIter++;
        }
    }
    
    RemoveRelatedManifolds(rigidBody);
}

const std::vector<RigidBody*>& PhysicsRoutine::getRigidBodies() const
{
    return m_rigidBodies;
}

bool RemoveManifoldIfRigidBodyMatches (const ContactManifold* manifold)
{
    return (manifold == nullptr);
}

void PhysicsRoutine::RemoveRelatedManifolds(RigidBody* rigidBody)
{
    std::list<ContactManifold*>::iterator iterBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = m_mainifolds.end();
    while(iterBegin != iterEnd)
    {
        if((*iterBegin)->m_rigidBodyA == rigidBody || (*iterBegin)->m_rigidBodyB == rigidBody)
        {
            delete *iterBegin;
            *iterBegin = nullptr;
        }
        iterBegin++;
    }
    
    m_mainifolds.remove_if(RemoveManifoldIfRigidBodyMatches);
}

void PhysicsRoutine::RemoveAllManifolds()
{
    std::list<ContactManifold*>::iterator iterBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = m_mainifolds.end();
    while(iterBegin != iterEnd)
    {
        delete *iterBegin;
        iterBegin++;
    }
    m_mainifolds.clear();
}

void PhysicsRoutine::AddOrUpdateManifold(btGjkEpaSolver2::sResults& result, RigidDataIndex dataIndex, RigidBody* rigidBodyA, RigidBody* rigidBodyB)
{
    ContactManifold* manifold = nullptr;
    
    std::list<ContactManifold*>::iterator iterBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = m_mainifolds.end();
    while(iterBegin != iterEnd)
    {
        ContactManifold* curManifold = *iterBegin;
        if((curManifold->m_rigidBodyA == rigidBodyA && curManifold->m_rigidBodyB == rigidBodyB)
           || (curManifold->m_rigidBodyB == rigidBodyA && curManifold->m_rigidBodyA == rigidBodyB))
        {
            manifold = curManifold;
            break;
        }
        iterBegin++;
    }
    
    if(manifold == nullptr)
    {
        manifold = new ContactManifold();
        manifold->m_rigidBodyA = rigidBodyA;
        manifold->m_rigidBodyB = rigidBodyB;
        m_mainifolds.push_back(manifold);
    }
    
    if(manifold != nullptr)
    {
        manifold->TryToAddNewContact(dataIndex, rigidBodyA, rigidBodyB, result);
    }
}

void PhysicsRoutine::Update(double deltaTime, long frame)
{
    const float smoothedDeltaTime = 0.01f;
    const int velocityConstrantIterTimes = 10;
    
    while(deltaTime > 0)
    {
        UpdateVelocities(smoothedDeltaTime, RDI_real, RDI_real);
        UpdateManifolds(RDI_real);
        
        WarmStart(smoothedDeltaTime, RDI_real);
        
        for (int i=0; i<velocityConstrantIterTimes; i++)
        {
            HandleVelocityConstraints(smoothedDeltaTime, RDI_real, RDI_real);
        }
        
        UpdatePosAndRotsAndCheckDormancy(smoothedDeltaTime, RDI_real, RDI_real);
        
        deltaTime -= smoothedDeltaTime;
    }
}

void PhysicsRoutine::HandleManifoldForVelocityConstraints(RigidDataIndex from, RigidDataIndex to, ContactManifold* manifold, double deltaTime)
{
    if(manifold == nullptr)
    {
        return;
    }
    
    if(manifold->m_contactPointCount == 0)
    {
        return;
    }
    
    RigidBody* rigidBodyA = manifold->m_rigidBodyA;
    RigidData& dataA = rigidBodyA->m_datas[to];
    
    RigidBody* rigidBodyB = manifold->m_rigidBodyB;
    RigidData& dataB = rigidBodyB->m_datas[to];
    
    if((dataA.m_isStatic || dataA.m_isDormant) && (dataB.m_isStatic || dataB.m_isDormant))
    {
        return;
    }
    
    Matrix4 inertialWorldInverseA = dataA.MakeRMatrix() * (dataA.MakeRMatrix().transpose() * rigidBodyA->m_inertiaLocalInverse);
    Matrix4 inertialWorldInverseB = dataB.MakeRMatrix() * (dataB.MakeRMatrix().transpose() * rigidBodyB->m_inertiaLocalInverse);
    
    float combinedImpu1seCoefficient = manifold->GetCollisionImpulseCoefficient();
    float combinedFrictionCoefficient = manifold->GetFrictionImpulseCoefficient();
    
    Vector3 combinedNormalForFriction;
    Vector3 combinedRelativePosForFrictionA;
    Vector3 combinedRelativePosForFrictionB;
    float totalNormalImpulse = 0;
    
    for (int i=0; i<manifold->m_contactPointCount; i++)
    {
        ContactPoint& contact = manifold->contactPoints[i];
        
        //let normal point from a-→b
        Vector3 normal = -dataB.m_rotation.matrix() * contact.localNormalInBSpace;
        Vector3 relativePosA = contact.globalWittnessPointA - dataA.m_position;
        Vector3 relativePosB = contact.globalWittnessPointB - dataB.m_position;
        Vector3 combinedVelocityA = dataA.m_velocity + (dataA.m_angularVel * relativePosA);
        Vector3 combinedVelocityB = dataB.m_velocity + (dataB.m_angularVel * relativePosB);
        Vector3 relativeVelocity = combinedVelocityB - combinedVelocityA;
        
        //--collision impulse begin
        float relativeVelocityValOnNormal = relativeVelocity.dot(normal);
        float biasPenetrationDepth = 0.0f;
        if((-contact.penetrationDistance) > PENETRATION_TOLERANCE && deltaTime > 0)
        {
            float erpScalar = fmax(0.0f, (-contact.penetrationDistance) - PENETRATION_TOLERANCE) / PENETRATION_TOLERANCE;
            //no reason, I just made this thing up
            erpScalar =fmax(0.1527f, fmin(erpScalar, 1.234f));
            float erp= 0.2f * erpScalar;
            biasPenetrationDepth = -(erp / deltaTime) * fmax(0.0f, (-contact.penetrationDistance) - PENETRATION_TOLERANCE);
        }
        
        float normalImpulseValA = normal.dot(inertialWorldInverseA.transform3x3(relativePosA * normal) * relativePosA);
        float normalImpulseValB = normal.dot(inertialWorldInverseB.transform3x3(relativePosB * normal) * relativePosB);
        float normalImpulseVal = -((1.000f + combinedImpu1seCoefficient) * relativeVelocityValOnNormal + biasPenetrationDepth) /
            (rigidBodyA->m_oneDivMass + rigidBodyB->m_oneDivMass + normalImpulseValA + normalImpulseValB);
        
        float oldNormalImpulse = contact.totalNormalImpulse;
        contact.totalNormalImpulse = fmax(contact.totalNormalImpulse + normalImpulseVal, 0.0f);
        normalImpulseVal = contact.totalNormalImpulse - oldNormalImpulse;
        
        totalNormalImpulse += contact.totalNormalImpulse;
        
        Vector3 collisionImpulse = normal * normalImpulseVal;
        
        if( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
        {
            dataA.m_velocity -= collisionImpulse * rigidBodyA->m_oneDivMass;
            dataA.m_angularVel -= normalImpulseVal * inertialWorldInverseA.transform3x3(relativePosA * normal);
        }
        if( !rigidBodyB->m_isStatic && !dataB.m_isDormant)
        {
            dataB.m_velocity += collisionImpulse * rigidBodyB->m_oneDivMass;
            dataB.m_angularVel += normalImpulseVal * inertialWorldInverseB.transform3x3(relativePosB *normal);
        }
        
        //--collision impulse end
        
        //--friction impulse begin
        Vector3 tangent = relativeVelocity - (relativeVelocityValOnNormal * normal);
        tangent = normal.cross(tangent).cross(normal).normalize();
        float relativeVelocityValOnTangent = relativeVelocity.dot(tangent);
        contact.tangent = tangent;
        
        float tangentImpulseValA = tangent.dot(inertialWorldInverseA.transform3x3(relativePosA * tangent)* relativePosA);
        float tangentImpulseValB = tangent.dot(inertialWorldInverseB.transform3x3(relativePosB * tangent)* relativePosB);
        
        //combinedFrictionCoefficient = 0.225f;
        float tangentImpulseVal = -combinedFrictionCoefficient * relativeVelocityValOnTangent /
            (rigidBodyA->m_oneDivMass + rigidBodyB->m_oneDivMass + tangentImpulseValA + tangentImpulseValB);
        
        Vector3 tangentImpulse = tangentImpulseVal * tangent;
        
        if( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
        {
            dataA.m_velocity -= tangentImpulse * rigidBodyA->m_oneDivMass;
            dataA.m_angularVel -= tangentImpulseVal * inertialWorldInverseA.transform3x3(relativePosA * tangent);
        }
        if( !rigidBodyB->m_isStatic && !dataB.m_isDormant)
        {
            dataB.m_velocity += tangentImpulse * rigidBodyB->m_oneDivMass;
            dataB.m_angularVel += tangentImpulseVal * inertialWorldInverseB.transform3x3(relativePosB * tangent);
        }
        
        //--friction impulse end
        
        combinedNormalForFriction += normal;
        combinedRelativePosForFrictionA += relativePosA;
        combinedRelativePosForFrictionB += relativePosB;
    }
    
    combinedNormalForFriction.normalize();
    combinedRelativePosForFrictionA /= (float)manifold->m_contactPointCount;
    combinedRelativePosForFrictionB /= (float)manifold->m_contactPointCount;
    
    //－-twist friction impulse begin
    Vector3 relativeVelocityForTwistFriction = dataB.m_angularVel - dataA.m_angularVel;
    float relativeVelocityValForTwistFrictionOnNormal = relativeVelocityForTwistFriction.dot(combinedNormalForFriction);
    //Vector3 relativeVelocityForTwistFrictionOnNormal = relativeVelocityValForTwistFrictionOnNormal * combinedNormalForFriction;
    
    float twistImpulseValA = combinedNormalForFriction.dot(inertialWorldInverseA.transform3x3(combinedNormalForFriction));
    float twistImpulseValB = combinedNormalForFriction.dot(inertialWorldInverseB.transform3x3(combinedNormalForFriction));
    float twistImpulseVal = -1.0f * relativeVelocityValForTwistFrictionOnNormal / (twistImpulseValA + twistImpulseValB);
    
    Vector3 twistImpulse = twistImpulseVal * combinedNormalForFriction;
    
    if( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
    {
        dataA.m_angularVel -= inertialWorldInverseA.transform3x3(twistImpulse);
    }
    if ( !rigidBodyB->m_isStatic && !dataB.m_isDormant)
    {
        dataB.m_angularVel += inertialWorldInverseB.transform3x3(twistImpulse);
    }
    //--twist friction impulse end
}

void PhysicsRoutine::WarmContacts(RigidDataIndex dataIndex, ContactManifold* manifold, double deltaTime)
{
    if(manifold == nullptr)
    {
        return;
    }
    
    if(manifold->m_contactPointCount == 0)
    {
        return;
    }
    
    RigidBody* rigidBodyA = manifold->m_rigidBodyA;
    RigidData& dataA = rigidBodyA->m_datas[dataIndex];
    
    RigidBody* rigidBodyB = manifold->m_rigidBodyB;
    RigidData& dataB = rigidBodyB->m_datas[dataIndex];
    
    if((dataA.m_isStatic || dataA.m_isDormant) && (dataB.m_isStatic || dataB.m_isDormant))
    {
        return;
    }
    
    Matrix4 inertialWorldInverseA = dataA.MakeRMatrix() * (dataA.MakeRMatrix().transpose() * rigidBodyA->m_inertiaLocalInverse);
    Matrix4 inertialWorldInverseB = dataB.MakeRMatrix() * (dataB.MakeRMatrix().transpose() * rigidBodyB->m_inertiaLocalInverse);
    
    bool isAtLeastOneContactPointIsAlive =false;
    
    for (int i=0; i< manifold->m_contactPointCount; i++)
    {
        ContactPoint& contact = manifold->contactPoints[i];
        
        isAtLeastOneContactPointIsAlive |= contact.isAlive;
        
        //let normal point from a-→b
        Vector3 normal = -dataB.m_rotation.matrix() * contact.localNormalInBSpace;
        Vector3 relativePosA = contact.globalWittnessPointA - dataA.m_position;
        Vector3 relativePosB = contact.globalWittnessPointB - dataB.m_position;
        
        //--collision impulse begin
        float normalImpulseVal = contact.totalNormalImpulse;
        Vector3 normalImpulse = normal * normalImpulseVal;
        if ( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
        {
            dataA.m_velocity -= normalImpulse * rigidBodyA->m_oneDivMass;
            dataA.m_angularVel -= normalImpulseVal * inertialWorldInverseA.transform3x3(relativePosA * normal);
        }
        
        if( !rigidBodyB->m_isStatic && !dataB.m_isDormant)
        {
            dataB.m_velocity += normalImpulse * rigidBodyB->m_oneDivMass;
            dataB.m_angularVel += normalImpulseVal * inertialWorldInverseB.transform3x3(relativePosB*normal);
        }
        //--collision impulse end
        
        //--friction impulse begin
        if(contact.isAlive)
        {
            Vector3 tangent = contact.tangent;
            float tangentImpulseVal = contact.totalTangentImpulse;
            Vector3 tangentImpulse = tangentImpulseVal * tangent;
            
            if ( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
            {
                dataA.m_velocity -= tangentImpulse * rigidBodyA->m_oneDivMass;
                dataA.m_angularVel -= tangentImpulseVal * inertialWorldInverseA.transform3x3(relativePosA * tangent);
            }
            if( !rigidBodyB->m_isStatic && !dataB.m_isDormant)
            {
                dataB.m_velocity += tangentImpulse * rigidBodyB->m_oneDivMass;
                dataB.m_angularVel += tangentImpulseVal * inertialWorldInverseB.transform3x3(relativePosB * tangent);
            }
            //contact.totalTangentImpulse =0;
        }
        //--friction impulse end
    }
}

bool PhysicsRoutine::IsAHasCheckedWithB(RigidBody* a, RigidBody* b)
{
    if(a == b)
    {
        return true;
    }
    
    std::vector<RigidBody*>::iterator iterBegin = a->haveCheckedWithList.begin();
    std::vector<RigidBody*>::iterator iteBEnd = a->haveCheckedWithList.end();
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
    std::vector<RigidBody*>::iterator iterABegin = m_rigidBodies.begin();
    std::vector<RigidBody*>::iterator iterAEnd = m_rigidBodies.end();
    while(iterABegin != iterAEnd)
    {
        RigidBody* rigidBody = *iterABegin;
        RigidData& fromData = rigidBody->m_datas[from];
        RigidData& toData = rigidBody->m_datas[to];
        
        if( !rigidBody->m_isStatic && !fromData.m_isDormant)
        {
            Vector3 combinedForce = fromData.m_force + rigidBody->m_mass * m_gravityAcc;
            toData.m_acceleration = combinedForce * rigidBody->m_oneDivMass;
            toData.m_velocity = fromData.m_velocity + (toData.m_acceleration * deltaTime);
            toData.m_angularVel = fromData.m_angularVel;
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
    std::vector<RigidBody*>::iterator iterRigidBodyBegin = m_rigidBodies.begin();
    std::vector<RigidBody*>::iterator iterRigidBodyEnd = m_rigidBodies.end();
    while(iterRigidBodyBegin != iterRigidBodyEnd)
    {
        (*iterRigidBodyBegin)->haveCheckedWithList.clear();
        iterRigidBodyBegin++;
    }
    
    //1. get rid of the contact point which drifted far from it's original position
    std::list<ContactManifold*>::iterator iterManifoldsBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterManifoldsEnd = m_mainifolds.end();
    while (iterManifoldsBegin !=iterManifoldsEnd)
    {
        (*iterManifoldsBegin)->UpdateContacts(dataIndex);
        iterManifoldsBegin++;
    }
    
    //2. try to find new contatct points
    std::vector<RigidBody*>::iterator iterRigidBodyABegin = m_rigidBodies.begin();
    std::vector<RigidBody*>::iterator iterRigidBodyAEnd = m_rigidBodies.end();
    while(iterRigidBodyABegin != iterRigidBodyAEnd)
    {
        RigidBody* rigidBodyA = *iterRigidBodyABegin;
        RigidData& dataA = rigidBodyA->m_datas[dataIndex];
        if( !rigidBodyA->m_isStatic && !dataA.m_isDormant)
        {
            std::vector<RigidBody*>::iterator iterRigidBodyBBegin = m_rigidBodies.begin();
            std::vector<RigidBody*>::iterator iterRigidBodyBEnd = m_rigidBodies.end();
            while (iterRigidBodyBBegin != iterRigidBodyBEnd)
            {
                RigidBody* rigidBodyB = *iterRigidBodyBBegin;
                RigidData& dataB = rigidBodyB->m_datas[dataIndex];
                if ( !IsAHasCheckedWithB(rigidBodyA,rigidBodyB))
                {
                    rigidBodyA->haveCheckedWithList.push_back(rigidBodyB);
                    rigidBodyB->haveCheckedWithList.push_back(rigidBodyA);
                    
                    Matrix4 transMatrA = dataA.MakeTRSMatrix();
                    Matrix4 transMatrB = dataB.MakeTRSMatrix();
                    
                    btGjkEpaSolver2::sResults resf;
                    memset(&resf, 0, sizeof(btGjkEpaSolver2::sResults));
                    
                    bool isCollisionHappened = btGjkEpaSolver2::Penetration(rigidBodyA, rigidBodyB,
                                                                            &transMatrA, &transMatrB,
                                                                            dataA.m_position - dataB.m_position,
                                                                            resf);
                    if(isCollisionHappened)
                    {
                        if(resf.distance<0)
                        {
                            dataA.m_isDormant = false;
                            dataB.m_isDormant = false;
                            AddOrUpdateManifold(resf, dataIndex, rigidBodyA, rigidBodyB);
                        }
                    }
                }
                iterRigidBodyBBegin++;
            }
        }
        iterRigidBodyABegin++;
    }
}

void PhysicsRoutine::WarmStart(double deltaTime, RigidDataIndex dataIndex)
{
    std::list<ContactManifold*>::iterator iterBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = m_mainifolds.end();
    while(iterBegin != iterEnd)
    {
        WarmContacts(dataIndex, *iterBegin, deltaTime);
        iterBegin++;
    }
}

void PhysicsRoutine::HandleVelocityConstraints(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::list<ContactManifold*>::iterator iterColRepBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterColRepEnd = m_mainifolds.end();
    while(iterColRepBegin != iterColRepEnd)
    {
        HandleManifoldForVelocityConstraints(from, to, *iterColRepBegin, deltaTime);
        iterColRepBegin++;
    }
}

void PhysicsRoutine::UpdatePosAndRotsAndCheckDormancy(double deltaTime, RigidDataIndex from, RigidDataIndex to)
{
    std::vector<RigidBody*>::iterator iterBegin = m_rigidBodies.begin();
    std::vector<RigidBody*>::iterator iterEnd = m_rigidBodies.end();
    while(iterBegin != iterEnd)
    {
        RigidBody* rigidBody = *iterBegin;
        RigidData& fromData = rigidBody->m_datas[from];
        RigidData& toData = rigidBody->m_datas[to];
        
        if( !rigidBody->m_isStatic && !fromData.m_isDormant)
        {
            toData.m_position = fromData.m_position + (toData.m_velocity * deltaTime);
            
            float angularVelVal = toData.m_angularVel.length();
            float angularDeltaVal = angularVelVal * deltaTime;
            Vector3 angularVelAxis = angularDeltaVal > EPSILON_FLT ? toData.m_angularVel / angularVelVal : sVec3AxisY;
            Quaternion deltaRotQuat(angularDeltaVal, angularVelAxis);
            toData.m_rotation = deltaRotQuat * fromData.m_rotation;
            
            toData.m_scale = fromData.m_scale;
            
            CheckIfRigidBodyCanBeDormant(rigidBody, from);
        }
        
        iterBegin++;
    }
}

bool PhysicsRoutine::FindAllMyManifolds(const RigidBody* me, std::vector<const ContactManifold*>& relatedManiforlds)
{
    bool ret = false;
    
    relatedManiforlds.clear();
    
    std::list<ContactManifold*>::iterator iterBegin = m_mainifolds.begin();
    std::list<ContactManifold*>::iterator iterEnd = m_mainifolds.end();
    while(iterBegin != iterEnd)
    {
        ContactManifold* manifold = *iterBegin;
        if(manifold != nullptr && (manifold->m_rigidBodyA == me || manifold->m_rigidBodyB == me))
        {
            relatedManiforlds.push_back(manifold);
            ret = true;
        }
        
        iterBegin++;
    }
    
    return ret;
}

bool PhysicsRoutine::CheckIfRigidBodyCanBeDormant(RigidBody* rigidBody, RigidDataIndex dataIndex)
{
    bool ret = false;
    
    RigidData& data = rigidBody->m_datas[dataIndex];
    if( !rigidBody->m_isStatic && !data.m_isDormant)
    {
        if(data.m_velocity.lengthSquared()<=DORMAINT_THRESHOLD && data.m_angularVel.lengthSquared() <= DORMAINT_THRESHOLD)
        {
            std::vector<const ContactManifold*> relatedManifold;
            if (FindAllMyManifolds(rigidBody, relatedManifold))
            {
                bool isCanBeDormant=true;
                
                std::vector<const ContactManifold*>::iterator iterManifoldBegin = relatedManifold.begin();
                std::vector<const ContactManifold*>::iterator iterManifoldEnd = relatedManifold.end();
                while (iterManifoldBegin != iterManifoldEnd)
                {
                    const ContactManifold* manifold = *iterManifoldBegin;
                    for (int i=0; i<manifold->m_contactPointCount; i++)
                    {
                        const ContactPoint& point = manifold->contactPoints[i];
                        isCanBeDormant &= (fabs(point.penetrationDistance) < PENETRATION_TOLERANCE);
                        if( !isCanBeDormant)
                        {
                            break;
                        }
                    }
                    
                    if( !isCanBeDormant)
                    {
                        break;
                    }
                    
                    iterManifoldBegin++;
                }
                
                if(isCanBeDormant)
                {
                    ret= true;
                    data.m_isDormant = true;
                    data.m_velocity =sVec3Zero;
                    data.m_angularVel=sVec3Zero;
                }
            }
        }
    }
    return ret;
}


