//
//  ContactReport.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/31.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef ContactReport_hpp
#define ContactReport_hpp

#include <stdio.h>
#include "G_GjkEpa.h"
#include "RigidBody.hpp"

#define CONTACT_POINT_COUNT 5
#define CONTACT_DRIFTING_THRESHOLD  0.004f

class ContactPoint
{
public:
    //A物体的碰撞点在其局部坐标系下的位置//
    Vector3 localWittnessPointA;
    //B物体的碰撞点在其局部坐标系下的位置//
    Vector3 localWittnessPointB;
    //A物体的碰撞点在在世界坐标系下的位置//
    Vector3 globalWittnessPointA;
    //B物体的碰撞点在在世界坐标系下的位置//
    Vector3 globalWittnessPointB;
    //第一次碰撞时 A物体的碰撞点在在世界坐标系下的位置//
    Vector3 originalGlobalWittnessPointA;
    //第一次碰撞时 B物体的碰撞点在在世界坐标系下的位置//
    Vector3 originalGlobalWittnessPointB;
    //基于物体B局部坐标系的法线//
    Vector3 localNormalInBSpace;
    //碰撞切线1，摩擦力的方向//
    Vector3 tangent;
    //碰撞切线2，摩擦力2的方向//
    Vector3 tangent2;
    //碰撞深度//
    float penetrationDistance;
    //法线方向上的总碰撞冲量//
    float totalNormalImpulse;
    //切线1方向上的总碰撞冲量//
    float totalTangentImpulse;
    //切线2方向上的总碰撞冲量//
    float totalTangentImpulse2;
    //这个接触点是不是在经过了多帧之后仍然有效
    bool isAlive;
    
};

class ContactManifold
{
public:
    
    //实体A，注意这里这是记录A和B碰撞了，它们实际碰撞点里面记录的entityA entityB才代表用来计算碰撞的方向//
    RigidBody* m_rigidBodyA;
    //实体B，注意这里这是记录A和B碰撞了，它们实际碰撞点里面记录的entityA entityB才代表用来计算碰撞的方向//
    RigidBody* m_rigidBodyB;
    
    //碰撞点们//
    ContactPoint contactPoints[CONTACT_POINT_COUNT];
    int m_contactPointCount = 0;
    
    void UpdateContacts(RigidDataIndex dataIndex);
    void TryToAddNewContact(RigidDataIndex dataIndex,
                            RigidBody* possibleRigidA,
                            RigidBody* possibleRigidB,
                            const btGjkEpaSolver2::sResults& result);
    
    float GetCollisionImpulseCoefficient() const;
    float GetFrictionImpulseCoefficient() const;
    
private:
    
    bool IfShouldAddNewContactPoint(RigidDataIndex dataIndex, const btGjkEpaSolver2::sResults& candidate);
    void RearrengeContactPoints();
    
    int GetIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const;
    int GetMaxArea(float area0, float area1, float area2, float area3) const;
};

#endif /* ContactReport_hpp */
