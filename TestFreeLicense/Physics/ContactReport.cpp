//
//  ContactReport.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/31.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#include "ContactReport.hpp"
#include "Entity.hpp"

Vector3 ContactPoint::GetGlobalNormalB2A(RigidDataIndex dataIndex)
{
    const RigidData& rigidDataB = entityB->rigidBody->datas[dataIndex];
    Matrix4 localToWorldB = rigidDataB.rotation.matrix();
    return localToWorldB * localNormalInBSpace;
}

void ContactManifold::UpdateContacts(RigidDataIndex dataIndex)
{
    for(int i=0; i<contactPointCount; i++)
    {
        ContactPoint& point = contactPoints[i];
        
        const RigidData& rigidDataA = point.entityA->rigidBody->datas[dataIndex];
        const RigidData& rigidDataB = point.entityB->rigidBody->datas[dataIndex];
        
        Matrix4 localToWorldA = rigidDataA.MakeTransMatrix();
        Matrix4 localToWorldB = rigidDataB.MakeTransMatrix();
        
        Vector3 curGlobalContactPointA = localToWorldA * point.localWittnessPointA;
        Vector3 curGlobalContactPointB = localToWorldB * point.localWittnessPointB;
        Vector3 curGlobalNormal = point.GetGlobalNormalB2A(dataIndex);
        
        float disBetweenCurAndOldA = (curGlobalContactPointA -  point.globalWittnessPointA).lengthSquared();
        float disBetweenCurAndOldB = (curGlobalContactPointB -  point.globalWittnessPointB).lengthSquared();
        
        bool isStillPenetrating = (curGlobalContactPointA - curGlobalContactPointB).dot(curGlobalNormal) < 0;
        
        if(!isStillPenetrating
           || disBetweenCurAndOldA > CONTACT_DRIFTING_THRESHOLD
           || disBetweenCurAndOldB > CONTACT_DRIFTING_THRESHOLD)
        {
            for(int shift = i; shift<CONTACT_POINT_COUNT - 1; shift++)
            {
                contactPoints[shift] = contactPoints[shift + 1];
            }
            contactPointCount--;
        }
    }
}

void ContactManifold::TryToAddNewContact(RigidDataIndex dataIndex,
                                         Entity* entityA,
                                         Entity* entityB,
                                         const btGjkEpaSolver2::sResults& result)
{
    if(IfShouldAddNewContactPoint(dataIndex, result))
    {
        ContactPoint newCPoint;
        memset(&newCPoint, 0, sizeof(ContactPoint));
        
        newCPoint.entityA = entityA;
        newCPoint.entityB = entityB;
        
        //newCPoint.result = result;
        newCPoint.globalWittnessPointA = result.witnesses[0];
        newCPoint.globalWittnessPointB = result.witnesses[1];
        
        const RigidData& rigidDataA = entityA->rigidBody->datas[dataIndex];
        const RigidData& rigidDataB = entityB->rigidBody->datas[dataIndex];
        
        Matrix4 localToGlobalTransA = rigidDataA.MakeTransMatrix().inverse();
        newCPoint.localWittnessPointA = localToGlobalTransA * newCPoint.globalWittnessPointA;
        Matrix4 localToGlobalTransB = rigidDataB.MakeTransMatrix().inverse();
        newCPoint.localWittnessPointB = localToGlobalTransB * newCPoint.globalWittnessPointB;
        
        //normal point from b -> a in local space of b//
        newCPoint.localNormalInBSpace = rigidDataB.rotation.matrix().inverse() * result.normal;
        newCPoint.tangent = sVec3Zero;
        newCPoint.tangent2 = sVec3Zero;
        
        newCPoint.penetrationDistance = result.distance;
        
        contactPointCount++;
        contactPoints[contactPointCount - 1] = newCPoint;
        
        if(contactPointCount == CONTACT_POINT_COUNT)
        {
            RearrengeContactPoints();
        }
    }
}

bool ContactManifold::IfShouldAddNewContactPoint(RigidDataIndex dataIndex, const btGjkEpaSolver2::sResults& candidate)
{
    bool ret = true;
    
    for(int i=0; i<contactPointCount; i++)
    {
        ContactPoint& point = contactPoints[i];
        
        float disA = (candidate.witnesses[0] -  point.globalWittnessPointA).lengthSquared();
        float disB = (candidate.witnesses[1] -  point.globalWittnessPointB).lengthSquared();
        
        if(disA <= CONTACT_DRIFTING_THRESHOLD && disB <= CONTACT_DRIFTING_THRESHOLD)
        {
            ret = false;
            break;
        }
    }
    
    return ret;
}

void ContactManifold::RearrengeContactPoints()
{
    float maxDepth = 999999.0f;
    int deepestIndex = 0;
    for(int i=0; i<contactPointCount; i++)
    {
        ContactPoint& point = contactPoints[i];
        if(point.penetrationDistance < maxDepth)
        {
            deepestIndex = i;
            maxDepth = point.penetrationDistance;
        }
    }
    
    ContactPoint& newCPoint = contactPoints[CONTACT_POINT_COUNT - 1];
    int indexToRemove = GetIndexToRemove(deepestIndex, newCPoint.localWittnessPointA);
    contactPoints[indexToRemove] = newCPoint;
    contactPointCount = CONTACT_POINT_COUNT - 1;
}

// Return the index that will be removed.
/// The index of the contact point with the larger penetration
/// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
/// the different area and we want to keep the contacts with the largest area. The new point is also
/// kept. In order to compute the area of a quadrilateral, we use the formula :
/// Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
/// when we compute this area, we do not calculate it exactly but we
/// only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
/// this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
/// by Erwin Coumans (http://wwww.bulletphysics.org).
int ContactManifold::GetIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const
{
    
    assert(contactPointCount == CONTACT_POINT_COUNT);
    
    float area0 = 0.0;       // Area with contact 1,2,3 and newPoint
    float area1 = 0.0;       // Area with contact 0,2,3 and newPoint
    float area2 = 0.0;       // Area with contact 0,1,3 and newPoint
    float area3 = 0.0;       // Area with contact 0,1,2 and newPoint
    
    if (indexMaxPenetration != 0) {
        // Compute the area
        Vector3 vector1 = newPoint - contactPoints[1].globalWittnessPointA;
        Vector3 vector2 = contactPoints[3].globalWittnessPointA -
        contactPoints[2].globalWittnessPointA;
        Vector3 crossProduct = vector1.cross(vector2);
        area0 = crossProduct.lengthSquared();
    }
    if (indexMaxPenetration != 1) {
        // Compute the area
        Vector3 vector1 = newPoint - contactPoints[0].globalWittnessPointA;
        Vector3 vector2 = contactPoints[3].globalWittnessPointA -
        contactPoints[2].globalWittnessPointA;
        Vector3 crossProduct = vector1.cross(vector2);
        area1 = crossProduct.lengthSquared();
    }
    if (indexMaxPenetration != 2) {
        // Compute the area
        Vector3 vector1 = newPoint - contactPoints[0].globalWittnessPointA;
        Vector3 vector2 = contactPoints[3].globalWittnessPointA -
        contactPoints[1].globalWittnessPointA;
        Vector3 crossProduct = vector1.cross(vector2);
        area2 = crossProduct.lengthSquared();
    }
    if (indexMaxPenetration != 3) {
        // Compute the area
        Vector3 vector1 = newPoint - contactPoints[0].globalWittnessPointA;
        Vector3 vector2 = contactPoints[2].globalWittnessPointA -
        contactPoints[1].globalWittnessPointA;
        Vector3 crossProduct = vector1.cross(vector2);
        area3 = crossProduct.lengthSquared();
    }
    
    // Return the index of the contact to remove
    return GetMaxArea(area0, area1, area2, area3);
}

// Return the index of maximum area
int ContactManifold::GetMaxArea(float area0, float area1, float area2, float area3) const
{
    if (area0 < area1) {
        if (area1 < area2) {
            if (area2 < area3) return 3;
            else return 2;
        }
        else {
            if (area1 < area3) return 3;
            else return 1;
        }
    }
    else {
        if (area0 < area2) {
            if (area2 < area3) return 3;
            else return 2;
        }
        else {
            if (area0 < area3) return 3;
            else return 0;
        }
    }
}












