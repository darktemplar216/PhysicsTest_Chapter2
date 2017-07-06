//
//  RigidBody.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "RigidBody.hpp"
#include "Entity.hpp"

Matrix4 RigidData::MakeTransMatrix() const
{
    Matrix4 tranMatrix; tranMatrix.translate(position);
    Matrix4 rotMatrix = rotation.matrix();
    Matrix4 scaleMatrix; scaleMatrix.scale(scale.x, scale.y, scale.z);

    return tranMatrix * rotMatrix * scaleMatrix;
}

RigidBody::RigidBody():debugVBO(0)
{
    
}

RigidBody::~RigidBody()
{
    if(hull != 0)
    {
        delete[] hull;
    }
}

void RigidBody::Init(std::string inName, const float* hullData, int size)
{
    //先简单memcpy一下，其实需要提取凸包//
    if(hullData != 0 && size > 0 && size % 3 == 0)
    {
        hull = new float[size];
        memcpy(hull, hullData, size * sizeof(float));
        vertexCount = size / 3;
    }
    else
    {
        NSLog(@"RigidBody::Init: data invalid");
    }
    
    memset(&datas, 0, sizeof(RigidData) * RDI_count);
    
    name = inName;
}

void RigidBody::SetPhysicallParams(bool inIsStatic, float inMass, const Matrix4& inInertia, float inLinearFrictionCoefficient)
{
    isStatic = inIsStatic;
    
    if(isStatic)
    {
        mass = 99999999.0f;
        oneDivMass = 0;
        inertia  = Matrix4(99999999.0f,0,0,0,
                                       0,99999999.0f,0,0,
                                       0,0,99999999.0f,0,
                                       0,0,0,1);
        inertiaInverse = Matrix4(0,0,0,0,
                                0,0,0,0,
                                0,0,0,0,
                                0,0,0,1);
    }
    else
    {
        mass = inMass;
        mass = mass > 0 ? mass : 1;
        oneDivMass = 1 / mass;
        inertia  = inInertia;
        inertiaInverse = inertia.inverse();
    }
}

void RigidBody::CalculateEnergy(RigidData& data)
{
    float linearSpeed = data.velocity.length();
    float kineticEnergy = 0.5f * mass * linearSpeed * linearSpeed;
    
    float gravitationalEnergy = mass * fmax(data.position.y - G_ZERO_HEIGHT, 0.0f);
    
    data.energy = kineticEnergy + gravitationalEnergy;
}

Vector3 RigidBody::getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const
{
    Vector3 ret;
    float length2OfRet = -99999999.0f;
    
    for(int i=0; i<vertexCount; i++)
    {
        Vector3 vertex(*(hull + 3 * i), *(hull + 3 * i + 1), *(hull + 3 * i + 2));
        if(trans != 0)
        {
            vertex = (*trans) * vertex;
        }
        float curDot = vertex.dot(dir);
        if(curDot > length2OfRet)
        {
            length2OfRet = curDot;
            ret = vertex;
        }
    }
    return ret;
}

bool RigidBody::CheckIfCanbeDormant(RigidDataIndex dataIndex)
{
    bool ret = false;
    
    RigidData& data = datas[dataIndex];
    
    if(!isStatic && !data.isDormant)
    {
        if(data.velocity.lengthSquared() <= DORMAINT_THRESHOLD && data.angularVel.lengthSquared() <= DORMAINT_THRESHOLD)
        {
            std::list<const ContactManifold*> relatedManifold;
            if(PhysicsRoutine::IsValid() && PhysicsRoutine::GetInstance()->FindAllMyManifolds(this, relatedManifold))
            {
                bool isCanBeDormant = true;
                
                std::list<const ContactManifold*>::iterator iterManifoldBegin = relatedManifold.begin();
                std::list<const ContactManifold*>::iterator iterManifoldEnd = relatedManifold.end();
                while(iterManifoldBegin != iterManifoldEnd)
                {
                    const ContactManifold* manifold = *iterManifoldBegin;
                    
                    for(int i=0; i<manifold->contactPointCount; i++)
                    {
                        const ContactPoint& point = manifold->contactPoints[i];
                        
                        isCanBeDormant &= (fabs(point.penetrationDistance) < (PENETRATION_TOLERANCE + EPSILON_FLT));
                        if(!isCanBeDormant)
                        {
                            break;
                        }
                    }
                    
                    if(!isCanBeDormant)
                    {
                        break;
                    }
                    
                    iterManifoldBegin++;
                }
                
                if(isCanBeDormant)
                {
                    ret = true;
                    data.isDormant = true;
                    data.velocity = sVec3Zero;
                    data.angularVel = sVec3Zero;
                }
            }
        }
    }
    
    return ret;
}
