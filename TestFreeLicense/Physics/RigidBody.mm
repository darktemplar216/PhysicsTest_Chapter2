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
#include "PhysicsRoutine.hpp"
#include "PhysicsDefs.h"

using namespace std;

Matrix4 RigidData::MakeTRSMatrix() const
{
    Matrix4 tranMatrix; tranMatrix.translate(m_position);
    Matrix4 rotMatrix = m_rotation.matrix();
    Matrix4 scaleMatrix; scaleMatrix.scale(m_scale.x, m_scale.y, m_scale.z);

    return tranMatrix * rotMatrix * scaleMatrix;
}

Matrix4 RigidData::MakeRSMatrix() const
{
    Matrix4 rotMatrix = m_rotation.matrix();
    Matrix4 scaleMatrix; scaleMatrix.scale(m_scale.x, m_scale.y, m_scale.z);
    return rotMatrix * scaleMatrix;
}

Matrix4 RigidData::MakeRMatrix() const
{
    return m_rotation.matrix();
}

RigidBody::RigidBody():m_hullVertices(nullptr),m_uid(0)
{
    
}

RigidBody::~RigidBody()
{
    if(m_routine != nullptr)
    {
        m_routine->OnRigidBodyRemoved(this);
    }
    
    delete[] m_hullVertices;
}

RigidData* RigidBody::getData(RigidDataIndex index)
{
    return const_cast<RigidData*>(getDataConst(index));
}

const RigidData* RigidBody::getDataConst(RigidDataIndex index) const
{
    const RigidData* ret = nullptr;
    if(index != RDI_count)
    {
        ret = &(m_datas[index]);
    }
    return ret;
}

int RigidBody::getUID() const
{
    return m_uid;
}

void RigidBody::InitParamsAsACube(const string& name,
                       const Vector3& scale,
                       const Vector3& pos,
                       const Quaternion& rot,
                       bool isStatic,
                       float mass,
                       float collisionCoefficient,
                       float linearFrictionCoefficient)
{
    m_name = name;
    
    delete[] m_hullVertices;
    m_hullVertices = new float[PhysicsDefs::StdCubeHullVertCount];
    memcpy(m_hullVertices, PhysicsDefs::StdCubeHullVertices, PhysicsDefs::StdCubeHullVertCount * sizeof(float));
    m_hullVerticesCount = PhysicsDefs::StdCubeHullVertCount / 3;
    
    memset(&m_datas, 0, sizeof(RigidData) * RDI_count);
    RigidData& realData = m_datas[RDI_real];
    realData.m_rotation = rot;
    realData.m_position = pos;
    realData.m_scale = scale;
    realData.m_isStatic = isStatic;
    realData.m_isDormant = m_isStatic;
    
    m_isStatic = isStatic;
    if(m_isStatic)
    {
        m_mass = 999999.0f;
        m_oneDivMass = 0.0f;
        m_inertiaLocal = Matrix4(999999.0f, 0, 0, 0,
                                 0, 999999.0f, 0, 0,
                                 0, 0, 999999.0f, 0,
                                 0, 0, 0, 1.0f);
        m_inertiaLocalInverse = Matrix4(0, 0, 0, 0,
                                        0, 0, 0, 0,
                                        0, 0, 0, 0,
                                        0, 0, 0, 1.0f);
    }
    else
    {
        m_mass = fmax(mass, 1.0f);
        m_oneDivMass = 1.0f / m_mass;
        
        // 我们认为标准方形的各边长度为1.0f
        float length = scale.x * 1.0f;
        float height = scale.y * 1.0f;
        float width = scale.z * 1.0f;
        
        m_inertiaLocal = Matrix4(m_mass * 0.33333f * (height * height + width * width), 0, 0, 0,
                                 0, m_mass * 0.33333f * (length * length + width * width), 0, 0,
                                 0, 0, m_mass * 0.33333f * (length * length + height * height), 0,
                                 0, 0, 0, 1.0f);
        m_inertiaLocalInverse = m_inertiaLocal.inverse();
    }

    m_collisionCoefficient = collisionCoefficient;
    m_linearFrictionCoefficient = linearFrictionCoefficient;
}

void RigidBody::CalculateEnergy(RigidData& data)
{
    float linearSpeed = data.m_velocity.length();
    float kineticEnergy = 0.5f * m_mass * linearSpeed * linearSpeed;
    float gravitationalEnergy = m_mass * fmax(data.m_position.y - G_ZERO_HEIGHT, 0.0f);
    data.m_energy = kineticEnergy + gravitationalEnergy;
}

Vector3 RigidBody::getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const
{
    Vector3 ret;
    float length2OfRet = -99999999.0f;
    
    for(int i=0; i<m_hullVerticesCount; i++)
    {
        Vector3 vertex(*(m_hullVertices + 3 * i), *(m_hullVertices + 3 * i + 1), *(m_hullVertices + 3 * i + 2));
        if(trans != nullptr)
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

