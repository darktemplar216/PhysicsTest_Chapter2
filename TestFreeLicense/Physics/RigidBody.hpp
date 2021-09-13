//
//  RigidBody.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef RigidBody_hpp
#define RigidBody_hpp

#include "GMath.h"
#include <string>

class VBO;
class Entity;
class PhysicsRoutine;

class RigidData
{
public:
    
    //这一次模拟帧，物体被施加了多大的力//
    Vector3 m_force;
    //线速度//
    Vector3 m_velocity;
    //加速度//
    Vector3 m_acceleration;
    //角速度//
    Vector3 m_angularVel;
    //位置//
    Vector3 m_position;
    //朝向//
    Quaternion m_rotation;
    //缩放//
    Vector3 m_scale = sVec3One;
    //能量//
    float m_energy = 0;
    //是不是“静置”状态，主要用来省性能和增加稳定性//
    bool m_isDormant = false;
    //是不是静态场景物体//
    bool m_isStatic = false;
    
    Matrix4 MakeTRSMatrix() const;
    
    Matrix4 MakeRSMatrix() const;
    
    Matrix4 MakeRMatrix() const;
};

class PointCloud
{
public:
    virtual Vector3 getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const = 0;
};

enum RigidDataIndex
{
    RDI_real = 0,
    RDI_rk4_1,
    RDI_count
};

class RigidBody : public PointCloud
{
public:
    
    friend class PhysicsRoutine;
    
public:
    
    std::string m_name;
    
    //用于表示这个刚体是不是“静态的”，不能被推动，比如我们的地板//
    bool m_isStatic = false;
    
    //质量//
    float m_mass = 1;
    
    //1除以质量//
    float m_oneDivMass = 1;
    
    //如果等于0那完全非弹性碰撞，1完全弹性碰撞//
    float m_collisionCoefficient = 1.0f;
    
    Matrix4 m_inertiaLocal;
    
    Matrix4 m_inertiaLocalInverse;
    
    //摩擦系数//
    float m_linearFrictionCoefficient = 0.3;
    
    //这里是我们把刚体的一些“易变”参数封装好几份，以后有用//
    RigidData m_datas[RDI_count];

protected:
    
    int m_uid = 0;
    
    float* m_hullVertices = nullptr;
    
    int m_hullVerticesCount = 0;
    
    PhysicsRoutine* m_routine = nullptr;
    
    std::vector<RigidBody*> haveCheckedWithList;
    
public:

    RigidBody();
    virtual ~RigidBody();
    
    void InitParamsAsACube(const std::string& name,
                           const Vector3& scale,
                           const Vector3& pos,
                           const Quaternion& rot,
                           bool isStatic,
                           float mass,
                           float collisionCoefficient,
                           float linearFrictionCoefficient);
        
    virtual Vector3 getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const;
        
    RigidData* getData(RigidDataIndex index);
    const RigidData* getDataConst(RigidDataIndex index) const;
    
    int getUID() const;

    void CalculateEnergy(RigidData& data);
    
};

#endif /* RigidBody_hpp */
