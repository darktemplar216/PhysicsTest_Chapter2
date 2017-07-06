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

class VBO;
class Entity;

class RigidData
{
public:
    
    //这一次模拟帧，物体被施加了多大的力//
    Vector3 force;
    //线速度//
    Vector3 velocity;
    //加速度//
    Vector3 acceleration;
    //角速度//
    Vector3 angularVel;
    //位置//
    Vector3 position;
    //朝向//
    Quaternion rotation;
    //缩放//
    Vector3 scale;
    //能量//
    float energy = 0;
    //是不是“静置”状态，主要用来省性能和增加稳定性//
    bool isDormant = false;
    
    Matrix4 MakeTransMatrix() const;
};

class PointCloud
{
public:
    virtual Vector3 getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const = 0;
};

class RigidVertex : public PointCloud
{
public:
    Vector3 vertex;
    
    RigidVertex(float x, float y, float z):vertex(x, y, z)
    {
    }
    
    virtual Vector3 getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const
    {
        if(trans != 0)
        {
            return (*trans) * vertex;
        }
        else
        {
            return vertex;
        }
    }
};

enum RigidDataIndex
{
    RDI_real = 0,
    RDI_rk4_1,
    RDI_rk4_2,
    RDI_rk4_3,
    RDI_rk4_4,
    RDI_count
};

class RigidBody : public PointCloud
{
public:
    std::string name;
    
    //用于表示这个刚体是不是“静态的”，不能被推动，比如我们的地板//
    bool isStatic = false;
    //质量//
    float mass = 1;
    //1除以质量//
    float oneDivMass = 1;
    //如果等于0那完全非弹性碰撞，1完全弹性碰撞//
    float impulseCoefficient = 1;
    //转动惯量，决定一个物体在xyz三个轴上面有多难转动//
    Matrix4 inertia;
    //转动惯量矩阵的逆矩阵，计算上有用//
    Matrix4 inertiaInverse;
    //摩擦系数//
    float linearFrictionCoefficient = 0.3;
    //这里是我们把刚体的一些“易变”参数封装好几份，以后有用//
    RigidData datas[RDI_count];

    void CalculateEnergy(RigidData& data);
    
    bool CheckIfCanbeDormant(RigidDataIndex dataIndex);
    
    void SetPhysicallParams(bool inIsStatic, float inMass, const Matrix4& inInertia, float inLinearFrictionCoefficient);
    
public:
    
    std::vector<Entity*> haveCheckedWithList;
    
    virtual Vector3 getFarthestVectAtDir(const Vector3& dir, const Matrix4* trans) const;
    
public:
    
    float* hull = 0;
    
    int vertexCount = 0;
    
    VBO* debugVBO;
    
    RigidBody();
    virtual ~RigidBody();
    
    void Init(std::string inName, const float* hullData, int size);
};

#endif /* RigidBody_hpp */
