//
//  Entity.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef Entity_hpp
#define Entity_hpp

#include "GMath.h"
#include "RenderRoutine.hpp"
#include "PhysicsRoutine.hpp"

class VBO;
class RigidBody;
class RigidData;
class SceneMgr;

class Entity
{
private:
    
    friend class SceneMgr;
    
private:
    
    SceneMgr* m_sceneMgr = nullptr;
    
public:
    
    std::string name;
    
    Quaternion rotation;
    
    Vector3 position;
    
    Vector3 scale;
    
public:

    VBO* vertexVBO = 0;
    
    VBO* normalVBO = 0;
    
    GLKVector4 baseColor;
    
    int m_rigidBodyUID = 0;
    RigidData* m_rigidData = nullptr;
    
public:
    
    Entity();
    virtual ~Entity();
    
    void InitRenderParam(const float* vertices, int verticesCount, const float* normals, int normalCount);
    void InitPhysicsParam(const RigidBody* rigidBody);
    void Uninit();
    
    void Update(float deltaTime);
    void Draw(const ShaderProgram* program);
    
    const RigidData* GetRigidData() const;
    int getRigidBodyUID() const;
};


#endif /* Entity_hpp */
