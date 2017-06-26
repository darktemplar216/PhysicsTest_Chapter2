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

class Entity
{
public:
    
    std::string name;
    
    Quaternion rotation;
    
    Vector3 position;
    
    Vector3 scale;
    
public:
    
    RigidBody* rigidBody = 0;
    
public:

    VBO* vertexVBO = 0;
    
    VBO* normalVBO = 0;
    
    GLKVector4 baseColor;
    
public:
    
    Entity();
    virtual ~Entity();
    
    void Init(const float* vertices, int verticesCount,
              const float* normals, int normalCount,
              const float* hull, int hullCount);
    
    void Uninit();
    
    void Draw(const ShaderProgram* program);
};


#endif /* Entity_hpp */
