//
//  SceneMgr.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "SceneMgr.hpp"
#include "Entity.hpp"
#include "TestData.h"


SceneMgr* SceneMgr::instance = 0;


SceneMgr::SceneMgr()
{
    memset(lightsOn, 0, sizeof(lightsOn));
    memset(lightWorldPos, 0, sizeof(lightWorldPos));
    memset(lightDiffuse, 0, sizeof(lightDiffuse));
    memset(lightSpecular, 0, sizeof(lightSpecular));
}

SceneMgr::~SceneMgr()
{
    
}

bool SceneMgr::IsValid()
{
    return instance != 0;
}

SceneMgr* SceneMgr::GetInstance()
{
    return instance;
}

SceneMgr* SceneMgr::CreateInstance()
{
    if(instance == 0)
    {
        instance = new SceneMgr();
    }
    return instance;
}

void SceneMgr::DestroyInstance()
{
    if(instance != 0)
    {
        delete instance;
        instance = 0;
    }
}

void SceneMgr::SetPerspectiveCamera(GLKVector3 position, GLKVector3 lookAt, GLKVector3 up,
                                    float fovyRadians, float aspect, float nearZ, float farZ)
{
    cameraPosition = position;
    
    baseModelViewMatrix = GLKMatrix4MakeLookAt(position.x, position.y, position.z,
                                               lookAt.x, lookAt.y, lookAt.z,
                                               up.x, up.y, up.z);
    
    baseModelMatrix = GLKMatrix4MakeTranslation(-position.x, -position.y, -position.z);
    
    GLKVector3 negLookDir = GLKVector3Normalize(GLKVector3Subtract(position, lookAt));
    GLKVector3 rightDir = GLKVector3Normalize(GLKVector3CrossProduct(up, negLookDir));
    GLKVector3 fixedUpDir = GLKVector3CrossProduct(negLookDir, rightDir);
    baseViewMatrix = GLKMatrix4Transpose(GLKMatrix4Make(rightDir.x, rightDir.y, rightDir.z, 0,
                                                        fixedUpDir.x, fixedUpDir.y, fixedUpDir.z, 0,
                                                        negLookDir.x, negLookDir.y, negLookDir.z, 0,
                                                        0, 0, 0, 1));
    
    GLKMatrix4 testmodleView = GLKMatrix4Multiply(baseViewMatrix, baseModelMatrix);
    
    baseProjectionMatrix = GLKMatrix4MakePerspective(fovyRadians, aspect, nearZ, farZ);
    
    baseMVPMatrix = GLKMatrix4Invert(baseModelViewMatrix, 0);
}

void SceneMgr::AddEntity(Entity* entity)
{
    bool found = false;
    
    std::list<Entity*>::iterator iterBegin = entityList.begin();
    std::list<Entity*>::iterator iterEnd = entityList.end();
    while(iterBegin != iterEnd)
    {
        if((*iterBegin) == entity)
        {
            found = true;
            break;
        }
        iterBegin++;
    }
    
    if(!found)
    {
        entityList.push_back(entity);
    }
}

void SceneMgr::DeleteEntity(Entity** entity)
{
    bool found = false;
    
    std::list<Entity*>::iterator iterBegin = entitiesToRemove.begin();
    std::list<Entity*>::iterator iterEnd = entitiesToRemove.end();
    while(iterBegin != iterEnd)
    {
        if(*entity == *iterBegin)
        {
            found = true;
            break;
        }
        iterBegin++;
    }
    
    if(!found)
    {
        entitiesToRemove.push_back(*entity);
    }
    
    *entity = 0;
}

void SceneMgr::ClearEntity()
{
    std::list<Entity*>::iterator iterBegin = entityList.begin();
    std::list<Entity*>::iterator iterEnd = entityList.end();
    while(iterBegin != iterEnd)
    {
        delete *iterBegin;
        iterBegin++;
    }
    entityList.clear();
}

Entity* SceneMgr::AddCubEntity(const char* name, GLKVector3 pos, GLKVector3 scale, GLKQuaternion rot, GLKVector4 color, bool isStatic)
{
    Entity* entity = new Entity();
    
    entity->name = name;
    
    entity->Init(TestData::cubeVertices, 108, TestData::cubeNormals, 108, TestData::cubeHull, 24);
    entity->position = pos;
    entity->scale = scale;
    entity->rotation = rot;
    
    entity->baseColor = color;
    
    float linearFrictionCoefficient = 0.3;

    //注意我们的cube认为是单位长度的 1 //
    float len = entity->scale.x * 1;
    float hei = entity->scale.y * 1;
    float wid = entity->scale.z * 1;
    
    float mass = entity->rigidBody->isStatic ? 99999999.0f : 1;
    Matrix4 cubeInertia(mass*0.083333f * (hei*hei + wid*wid),0,0,0,
                        0,mass*0.083333f * (len*len + wid*wid),0,0,
                        0,0,mass*0.083333f * (len*len + hei*hei),0,
                        0,0,0,1);
    entity->rigidBody->SetPhysicallParams(isStatic, mass, cubeInertia, linearFrictionCoefficient);
    
    AddEntity(entity);
    
    return entity;
}

void SceneMgr::SetupLight(int index, GLKVector3 pos, GLKVector3 diffuse, GLKVector3 specular)
{
    if(index > -1 && index < LIGHT_COUNT)
    {
        lightWorldPos[index] = pos;
        lightDiffuse[index] = diffuse;
        lightSpecular[index] = specular;
    }
}

void SceneMgr::SetLightEnable(int index, bool enable)
{
    if(index > -1 && index < LIGHT_COUNT)
    {
        lightsOn[index] = enable;
    }
}

void SceneMgr::UpdateFrameEnd()
{
    std::list<Entity*>::iterator iterBegin = entitiesToRemove.begin();
    std::list<Entity*>::iterator iterEnd = entitiesToRemove.end();
    while(iterBegin != iterEnd)
    {
        entityList.remove(*iterBegin);
        
        if(PhysicsRoutine::IsValid()) PhysicsRoutine::GetInstance()->RemoveRelatedManifold(*iterBegin);
        
        delete *iterBegin;
        iterBegin++;
    }
    
    entitiesToRemove.clear();
}























