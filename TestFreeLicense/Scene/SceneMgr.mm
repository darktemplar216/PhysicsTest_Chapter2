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
#include "RenderRoutine.hpp"
#include "PhysicsRoutine.hpp"

SceneMgr::SceneMgr()
{
    memset(lightsOn, 0, sizeof(lightsOn));
    memset(lightWorldPos, 0, sizeof(lightWorldPos));
    memset(lightDiffuse, 0, sizeof(lightDiffuse));
    memset(lightSpecular, 0, sizeof(lightSpecular));
        
    SetPerspectiveCamera(GLKVector3Make(0, 0, -1),
                         GLKVector3Make(0, 0, 0),
                         GLKVector3Make(0, 1, 0),
                         GLKMathDegreesToRadians(45.0f),
                         1,
                         0.1f,
                         100.0f);
}

SceneMgr::~SceneMgr()
{
    ClearEntity();
    
    delete m_renderRoutine;
    delete m_multiThreadPhysicsRoutine;
}

void SceneMgr::SetPerspectiveCamera(GLKVector3 position, GLKVector3 lookAt, GLKVector3 up,
                                    float fovyRadians, float aspect, float nearZ, float farZ)
{
    
    /*
    baseModelViewMatrix = GLKMatrix4MakeLookAt(position.x, position.y, position.z,
                                               lookAt.x, lookAt.y, lookAt.z,
                                               up.x, up.y, up.z);
    */
    baseModelMatrix = GLKMatrix4MakeTranslation(-position.x, -position.y, -position.z);
    
    GLKVector3 negLookDir = GLKVector3Normalize(GLKVector3Subtract(position, lookAt));
    GLKVector3 rightDir = GLKVector3Normalize(GLKVector3CrossProduct(up, negLookDir));
    GLKVector3 fixedUpDir = GLKVector3CrossProduct(negLookDir, rightDir);
    baseViewMatrix = GLKMatrix4Transpose(GLKMatrix4Make(rightDir.x, rightDir.y, rightDir.z, 0,
                                                        fixedUpDir.x, fixedUpDir.y, fixedUpDir.z, 0,
                                                        negLookDir.x, negLookDir.y, negLookDir.z, 0,
                                                        0, 0, 0, 1));
    
    cameraPosition = position;
    cameraForward.x = -negLookDir.x; cameraForward.y = -negLookDir.y; cameraForward.z = -negLookDir.z;
    cameraUp = fixedUpDir;
    cameraRight = rightDir;
    
    baseProjectionMatrix = GLKMatrix4MakePerspective(fovyRadians, aspect, nearZ, farZ);
}

void SceneMgr::AddEntity(Entity* entity)
{
    bool found = false;
    
    std::list<Entity*>::iterator iterBegin = m_entityList.begin();
    std::list<Entity*>::iterator iterEnd = m_entityList.end();
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
        m_entityList.push_back(entity);
    }
    
    entity->m_sceneMgr = this;
}

void SceneMgr::DeleteEntity(Entity** entity)
{
    bool found = false;
    
    std::list<Entity*>::iterator iterBegin = m_entitiesToRemove.begin();
    std::list<Entity*>::iterator iterEnd = m_entitiesToRemove.end();
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
        m_entitiesToRemove.push_back(*entity);
    }
    
    *entity = nullptr;
}

void SceneMgr::ClearEntity()
{
    std::list<Entity*>::iterator iterBegin = m_entityList.begin();
    std::list<Entity*>::iterator iterEnd = m_entityList.end();
    while(iterBegin != iterEnd)
    {
        delete *iterBegin;
        iterBegin++;
    }
    m_entityList.clear();
}

Entity* SceneMgr::GenerateCubEntity(const char* name, GLKVector3 pos, GLKVector3 scale, GLKQuaternion rot, GLKVector4 color)
{
    Entity* entity = new Entity();
    
    entity->name = name;
    
    entity->InitRenderParam(TestData::cubeVertices, 108, TestData::cubeNormals, 108);
    entity->position = pos;
    entity->scale = scale;
    entity->rotation = rot;
    entity->baseColor = color;
    
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
    std::list<Entity*>::iterator iterBegin = m_entitiesToRemove.begin();
    std::list<Entity*>::iterator iterEnd = m_entitiesToRemove.end();
    while(iterBegin != iterEnd)
    {
        m_entityList.remove(*iterBegin);
        
        delete *iterBegin;
        iterBegin++;
    }
    
    m_entitiesToRemove.clear();
}

void SceneMgr::Update(float deltaTime, long frameCount)
{
    // 注意， 这里以后会改成多线程的
    if(m_multiThreadPhysicsRoutine != nullptr)
    {
        m_multiThreadPhysicsRoutine->Update(deltaTime, frameCount);
        
        std::list<Entity*>::iterator iterBegin = m_entityList.begin();
        std::list<Entity*>::iterator iterEnd = m_entityList.end();
        while(iterBegin != iterEnd)
        {
            Entity* entity = *iterBegin;
            if(entity != nullptr)
            {
                RigidBody* rigidBody = m_multiThreadPhysicsRoutine->GetRigidBody(entity->m_rigidBodyUID);
                if(rigidBody != nullptr)
                {
                    memcpy(entity->m_rigidData, rigidBody->getDataConst(RDI_real), sizeof(RigidData));
                }
                
                entity->position = entity->m_rigidData->m_position;
                entity->rotation = entity->m_rigidData->m_rotation;
                entity->scale = entity->m_rigidData->m_scale;
                
                entity->Update(deltaTime);
            }
            iterBegin++;
        }
    }
}

void SceneMgr::Render()
{
    if(m_renderRoutine == nullptr)
    {
        return;
    }
    
    m_renderRoutine->Render();
}

void SceneMgr::SetRenderRoutine(RenderRoutine* routine)
{
    m_renderRoutine = routine;
    m_renderRoutine->m_sceneMgr = this;
}

void SceneMgr::SetMultiThreadPhysicsRoutine(PhysicsRoutine* routine)
{
    m_multiThreadPhysicsRoutine = routine;
}

PhysicsRoutine* SceneMgr::GetMultiThreadPhysicesRoutine()
{
    return m_multiThreadPhysicsRoutine;
}

std::list<Entity*>& SceneMgr::GetEntityList()
{
    return m_entityList;
}

















