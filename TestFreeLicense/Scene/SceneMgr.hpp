//
//  SceneMgr.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef SceneMgr_hpp
#define SceneMgr_hpp

#include <stdio.h>
#include "G_CommonDef.h"
#include "GMath.h"

#define LIGHT_COUNT 3

class Entity;
class RenderRoutine;
class PhysicsRoutine;

class SceneMgr
{
    friend class RenderRoutine;
    
private:
    
    std::list<Entity*> m_entitiesToRemove;
    std::list<Entity*> m_entityList;
    
    RenderRoutine* m_renderRoutine = nullptr;
    
    // 要注意 m_physicsRoutine 可能属于另外一个线程，后面的章节会用到
    PhysicsRoutine* m_multiThreadPhysicsRoutine = nullptr;

public:
    
    Vector3 cameraPosition;
    Vector3 cameraForward;
    Vector3 cameraUp;
    Vector3 cameraRight;
    
    GLKMatrix4 baseProjectionMatrix;
    GLKMatrix4 baseModelMatrix;
    GLKMatrix4 baseViewMatrix;
    
    void SetPerspectiveCamera(GLKVector3 position, GLKVector3 lookAt, GLKVector3 up,
                              float fovyRadians, float aspect, float nearZ, float farZ);
    
public:
    
    SceneMgr();
    
    virtual ~SceneMgr();
    
    void AddEntity(Entity* entity);
    
    void DeleteEntity(Entity** entity);
    
    void ClearEntity();
    
    void SetRenderRoutine(RenderRoutine* routine);
    
    void SetMultiThreadPhysicsRoutine(PhysicsRoutine* routine);
    
    PhysicsRoutine* GetMultiThreadPhysicesRoutine();
    
    std::list<Entity*>& GetEntityList();
    
public:
    
    void Update(float deltaTime, long frameCount);
    
    void UpdateFrameEnd();
    
    void Render();
    
public:
    
    bool lightsOn[LIGHT_COUNT];
    GLKVector3 lightWorldPos[LIGHT_COUNT];
    GLKVector3 lightDiffuse[LIGHT_COUNT];
    GLKVector3 lightSpecular[LIGHT_COUNT];
    
    void SetupLight(int index, GLKVector3 pos, GLKVector3 diffuse, GLKVector3 specular);
    void SetLightEnable(int index, bool enable);
    
public:
    
    static Entity* GenerateCubEntity(const char* name, GLKVector3 pos, GLKVector3 scale, GLKQuaternion rot, GLKVector4 color);
    
};



#endif /* SceneMgr_hpp */
