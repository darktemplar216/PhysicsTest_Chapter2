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

class SceneMgr
{
private:
    static SceneMgr* instance;
    
    SceneMgr();
    
public:
    
    ~SceneMgr();
    
    static bool IsValid();
    static SceneMgr* GetInstance();
    static SceneMgr* CreateInstance();
    static void DestroyInstance();
    
private:
    
    std::list<Entity*> entitiesToRemove;

public:
    std::list<Entity*> entityList;
    
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
    
    void AddEntity(Entity* entity);
    
    void DeleteEntity(Entity** entity);
    
    void ClearEntity();
    
    void UpdateFrameEnd();
    
public:
    
    bool lightsOn[LIGHT_COUNT];
    GLKVector3 lightWorldPos[LIGHT_COUNT];
    GLKVector3 lightDiffuse[LIGHT_COUNT];
    GLKVector3 lightSpecular[LIGHT_COUNT];
    
    void SetupLight(int index, GLKVector3 pos, GLKVector3 diffuse, GLKVector3 specular);
    void SetLightEnable(int index, bool enable);
    
public:
    
    Entity* AddCubEntity(const char* name,
                         GLKVector3 pos,
                         GLKVector3 scale,
                         GLKQuaternion rot,
                         GLKVector4 color,
                         bool isStatic);
    
};



#endif /* SceneMgr_hpp */
