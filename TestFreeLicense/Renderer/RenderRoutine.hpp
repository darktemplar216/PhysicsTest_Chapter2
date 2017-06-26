//
//  RenderRoutine.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef RenderRoutine_hpp
#define RenderRoutine_hpp

#include <stdio.h>
#include "GMath.h"

class ShaderProgram;

class RenderRoutine
{
private:
    static RenderRoutine* instance;
    
    RenderRoutine();
    
public:
    
    ~RenderRoutine();
    
    static bool IsValid();
    static RenderRoutine* GetInstance();
    static RenderRoutine* CreateInstance();
    static void DestroyInstance();
    
    void Init();
    void Uninit();
    
private:
    
    ShaderProgram* cubeShaderProgram;
    
public:
    
    void Update(double deltaTime);
    
    void PipelineGo();
    
private:
    
    bool LightsOn(ShaderProgram* program);
    
    bool LightsOff(ShaderProgram* program);
    
    void GroupEntitiesGo();
    
    void GroupRigidBodiesGo();
    
private:
    
    bool isPassTransparent = false;
    
};


#endif /* RenderRoutine_hpp */
