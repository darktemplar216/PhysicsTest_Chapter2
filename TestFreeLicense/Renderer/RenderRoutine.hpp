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
class SceneMgr;

class RenderRoutine
{
    friend class SceneMgr;
    
public:
    
    RenderRoutine();
    virtual ~RenderRoutine();
    
    void InitParams();
    void Uninit();
    
private:
    
    ShaderProgram* m_cubeShaderProgram = nullptr;
    SceneMgr* m_sceneMgr = nullptr;
    
public:
    
    void Update(double deltaTime);
    
    void Render();
    
private:
    
    bool LightsOn(ShaderProgram* program);
    
    bool LightsOff(ShaderProgram* program);
    
    void GroupEntitiesGo();
    
    void GroupRigidBodiesGo();
    
private:
    
    bool isPassTransparent = false;
    
};


#endif /* RenderRoutine_hpp */
