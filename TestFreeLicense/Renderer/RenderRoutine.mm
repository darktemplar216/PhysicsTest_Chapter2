//
//  RenderRoutine.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "RenderRoutine.hpp"
#include "SceneMgr.hpp"
#include "Entity.hpp"
#include "ShaderProgram.hpp"

RenderRoutine::RenderRoutine()
{
    InitParams();
}

RenderRoutine::~RenderRoutine()
{
    Uninit();
}

void RenderRoutine::InitParams()
{
    Uninit();
    
    NSString* vertShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"];
    NSString* fragShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"];
    
    const char* fragShaderSrc = (const char *)[[NSString stringWithContentsOfFile:fragShaderPathname encoding:NSUTF8StringEncoding error:nil] UTF8String];
    const char* vertexShaderSrc = (const char *)[[NSString stringWithContentsOfFile:vertShaderPathname encoding:NSUTF8StringEncoding error:nil] UTF8String];
    
    m_cubeShaderProgram = new ShaderProgram();
    if(!m_cubeShaderProgram->Init(fragShaderSrc, vertexShaderSrc))
    {
        delete m_cubeShaderProgram;
        m_cubeShaderProgram = nullptr;
        
        NSLog(@"RenderRoutine::Init: Failed to init cube shader");
    }
}

void RenderRoutine::Uninit()
{
    if(m_cubeShaderProgram != nullptr)
    {
        delete m_cubeShaderProgram;
        m_cubeShaderProgram = nullptr;
    }
}

void RenderRoutine::Update(double deltaTime)
{
    
}

void RenderRoutine::Render()
{
    glClearColor(0.65f, 0.65f, 0.65f, 1.0f);
    glDepthMask(true);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    isPassTransparent = false;
    
    GroupEntitiesGo();
    
    isPassTransparent = true;
    
    GroupEntitiesGo();
    
    isPassTransparent = false;
    
    GroupRigidBodiesGo();
}

bool RenderRoutine::LightsOn(ShaderProgram* program)
{
    bool ret = false;
    
    if(m_sceneMgr == nullptr)
    {
        return ret;
    }
    
    GLint locOfUniformLight1Pos = -1;
    GLint locOfUniformLight1Diffuse = -1;
    GLint locOfUniformLight1Specular = -1;
    GLint locOfUniformEyePos = -1;

    if(m_sceneMgr->lightsOn[0]
       && program->GetLocByName("light1Pos", SPLNT_Uniform, locOfUniformLight1Pos)
       && program->GetLocByName("light1Diffuse", SPLNT_Uniform, locOfUniformLight1Diffuse)
       && program->GetLocByName("light1Specular", SPLNT_Uniform, locOfUniformLight1Specular)
       && program->GetLocByName("eyePos", SPLNT_Uniform, locOfUniformEyePos))
    {
        ret = true;
        glUniform3fv(locOfUniformLight1Pos, 1, m_sceneMgr->lightWorldPos[0].v);
        glUniform3fv(locOfUniformLight1Diffuse, 1, m_sceneMgr->lightDiffuse[0].v);
        glUniform3fv(locOfUniformLight1Specular, 1, m_sceneMgr->lightSpecular[0].v);
        glUniform3fv(locOfUniformEyePos, 1, m_sceneMgr->cameraPosition.v);
    }

    return ret;
}

bool RenderRoutine::LightsOff(ShaderProgram* program)
{
    bool ret = false;
    return ret;
}

void RenderRoutine::GroupEntitiesGo()
{
    if(m_sceneMgr == nullptr)
    {
        return;
    }
    
    if(m_cubeShaderProgram != 0)
    {
        
        if(isPassTransparent)
        {
            /**/
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            
            //glDisable(GL_DEPTH_TEST);
            glDepthMask(false);
            //glDepthFunc(GL_ALWAYS);
        }
        else
        {
            glDisable(GL_BLEND);
            
            /**/
            glEnable(GL_DEPTH_TEST);
            glDepthMask(true);
            glDepthFunc(GL_LESS);
        }
        
        m_cubeShaderProgram->EnableShader();
        
        LightsOn(m_cubeShaderProgram);

        std::list<Entity*>::iterator iterBegin = m_sceneMgr->m_entityList.begin();
        std::list<Entity*>::iterator iterEnd = m_sceneMgr->m_entityList.end();
        while(iterBegin != iterEnd)
        {
            Entity* entity = *iterBegin;
            if(isPassTransparent == fabs(entity->baseColor.a - 1.0f) > 0.001f )
            {
                entity->Draw(m_cubeShaderProgram);
            }
            iterBegin++;
        }
        
        LightsOff(m_cubeShaderProgram);
        
        m_cubeShaderProgram->DisableShader();
        
        glDisable(GL_BLEND);
    }
    else
    {
        NSLog(@"RenderRoutine::GroupEntitiesGo: cubeShaderProgram invalid");
    }
}

void RenderRoutine::GroupRigidBodiesGo()
{
    
}

















