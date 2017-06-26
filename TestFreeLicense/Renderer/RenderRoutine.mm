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


RenderRoutine* RenderRoutine::instance = 0;


RenderRoutine::RenderRoutine():cubeShaderProgram(0)
{
    Init();
}

RenderRoutine::~RenderRoutine()
{
    Uninit();
}

bool RenderRoutine::IsValid()
{
    return instance != 0;
}

RenderRoutine* RenderRoutine::GetInstance()
{
    return instance;
}

RenderRoutine* RenderRoutine::CreateInstance()
{
    if(instance == 0)
    {
        instance = new RenderRoutine();
    }
    return instance;
}

void RenderRoutine::DestroyInstance()
{
    if(instance != 0)
    {
        delete instance;
        instance = 0;
    }
}

void RenderRoutine::Init()
{
    NSString* vertShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"];
    NSString* fragShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"];
    
    const char* fragShaderSrc = (const char *)[[NSString stringWithContentsOfFile:fragShaderPathname encoding:NSUTF8StringEncoding error:nil] UTF8String];
    const char* vertexShaderSrc = (const char *)[[NSString stringWithContentsOfFile:vertShaderPathname encoding:NSUTF8StringEncoding error:nil] UTF8String];
    
    cubeShaderProgram = new ShaderProgram();
    if(!cubeShaderProgram->Init(fragShaderSrc, vertexShaderSrc))
    {
        delete cubeShaderProgram;
        cubeShaderProgram = 0;
        
        NSLog(@"RenderRoutine::Init: Failed to init cube shader");
    }
}

void RenderRoutine::Uninit()
{
    if(cubeShaderProgram != 0)
    {
        delete cubeShaderProgram;
        cubeShaderProgram = 0;
    }
}

void RenderRoutine::Update(double deltaTime)
{
    
}

void RenderRoutine::PipelineGo()
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
    
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    
    if(sceneMgr != 0)
    {
        GLint locOfUniformLight1Pos = -1;
        GLint locOfUniformLight1Diffuse = -1;
        GLint locOfUniformLight1Specular = -1;
        GLint locOfUniformEyePos = -1;
        
        if(sceneMgr->lightsOn[0]
           && program->GetLocByName("light1Pos", SPLNT_Uniform, locOfUniformLight1Pos)
           && program->GetLocByName("light1Diffuse", SPLNT_Uniform, locOfUniformLight1Diffuse)
           && program->GetLocByName("light1Specular", SPLNT_Uniform, locOfUniformLight1Specular)
           && program->GetLocByName("eyePos", SPLNT_Uniform, locOfUniformEyePos))
        {
            ret = true;
            glUniform3fv(locOfUniformLight1Pos, 1, GLKMatrix4MultiplyVector3(sceneMgr->baseModelViewMatrix, sceneMgr->lightWorldPos[0]).v);
            glUniform3fv(locOfUniformLight1Diffuse, 1, sceneMgr->lightDiffuse[0].v);
            glUniform3fv(locOfUniformLight1Specular, 1, sceneMgr->lightSpecular[0].v);
            glUniform3fv(locOfUniformEyePos, 1, GLKMatrix4MultiplyVector3(sceneMgr->baseModelViewMatrix, sceneMgr->cameraPosition).v);
        }
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
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    
    if(SceneMgr::IsValid())
    {
        if(cubeShaderProgram != 0)
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
            
            cubeShaderProgram->EnableShader();
            
            LightsOn(cubeShaderProgram);

            std::list<Entity*>::iterator iterBegin = sceneMgr->entityList.begin();
            std::list<Entity*>::iterator iterEnd = sceneMgr->entityList.end();
            while(iterBegin != iterEnd)
            {
                Entity* entity = *iterBegin;
                if(isPassTransparent == fabs(entity->baseColor.a - 1.0f) > 0.001f )
                {
                    entity->Draw(cubeShaderProgram);
                }
                iterBegin++;
            }
            
            LightsOff(cubeShaderProgram);
            
            cubeShaderProgram->DisableShader();
            
            glDisable(GL_BLEND);
        }
        else
        {
            NSLog(@"RenderRoutine::GroupEntitiesGo: cubeShaderProgram invalid");
        }
    }
    else
    {
        NSLog(@"RenderRoutine::GroupEntitiesGo: SceneMgr invalid");
    }
}

void RenderRoutine::GroupRigidBodiesGo()
{
    
}

















