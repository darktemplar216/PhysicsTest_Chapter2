//
//  Entity.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "Entity.hpp"
#include "SceneMgr.hpp"
#include "VBO.hpp"
#include "ShaderProgram.hpp"


Entity::Entity()
{
    position.x = 0; position.y = 0; position.z = 0;
    scale.x = 1;scale.y = 1;scale.z = 1;
    rotation = GLKQuaternionMakeWithAngleAndAxis(0, 0, 1, 0);
    
    baseColor = GLKVector4Make(1, 1, 1, 1);
}

Entity::~Entity()
{
    Uninit();
}

void Entity::Init(const float* vertices, int verticesCount,
          const float* normals, int normalCount,
          const float* hull, int hullCount)
{
    Uninit();
    
    vertexVBO = new VBO();
    if(!vertexVBO->Init(vertices, verticesCount))
    {
        delete vertexVBO;
        vertexVBO = 0;
    }
    
    normalVBO = new VBO();
    if(!normalVBO->Init(normals, normalCount))
    {
        delete normalVBO;
        normalVBO = 0;
    }
    
    rigidBody = new RigidBody();
    rigidBody->Init(name, hull, hullCount);
}

void Entity::Uninit()
{
    if(vertexVBO != 0)
    {
        delete vertexVBO;
        vertexVBO = 0;
    }
    
    if(normalVBO != 0)
    {
        delete normalVBO;
        normalVBO = 0;
    }
    
    if(rigidBody != 0)
    {
        delete rigidBody;
        rigidBody = 0;
    }
}

void Entity::Draw(const ShaderProgram* program)
{
    if(SceneMgr::IsValid())
    {
        SceneMgr* sceneMgr = SceneMgr::GetInstance();
        
        GLint locOfUniformModelMatrix = -1;
        GLint locOfUniformViewMatrix = -1;
        GLint locOfUniformProjectionMatrix = -1;
        GLint locOfUniformAmbientMatColor = -1;
        GLint locOfInvMV = -1;
        
        GLint locOfAttrPosition = -1;
        GLint locOfAttrNormal = -1;
        
        if(program->GetLocByName("modelMatrix", SPLNT_Uniform, locOfUniformModelMatrix)
           && program->GetLocByName("viewMatrix", SPLNT_Uniform, locOfUniformViewMatrix)
           && program->GetLocByName("projectionMatrix", SPLNT_Uniform, locOfUniformProjectionMatrix)
           && program->GetLocByName("ambientMaterialColor", SPLNT_Uniform, locOfUniformAmbientMatColor)
           && program->GetLocByName("invMV", SPLNT_Uniform, locOfInvMV)
           
           && program->GetLocByName("position", SPLNT_Attribute, locOfAttrPosition)
           && program->GetLocByName("normal", SPLNT_Attribute, locOfAttrNormal)
           )
        {
            //model view projection
            GLKMatrix4 tranMatrix = GLKMatrix4MakeTranslation(position.x, position.y, position.z);
            GLKMatrix4 rotMatrix = GLKMatrix4MakeWithQuaternion(rotation.toGLKQuaternion());
            GLKMatrix4 scaleMatrix = GLKMatrix4MakeScale(scale.x, scale.y, scale.z);
            
            GLKMatrix4 curModelMatrix = GLKMatrix4Multiply(tranMatrix, GLKMatrix4Multiply(rotMatrix, scaleMatrix));
            curModelMatrix = GLKMatrix4Multiply(sceneMgr->baseModelMatrix, curModelMatrix);
            
            glUniformMatrix4fv(locOfUniformModelMatrix, 1, 0, curModelMatrix.m);
            glUniformMatrix4fv(locOfUniformViewMatrix, 1, 0, sceneMgr->baseViewMatrix.m);
            glUniformMatrix4fv(locOfUniformProjectionMatrix, 1, 0, sceneMgr->baseProjectionMatrix.m);
            
            GLKMatrix4 invMV = GLKMatrix4Invert(GLKMatrix4Multiply(sceneMgr->baseViewMatrix, curModelMatrix), 0);
            glUniformMatrix4fv(locOfInvMV, 1, 0, invMV.m);

            //position
            glBindBuffer(GL_ARRAY_BUFFER, vertexVBO->loc);
            glEnableVertexAttribArray(locOfAttrPosition);
            glVertexAttribPointer(locOfAttrPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
            
            //normal
            glBindBuffer(GL_ARRAY_BUFFER, normalVBO->loc);
            glEnableVertexAttribArray(locOfAttrNormal);
            glVertexAttribPointer(locOfAttrNormal, 3, GL_FLOAT, GL_FALSE, 0, 0);
            
            //color
            glUniform4fv(locOfUniformAmbientMatColor, 1, baseColor.v);

            glDrawArrays(GL_TRIANGLES, 0, vertexVBO->verticesCount);
            
            //position
            glDisableVertexAttribArray(locOfAttrPosition);
            //normal
            glDisableVertexAttribArray(locOfAttrNormal);
            
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else
        {
            NSLog(@"Entity::Draw: ShaderProgram invalid ");
        }
    }
    else
    {
        NSLog(@"Entity::Draw: SceneMgr invalid ");
    }

}









