//
//  ShaderProgram.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "ShaderProgram.hpp"

ShaderProgram::ShaderProgram()
{
    
}

ShaderProgram::~ShaderProgram()
{
    UnInit();
}

bool ShaderProgram::Init(const char* fragmentShaderSrc, const char* vertexShaderSrc)
{
    bool ret = true;
    
    UnInit();

    // Create shader program.
    programLoc = glCreateProgram();
    
    if(ret)
    {
        if (!CompileShader(vertexShaderLoc, GL_VERTEX_SHADER, vertexShaderSrc))
        {
            NSLog(@"ShaderProgram::Init: Failed to compile vertex shader");
            ret = false;
        }
    }
    
    if(ret)
    {
        if (!CompileShader(fragShaderLoc, GL_FRAGMENT_SHADER, fragmentShaderSrc))
        {
            NSLog(@"ShaderProgram::Init: Failed to compile fragment shader");
            ret = false;
        }
    }
    
    if(ret)
    {
        glAttachShader(programLoc, vertexShaderLoc);
        glAttachShader(programLoc, fragShaderLoc);
        
        ret = LinkProgram();
        
        if (vertexShaderLoc) {
            glDetachShader(programLoc, vertexShaderLoc);
            glDeleteShader(vertexShaderLoc);
            vertexShaderLoc = 0;
        }
        if (fragShaderLoc) {
            glDetachShader(programLoc, fragShaderLoc);
            glDeleteShader(fragShaderLoc);
            fragShaderLoc = 0;
        }
    }
    
    if(!ret)
    {
        UnInit();
    }
    else
    {
        PrepareBaseProperties();
    }
    
    return ret;
}

void ShaderProgram::UnInit()
{
    mapNameAndLocs.clear();
    
    if (vertexShaderLoc) {
        glDetachShader(programLoc, vertexShaderLoc);
        glDeleteShader(vertexShaderLoc);
        vertexShaderLoc = 0;
    }
    if (fragShaderLoc) {
        glDetachShader(programLoc, fragShaderLoc);
        glDeleteShader(fragShaderLoc);
        fragShaderLoc = 0;
    }
    if(programLoc != 0)
    {
        glDeleteProgram(programLoc);
        programLoc= 0;
    }
}

bool ShaderProgram::CompileShader(GLuint& loc, GLenum shaderType, const GLchar* source)
{
    bool ret = true;
    
    if(source != 0)
    {
        GLint status;
        
        loc = glCreateShader(shaderType);
        glShaderSource(loc, 1, &source, NULL);
        glCompileShader(loc);
        
#if defined(DEBUG)
        GLint logLength;
        glGetShaderiv(loc, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetShaderInfoLog(loc, logLength, &logLength, log);
            NSLog(@"ShaderProgram::Init: Shader compile log:\n%s", log);
            free(log);
        }
#endif
        
        glGetShaderiv(loc, GL_COMPILE_STATUS, &status);
        if (status == 0) {
            glDeleteShader(loc);
            ret = false;
        }
    }
    
    return ret;
}

bool ShaderProgram::LinkProgram()
{
    bool ret = true;
    
    GLint status;
    glLinkProgram(programLoc);
    
#if defined(DEBUG)
    GLint logLength;
    glGetProgramiv(programLoc, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetProgramInfoLog(programLoc, logLength, &logLength, log);
        NSLog(@"ShaderProgram::Init: Program link log:\n%s", log);
        free(log);
    }
#endif
    
    glGetProgramiv(programLoc, GL_LINK_STATUS, &status);
    if (status == 0) {
        ret = false;
    }
    
    return ret;
}

bool ShaderProgram::FetchLocByName(const std::string& name, ShaderProgramLocNameType type, GLint& loc)
{
    bool ret = false;
    std::map<std::string, GLint>::iterator finder = mapNameAndLocs.find(name);
    if(finder != mapNameAndLocs.end())
    {
        if(finder->second != 0)
        {
            loc = finder->second;
            ret = true;
        }
        else
        {
            loc = 0;
            ret = false;
        }
    }
    else
    {
        switch(type)
        {
            case SPLNT_Attribute:
            {
                ret = _FetchLocByNameForAttribute(name, loc);
                break;
            }
            case SPLNT_Uniform:
            {
                ret = _FetchLocByNameForUniform(name, loc);
                break;
            }
            default:
            {
                ret = false;
                NSLog(@"PShaderProgram::FetchLocByName: wrong ShaderProgramLocNameType -> %d for name -> %s ", type, name.c_str());
                break;
            }
        }
    }
    
    return ret;
}

bool ShaderProgram::GetLocByName(const std::string& name, ShaderProgramLocNameType type, GLint& loc) const
{
    bool ret = false;
    std::map<std::string, GLint>::const_iterator finder = mapNameAndLocs.find(name);
    if(finder != mapNameAndLocs.end())
    {
        if(finder->second >= 0)
        {
            loc = finder->second;
            ret = true;
        }
    }
    return ret;
}

bool ShaderProgram::_FetchLocByNameForAttribute(const std::string& name, GLint& loc)
{
    loc = glGetAttribLocation(programLoc, name.c_str());
    if(loc >= 0)
    {
        mapNameAndLocs.insert(std::pair<std::string, GLuint>(name, loc));
        return true;
    }
    return false;
}

bool ShaderProgram::_FetchLocByNameForUniform(const std::string& name, GLint& loc)
{
    loc = glGetUniformLocation(programLoc, name.c_str());
    if(loc >= 0)
    {
        mapNameAndLocs.insert(std::pair<std::string, GLuint>(name, loc));
        return true;
    }
    return false;
}

void ShaderProgram::PrepareBaseProperties()
{
    GLint dummyInt = 0;
    if(!FetchLocByName("position", SPLNT_Attribute, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Attribute %s not found ", "position");
    }
    
    if(!FetchLocByName("normal", SPLNT_Attribute, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Attribute %s not found ", "normal");
    }
    
    if(!FetchLocByName("modelMatrix", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "modelMatrix");
    }
    
    if(!FetchLocByName("viewMatrix", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "viewMatrix");
    }
    
    if(!FetchLocByName("projectionMatrix", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "projectionMatrix");
    }
    
    if(!FetchLocByName("light1Pos", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "light1Pos");
    }
    
    if(!FetchLocByName("light1Diffuse", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "light1Diffuse");
    }
    
    if(!FetchLocByName("light1Specular", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "light1Specular");
    }
    
    if(!FetchLocByName("eyePos", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "eyePos");
    }
    
    if(!FetchLocByName("invMV", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "invMV");
    }
    
    if(!FetchLocByName("ambientMaterialColor", SPLNT_Uniform, dummyInt))
    {
        NSLog(@"PShaderProgram::PrepareBaseProperties: SPLNT_Uniform %s not found ", "ambientMaterialColor");
    }
    
    
}

void ShaderProgram::EnableShader()
{
    if(programLoc != 0)
        glUseProgram(programLoc);
}

void ShaderProgram::DisableShader()
{
    if(programLoc != 0)
        glUseProgram(0);
}












