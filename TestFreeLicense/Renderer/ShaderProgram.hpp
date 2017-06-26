//
//  ShaderProgram.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef ShaderProgram_hpp
#define ShaderProgram_hpp

#include "G_CommonDef.h"
#include <OpenGLES/ES2/glext.h>

enum ShaderProgramLocNameType
{
    SPLNT_Attribute,
    SPLNT_Uniform
};

class ShaderProgram
{
public:
    GLuint programLoc = 0;
    GLuint vertexShaderLoc = 0;
    GLuint fragShaderLoc = 0;
    
    ShaderProgram();
    ~ShaderProgram();
    
    bool Init(const char* fragShaderPath, const char* vertexShaderPath);
    void UnInit();
    
    bool FetchLocByName(const std::string& name, ShaderProgramLocNameType type, GLint& loc);
    
    bool GetLocByName(const std::string& name, ShaderProgramLocNameType type, GLint& loc) const;
    
    void PrepareBaseProperties();
    
    void EnableShader();
    void DisableShader();
    
private:
    std::map<std::string, GLint> mapNameAndLocs;
    
    bool _FetchLocByNameForAttribute(const std::string& name, GLint& loc);
    
    bool _FetchLocByNameForUniform(const std::string& name, GLint& loc);
    
    bool CompileShader(GLuint& loc, GLenum shaderType, const GLchar* content);
    
    bool LinkProgram();
};

#endif /* ShaderProgram_hpp */
