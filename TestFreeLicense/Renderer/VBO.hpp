//
//  VBO.hpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef VBO_hpp
#define VBO_hpp

#include "G_CommonDef.h"
#include <OpenGLES/ES2/glext.h>

class VBO
{
public:
    
    GLuint loc = 0;
    
    int verticesCount = 0;
    
    VBO();
    ~VBO();
    
    bool Init(const float* buffer, int count);
    void UnInit();
};

#endif /* VBO_hpp */
