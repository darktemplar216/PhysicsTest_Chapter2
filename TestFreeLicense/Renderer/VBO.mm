//
//  VBO.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#include "VBO.hpp"

VBO::VBO()
{
    
}

VBO::~VBO()
{
    UnInit();
}

bool VBO::Init(const float* buffer, int count)
{
    bool ret = false;
    
    UnInit();
    
    if(buffer != 0 && count > 0)
    {
        glGenBuffers(1, &loc);
        glBindBuffer(GL_ARRAY_BUFFER, loc);
        glBufferData(GL_ARRAY_BUFFER, count * sizeof(float), buffer, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        
        //先写死，认为一定是glDrawArray//
        verticesCount = count / 3;
        ret = true;
    }
    
    return ret;
}

void VBO::UnInit()
{
    if(loc != 0)
    {
        glDeleteBuffers(1, &loc);
        loc = 0;
    }
}
