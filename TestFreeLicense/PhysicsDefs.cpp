//
//  PhysicsDefs.cpp
//  PhysicsTest
//
//  Created by taowei on 2021/9/5.
//  Copyright © 2021 TaoweisMac. All rights reserved.
//

#include <stdio.h>
#include "PhysicsDefs.h"

const float PhysicsDefs::StdCubeHullVertices[PhysicsDefs::StdCubeHullVertCount] =
{
    -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, 0.5f,
    0.5f, -0.5f, 0.5f,
    0.5f, -0.5f, -0.5f,
    
    -0.5f, 0.5f, -0.5f,
    -0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, -0.5f,
};
