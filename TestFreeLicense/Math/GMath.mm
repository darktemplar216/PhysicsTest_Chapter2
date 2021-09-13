//
//  GMath.cpp
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/31.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#include <stdio.h>
#import <Foundation/Foundation.h>

#include "GMath.h"

const GLKVector4 sFloorColor{224.0f / 255.0f, 134.0f / 225.0f, 0, 0.7f};

const GLKVector4 sCubeColor{0.2, 0.7, 0.2, 1.0f};

const GLKVector4 sDormantCubeColor{0.2, 0.2, 0.8, 1};

const Vector3 sVec3Zero(0, 0, 0);

const Vector3 sVec3One(1, 1, 1);

const Vector3 sVec3AxisX(1, 0, 0);

const Vector3 sVec3AxisY(0, 1, 0);

const Vector3 sVec3AxisNegY(0, -1, 0);

const Vector3 sVec3AxisZ(0, 0, 1);

extern const Vector3 sVec3Gravity(0, -98.0f, 0);

const GLKVector3 sGLKVec3Zero{0, 0, 0};

const GLKVector3 sGLKVec3One{1, 1, 1};

const GLKVector3 sGLKVec3AxisX{1, 0, 0};

const GLKVector3 sGLKVec3AxisY{0, 1, 0};

const GLKVector3 sGLKVec3AxisNegY{0, -1, 0};

const GLKVector3 sGLKVec3AxisZ{0, 0, 1};

const Quaternion IdentityQuaternion(1, 0, 0, 0);


GLKQuaternion GLKQuaternionMultiplyVector3(const GLKQuaternion& q, const GLKVector3& v)
{
    return GLKQuaternionMake(q.w*v.x + q.y*v.z - q.z*v.y, q.w*v.y + q.z*v.x - q.x*v.z, q.w*v.z + q.x*v.y - q.y*v.x, -(q.x*v.x + q.y*v.y + q.z*v.z));
}

GLKQuaternion GLKQuaternionMultiplyScalar(const GLKQuaternion& q, float s)
{
    return GLKQuaternionMake(q.x*s, q.y*s, q.z*s, q.w*s);
}

void LogGLKVector3(const GLKVector3& vec)
{
    NSLog(@"GLKVector3: x -> %f, y -> %f, z -> %f", vec.x, vec.y, vec.z);
}

void LogGLKQuaternion(const GLKQuaternion& vec)
{
    NSLog(@"GLKVector3: x -> %f, y -> %f, z -> %f, w -> %f ", vec.x, vec.y, vec.z, vec.w);
}















