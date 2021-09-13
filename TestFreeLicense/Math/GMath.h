//
//  GMath.h
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/5/29.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef GMath_h
#define GMath_h

/*
#include "Matrix3.h"
#include "Matrix4.h"
#include "MatrixMathUtil.h"
#include "NumberUtils.h"
#include "Quaternion.h"
#include "GLKMatrix4.h"
#include "Vector3.h"
 */

#include "G_CommonDef.h"

#include <GLKit/GLKMath.h>

#include "GMathCommon.h"

#include "Vector3.h"

#include "Matrix4.h"

#include "Quaternion.h"

extern const GLKVector4 sCubeColor;

extern const GLKVector4 sFloorColor;

extern const GLKVector4 sDormantCubeColor;

extern const Vector3 sVec3Zero;

extern const Vector3 sVec3One;

extern const Vector3 sVec3AxisX;

extern const Vector3 sVec3AxisY;

extern const Vector3 sVec3AxisNegY;

extern const Vector3 sVec3AxisZ;

extern const Vector3 sVec3Gravity;


extern const GLKVector3 sGLKVec3Zero;

extern const GLKVector3 sGLKVec3One;

extern const GLKVector3 sGLKVec3AxisX;

extern const GLKVector3 sGLKVec3AxisY;

extern const GLKVector3 sGLKVec3AxisNegY;

extern const GLKVector3 sGLKVec3AxisX;

extern const Quaternion IdentityQuaternion;


#define G_ACC 9.80665f

#define G_ZERO_HEIGHT -300.0f

#define EPSILON 0.0001f

GLKQuaternion GLKQuaternionMultiplyVector3(const GLKQuaternion& q, const GLKVector3& v);

GLKQuaternion GLKQuaternionMultiplyScalar(const GLKQuaternion& q, float s);

void LogGLKVector3(const GLKVector3& vec);

void LogGLKQuaternion(const GLKQuaternion& vec);
























#endif /* GMath_h */
