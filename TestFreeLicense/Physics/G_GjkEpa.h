/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the
 use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software in a
 product, an acknowledgment in the product documentation would be appreciated
 but is not required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

/*
 GJK-EPA collision solver by Nathanael Presson, 2008
 */
#ifndef _G_GJKEPA_
#define _G_GJKEPA_

#include "GMath.h"
#include "RigidBody.hpp"



///btGjkEpaSolver contributed under zlib by Nathanael Presson
struct	btGjkEpaSolver2
{
    struct	sResults
    {
        enum eStatus
        {
            Separated,		/* Shapes doesnt penetrate												*/
            Penetrating,	/* Shapes are penetrating												*/
            GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
            EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/
        }		status;
        Vector3	witnesses[2];
        Vector3	normal;
        double	distance;
    };
    
    static bool Distance(const PointCloud* shape0,
                         const PointCloud* shape1,
                         const Matrix4* trans0,
                         const Matrix4* trans1,
                         const Vector3& guess,
                         sResults& results);
    
    static bool Penetration(const PointCloud* shape0,
                            const PointCloud* shape1,
                            const Matrix4* trans0,
                            const Matrix4* trans1,
                            const Vector3& guess,
                            sResults& results,
                            bool usemargins=true);
    /*
     static float	SignedDistance(const GLKVector3& position,
     float margin,
     const RigidBody* shape,
     const GLKMatrix4* trans0,
     sResults& results);
     */
    static bool SignedDistance(const PointCloud* shape0,
                               const PointCloud* shape1,
                               const Matrix4* trans0,
                               const Matrix4* trans1,
                               const Vector3& guess,
                               sResults& results);
};

template<typename T>
void btSwap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

template <class T>
const T& btMin(const T& a, const T& b)
{
    return a < b ? a : b ;
}

template <class T>
const T& btMax(const T& a, const T& b)
{
    return  a > b ? a : b;
}

static float btSqrt(float y)
{
    return sqrtf(y);
}

static float btFabs(float x) { return fabsf(x); }

#endif
