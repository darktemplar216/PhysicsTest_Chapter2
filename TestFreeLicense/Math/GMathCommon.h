//
//  GMathCommon.h
//  PhysicsTest
//
//  Created by TaoweisMac on 2017/6/4.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#ifndef GMathCommon_h
#define GMathCommon_h

#define EPSILON_FLT	0.00001f
#define EPSILON_PW_FLT EPSILON_FLT * EPSILON_FLT

#define PI_DIVIDE_180_FLT 0.01745329251994329f
#define RADIAN_TO_DEGREE 57.2957795130823799f
#define DEGREE_TO_RADIAN 0.01745329251994327f

#define PI_MUL_2_FLT 6.2831853071795864f
#define PI_FLT 3.1415926535897932f

#define DORMAINT_THRESHOLD EPSILON_FLT

#define PENETRATION_TOLERANCE 0.008f

#define PENETRATION_VELOCITY_BIAS 0.004f

inline bool equal(float a, float b)
{
    float d = a - b;
    return (d < EPSILON_FLT && d > -EPSILON_FLT);
}


#endif /* GMathCommon_h */
