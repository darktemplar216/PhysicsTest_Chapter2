//
//  Shader.vsh
//  TestFreeLicense
//
//  Created by TaoweisMac on 2017/3/3.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

attribute vec3 position;

attribute vec3 normal;

varying lowp vec4 colorVarying;

uniform mat4 modelMatrix;

uniform mat4 viewMatrix;

uniform mat4 projectionMatrix;

uniform mat4 invMV;

uniform vec3 light1Pos;
uniform vec3 light1Diffuse;
uniform vec3 light1Specular;

uniform vec3 light2Pos;
uniform vec3 light2Diffuse;
uniform vec3 light2Specular;

uniform vec3 light3Pos;
uniform vec3 light3Diffuse;
uniform vec3 light3Specular;

uniform vec3 eyePos;

const float shiness = 2.0;

uniform vec4 ambientMaterialColor;

void main()
{
    mat4 mv = viewMatrix * modelMatrix;
    mat4 mvp = projectionMatrix * viewMatrix * modelMatrix;
    mat3 modelNoTrans = mat3(modelMatrix[0][0], modelMatrix[0][1], modelMatrix[0][2],
                             modelMatrix[1][0], modelMatrix[1][1], modelMatrix[1][2],
                             modelMatrix[2][0], modelMatrix[2][1], modelMatrix[2][2]);
    
    vec3 o_normal = normalize(modelNoTrans * normal);
    vec3 o_toLight = normalize(light1Pos - (modelMatrix * vec4(position, 1)).xyz);
    vec3 o_toCamera	= normalize(eyePos - (modelMatrix * vec4(position, 1)).xyz);
    
    colorVarying.rgb = ambientMaterialColor.rgb * clamp(dot(o_normal, o_toLight), 0.0, 1.0);
    colorVarying.a = ambientMaterialColor.a;
    
    gl_Position = mvp * vec4(position, 1);
}
