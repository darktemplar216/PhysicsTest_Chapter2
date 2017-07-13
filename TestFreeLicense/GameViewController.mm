//
//  GameViewController.m
//  TestFreeLicense
//
//  Created by TaoweisMac on 2017/3/3.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import "GameViewController.h"
#import <OpenGLES/ES2/glext.h>
#include "SceneMgr.hpp"
#include "RenderRoutine.hpp"
#include "Entity.hpp"
#include "GMath.h"

@interface GameViewController () {
    GLuint _program;
    
    GLKMatrix4 _modelViewProjectionMatrix;
    GLKMatrix3 _normalMatrix;
    float _rotation;
    
    GLuint _vertexArray;
    GLuint _vertexBuffer;
    
    GLKVector2 _beginTouchPos;
    GLKVector2 _lastTouchPos;
    GLKVector2 _deltaTouchPos;
    GLKVector2 _accumTouchPos;
    
    Quaternion _curRotation;
}
@property (strong, nonatomic) EAGLContext *context;

- (void)setupGL;
- (void)tearDownGL;
- (void)initScene;

@end

@implementation GameViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    
    if (!self.context) {
        NSLog(@"Failed to create ES context");
    }
    
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    
    self.preferredFramesPerSecond = 60;
    
    _curRotation = GLKQuaternionIdentity;
    
    [self setupGL];
}

- (void)dealloc
{
    [self tearDownGL];
    
    if ([EAGLContext currentContext] == self.context) {
        [EAGLContext setCurrentContext:nil];
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    
    if ([self isViewLoaded] && ([[self view] window] == nil)) {
        self.view = nil;
        
        [self tearDownGL];
        
        if ([EAGLContext currentContext] == self.context) {
            [EAGLContext setCurrentContext:nil];
        }
        self.context = nil;
    }
    
    // Dispose of any resources that can be recreated.
}

- (BOOL)prefersStatusBarHidden {
    return YES;
}

- (void)setupGL
{
    [EAGLContext setCurrentContext:self.context];
    
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    
    _accumTouchPos = GLKVector2Make(0, 0);
    
    SceneMgr::CreateInstance();
    RenderRoutine::CreateInstance();
    PhysicsRoutine::CreateInstance();
    
    [self initScene];
}

- (void)tearDownGL
{
    SceneMgr::DestroyInstance();
    RenderRoutine::DestroyInstance();
    PhysicsRoutine::DestroyInstance();
    
    [EAGLContext setCurrentContext:self.context];
}

#pragma mark - GLKView and GLKViewController delegate methods

- (void)update
{
    double deltaTime = self.timeSinceLastUpdate;
    
    //test taowei
    deltaTime = 1.0f / 60.0f;
    
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    if(sceneMgr != 0)
    {
        //camera section//
        Vector3 dragAxis = Vector3(_deltaTouchPos.x, _deltaTouchPos.y, 0);
        float dragVal = dragAxis.normalizeF();
        
        Quaternion rotationThisTime(dragVal * deltaTime, dragAxis.cross(sVec3AxisZ));
        _curRotation = rotationThisTime * _curRotation;
        _curRotation.normalize();
        
        GLKVector3 lookAtPos = GLKVector3Make(0, -10, 0);
        
        Vector3 camePosVec = _curRotation.rotateV(Vector3(0, 60, -80));
        
        float aspect = fabs(self.view.bounds.size.width / self.view.bounds.size.height);
        sceneMgr->SetPerspectiveCamera(camePosVec.toGLKVector3(), lookAtPos, sGLKVec3AxisY, GLKMathDegreesToRadians(45.0f), aspect, 0.1f, 4000.0f);
        
        if(PhysicsRoutine::IsValid()) PhysicsRoutine::GetInstance()->Update(deltaTime, self.framesDisplayed);
        
        [self RemoveCubesInTheAbyss];
        
        sceneMgr->UpdateFrameEnd();
    }
    
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    if(RenderRoutine::IsValid()) RenderRoutine::GetInstance()->PipelineGo();
}

#pragma mark -  OpenGL ES 2 shader compilation

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    //if ([touches count] &lt; 1) return;
    CGPoint point = [[touches anyObject] locationInView:[self view]];
    
    _lastTouchPos = GLKVector2Make(point.x, point.y);
    _beginTouchPos = _lastTouchPos;
    _deltaTouchPos = GLKVector2Make(0, 0);
    
    //NSLog(@"_beginTouchPos: x -> %f, y -> %f ", _beginTouchPos.x, _beginTouchPos.y);
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    //if ([touches count] &lt; 1) return;
    CGPoint point = [[touches anyObject] locationInView:[self view]];
    
    GLKVector2 curTouchPos = GLKVector2Make(point.x, point.y);
    //NSLog(@"curTouchPos: x -> %f, y -> %f ", curTouchPos.x, curTouchPos.y);
    _deltaTouchPos = GLKVector2Subtract(curTouchPos, _lastTouchPos);
    
    //NSLog(@"_deltaTouchPos: x -> %f, y -> %f ", _deltaTouchPos.x, _deltaTouchPos.y);
    _accumTouchPos = GLKVector2Add(_accumTouchPos, GLKVector2MultiplyScalar(_deltaTouchPos, 0.005f));
    
    _lastTouchPos = curTouchPos;
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    //if ([touches count] &lt; 1) return;
    CGPoint point = [[touches anyObject] locationInView:[self view]];
    
    _lastTouchPos = GLKVector2Make(point.x, point.y);
    _deltaTouchPos = GLKVector2Make(0, 0);
    
    float disPow = GLKVector2Distance(_lastTouchPos, _beginTouchPos);
    if(disPow < 15.0f)
    {
        [self throwANewCube];
    }
}

- (void)initScene
{
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    
    sceneMgr->SetupLight(0,
                         GLKVector3Make(400, 400, 400),
                         GLKVector3Make(0.3f, 0.7f, 0.7f),
                         GLKVector3Make(1.0f, 1.0f, 1.0f));
    sceneMgr->SetLightEnable(0, true);
    
    Entity* entity = 0;
    
    srand([[NSDate date] timeIntervalSince1970]);
    
    //add floor//
    entity = sceneMgr->AddCubEntity("floor",
                                    GLKVector3Make(0, -10, 0),
                                    GLKVector3Make(50, 2, 50),
                                    GLKQuaternionIdentity,
                                    sFloorColor,//
                                    true);
    /*
     entity = sceneMgr->AddCubEntity("a",
     sVec3Zero,
     sVec3One,
     GLKQuaternionIdentity,
     sCubeColor,
     false);
     
     entity = sceneMgr->AddCubEntity("b",
     GLKVector3Make( 0.2, 4, 0.2),
     sVec3One,
     GLKQuaternionIdentity,
     sCubeColor,
     false);
     */
}

int cubeIndexAccum = 0;

- (void) throwANewCube
{
    SceneMgr* sceneMgr = SceneMgr::GetInstance();
    
    Entity* entity = 0;
    
    float randX = (float)(rand() % 100) * 0.01f;
    float randY = (float)(rand() % 100) * 0.01f;
    float randZ = (float)(rand() % 100) * 0.01f;
    
    float randQX = (float)(rand() % 100) * 0.01f; float signQX = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randQY = (float)(rand() % 100) * 0.01f; float signQY = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randQZ = (float)(rand() % 100) * 0.01f; float signQZ = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randQW = (float)(rand() % 100) * 0.01f;
    
    cubeIndexAccum++;
    
    char cubeName[25] = {0};
    sprintf(cubeName, "cube_%d", cubeIndexAccum);
    /*
     entity = sceneMgr->AddCubEntity(cubeName,
     GLKVector3Make(randX * 25.0f, 0, randZ * 25.0f),
     sVec3One,
     GLKQuaternionMakeWithAngleAndVector3Axis(randQW * 3.0f, GLKVector3Normalize(GLKVector3Make(randQX, randQY, randQZ))), // //GLKQuaternionIdentity
     sCubeColor,
     false);
     */
    entity = sceneMgr->AddCubEntity(cubeName,
                                    GLKVector3Make(randX * 9.0f, 30 + randY * 10.0f, randZ * 9.0f),
                                    GLKVector3Make(6, 6, 6),
                                    GLKQuaternionMakeWithAngleAndVector3Axis(randQW * 3.0f, GLKVector3Normalize(GLKVector3Make(signQX * randQX, signQY * randQY, signQZ * randQZ))), //GLKQuaternionIdentity, // //GLKQuaternionIdentity
                                    sCubeColor,
                                    false);
}

- (void) RemoveCubesInTheAbyss
{
    if(SceneMgr::IsValid())
    {
        SceneMgr* sceneMgr = SceneMgr::GetInstance();
        
        std::list<Entity*>::iterator iterBegin = sceneMgr->entityList.begin();
        std::list<Entity*>::iterator iteBEnd = sceneMgr->entityList.end();
        while(iterBegin != iteBEnd)
        {
            Entity* entity = *iterBegin;
            if(entity->rigidBody != 0)
            {
                if(!entity->rigidBody->isStatic)
                {
                    if(entity->rigidBody->datas[RDI_real].isDormant)
                    {
                        entity->baseColor = sDormantCubeColor;
                    }
                    else
                    {
                        entity->baseColor = sCubeColor;
                    }
                }
            }

            if(entity->position.y < -80)
            {
                
                sceneMgr->DeleteEntity(&entity);
            }
            iterBegin++;
        }
    }
}

@end
