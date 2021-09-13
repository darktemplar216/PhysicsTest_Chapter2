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
    delete m_sceneMgr;
    
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
    
    m_sceneMgr = new SceneMgr();
    
    RenderRoutine* renderRoutine = new RenderRoutine();
    renderRoutine->InitParams();
    
    PhysicsRoutine* physicsRoutine = new PhysicsRoutine();
    physicsRoutine->InitParams(sVec3Gravity);
    
    m_sceneMgr->SetRenderRoutine(renderRoutine);
    
    // 注意以后的章节里这个PhysicsRoutine会被改为多线程的
    m_sceneMgr->SetMultiThreadPhysicsRoutine(physicsRoutine);
    
    [self initScene];
}

- (void)tearDownGL
{
    delete m_sceneMgr;
    m_sceneMgr = nullptr;
    
    [EAGLContext setCurrentContext:self.context];
}

#pragma mark - GLKView and GLKViewController delegate methods

- (void)update
{
    double deltaTime = self.timeSinceLastUpdate;
    
    if(m_sceneMgr != nullptr)
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
        m_sceneMgr->SetPerspectiveCamera(camePosVec.toGLKVector3(), lookAtPos, sGLKVec3AxisY, GLKMathDegreesToRadians(45.0f), aspect, 0.1f, 4000.0f);
        
        m_sceneMgr->Update(deltaTime, self.framesDisplayed);
        
        [self RemoveCubesInTheAbyss];
        
        m_sceneMgr->UpdateFrameEnd();
    }
    
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    if(m_sceneMgr != nullptr)
    {
        m_sceneMgr->Render();
    }
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
    m_sceneMgr->SetupLight(0, GLKVector3Make(400, 400, 400), GLKVector3Make(0.3f, 0.7f, 0.7f), GLKVector3Make(1.0f, 1.0f, 1.0f));
    m_sceneMgr->SetLightEnable(0, true);
    
    srand([[NSDate date] timeIntervalSince1970]);
    
    //add floor, 注意 RigidBody 那块以后是多线程的//
    Vector3 pos(0, -10.0f, 0);
    Vector3 scale(50.0f, 2.0f, 50.0f);
    Quaternion rot = IdentityQuaternion;
    
    RigidBody* rigidFloor = new RigidBody();
    rigidFloor->InitParamsAsACube("floor", scale, pos, rot, true, 100.0f, 0.4f, 0.3f);
    m_sceneMgr->GetMultiThreadPhysicesRoutine()->AddRigidBody(rigidFloor);
    
    Entity* entity = SceneMgr::GenerateCubEntity("floor", pos.toGLKVector3(), scale.toGLKVector3(), rot.toGLKQuaternion(), sFloorColor);
    entity->InitPhysicsParam(rigidFloor);
    
    m_sceneMgr->AddEntity(entity);
}

int cubeIndexAccum = 0;

- (void) throwANewCube
{
    float randX = (float)(rand() % 100) * 0.01f;
    float randY = (float)(rand() % 100) * 0.01f;
    float randZ = (float)(rand() % 100) * 0.01f;
    
    float randQX = (float)(rand() % 100); float signQX = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randQY = (float)(rand() % 100); float signQY = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randQZ = (float)(rand() % 100); float signQZ = (float)(rand() % 100) > 50.0f ? 1 : -1;
    float randAngle = (float)(rand() % 360);
    
    cubeIndexAccum++;
    
    char cubeName[25] = {0};
    sprintf(cubeName, "cube_%d", cubeIndexAccum);
    
    Vector3 pos(randX * 9.0f, 30 + randY * 10.0f, randZ * 9.0f);
    Vector3 scale(6, 6, 6);
    Vector3 rotAxis(signQX * randQX, signQY * randQY, signQZ * randQZ);
    rotAxis.normalize();
    Quaternion rot(randAngle, rotAxis);
    
    RigidBody* rigidBox = new RigidBody();
    rigidBox->InitParamsAsACube(cubeName, scale, pos, rot, false, 100.0f, 0.7f, 0.5f);
    m_sceneMgr->GetMultiThreadPhysicesRoutine()->AddRigidBody(rigidBox);
    
    Entity* entity = SceneMgr::GenerateCubEntity(cubeName, pos.toGLKVector3(), scale.toGLKVector3(), rot.toGLKQuaternion(), sCubeColor);
    entity->InitPhysicsParam(rigidBox);
    
    m_sceneMgr->AddEntity(entity);
}

- (void) RemoveCubesInTheAbyss
{
    if(m_sceneMgr != nullptr)
    {
        std::list<Entity*>::iterator iterBegin = m_sceneMgr->GetEntityList().begin();
        std::list<Entity*>::iterator iteBEnd = m_sceneMgr->GetEntityList().end();
        while(iterBegin != iteBEnd)
        {
            Entity* entity = *iterBegin;
            const RigidData* rigidData = entity->GetRigidData();
            
            if(entity->getRigidBodyUID() != 0 && rigidData != nullptr)
            {
                if( ! rigidData->m_isStatic)
                {
                    if(rigidData->m_isDormant)
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
                m_sceneMgr->DeleteEntity(&entity);
            }
            iterBegin++;
        }
    }
}

@end
