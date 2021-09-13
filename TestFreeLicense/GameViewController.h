//
//  GameViewController.h
//  TestFreeLicense
//
//  Created by TaoweisMac on 2017/3/3.
//  Copyright © 2017年 TaoweisMac. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

class SceneMgr;

@interface GameViewController : GLKViewController
{
    SceneMgr* m_sceneMgr;
}
@end
