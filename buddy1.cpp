/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

buggy with suspension.
this also shows you how to use geom groups.

*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <unistd.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define STEP_SIZE 0.01 // 1ステップの時間[s]
#define CONTROL_CYCLE 0.5 // 制御周期 [s]

#define LENGTH 0.1	   // バディの長さ [m]
#define WIDTH 0.1	   // バディの幅 [m]
#define HEIGHT 0.05	   // バディの高さ [m]
#define RADIUS 0.025	   // タイヤの半径 [m]
#define STARTZ 0.1	   // バディのスタート高さ [m]

#define CMASS 1		   // バディの重さ [kg]
#define WMASS 0.2	   // タイヤの重さ [kg]

#define DIST_FORWARD_MASTER 1.0 // マスターの初期位置（バディから前方向に何mか）[m]
#define DIST_LEFTWARD_MASTER -1.0 // マスターの初期位置（バディから左方向に何mか）[m]

#define HEIGHT_MASTER 1.8 // マスターの高さ [m]
#define SPEED_MASTER 0.5 // マスターの歩行速度 [m/s]

#define EFFICIENT 0.5 // モータの回転効率（=実際の回転速度/最大回転速度; 摩擦や空滑り等で効率減）
#define GEAR_RATIO 38.2 // モータのギア比
#define NO_LOAD_SPEED 12300 // 無負荷回転数 [rpm]
#define MAX_MOTOR_SPEED 2 * M_PI * NO_LOAD_SPEED / 60 / GEAR_RATIO * EFFICIENT // モータの最大回転速度 [rad/s]

#define HEIGHT_CAMERA 5.0 // カメラ位置の高さ [m]
#define GRAVITY -9.80665 // 重力加速度 [m/s2]

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[5]; // ボックス×１、タイヤ×４
static dJointID joint[4];	// タイヤごとにジョイント（0:左前, 1:右前, 2:左後, 3:右後）
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[4]; // 4 tires
static dGeomID ground_box;
static dGeomID master; // master of buddy who is the target person

// things that the user controls

static dReal speed=0; // バディの直進量
static dReal steer=0; // バディの旋回量
static int step = 0; // シミュレーション開始からのステップ数
static dReal motorSpeed[2] = {0.0, 0.0}; // 左右モータの回転速度
static int goForward = 0;
static int goLeft = 0;

// シミュレーション開始前に呼ばれる関数
// 衝突関係の計算

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0,0,HEIGHT_CAMERA};
  static float hpr[3] = {0,-90,0};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'e' to increase speed.\n"
	  "\t'd' to decrease speed.\n"
	  "\t's' to steer left.\n"
	  "\t'f' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}


// キーボードを押したときに呼び出される関数

static void command (int cmd)
{
  switch (cmd) {
  // eを押したら、バディを前進
  case 'e':
    speed += 1.0;
    break;
  // dを押したら、バディを後進
  case 'd':
    speed += -1.0;
    break;
  // sを押したら、バディを左方向に旋回
  case 's':
    steer += 1.0;
    break;
  // fを押したら、バディを右方向に旋回
  case 'f':
    steer += -1.0;
    break;
  // スペースを押したら、バディとマスターを停止
  case ' ':
    speed = 0;
    steer = 0;
    goForward = 0;
    goLeft = 0;
    break;
  // iを押したら、マスターを画面上方向に進める（N回押したらN倍速）
  case 'i':
    goForward += 1;
    break;
  // kを押したら、マスターを画面下方向に進める（N回押したらN倍速）
  case 'k':
    goForward -= 1;
    break;
  // jを押したら、マスターを画面左方向に進める（N回押したらN倍速）
  case 'j':
    goLeft += 1;
    break;
  // lを押したら、マスターを画面右方向に進める（N回押したらN倍速）
  case 'l':
    goLeft -= 1;
    break;
  // 1を押したら、現在の状態を保存
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}

// 内積の計算　

static dReal getInnerProduct(const dReal *a, const dReal *b){
    return a[0] * b[0] + a[1] * b[1];
}

// ベクトル2要素の長さ

static dReal getLength(dReal x, dReal y){
    return sqrt(x*x+y*y);
}

// 回転行列から水平角度を計算

static dReal getHorizontalAngleFromR(const dReal *R){
    return -atan2(R[4],R[0]);
}

// 水平角度の計算

static dReal getHorizontalAngle(dGeomID a, dGeomID b){
    const dReal *a_rot = dGeomGetRotation(a);
    const dReal a_angle = getHorizontalAngleFromR(a_rot);

    const dReal *a_pos = dGeomGetPosition(a);
    const dReal *b_pos = dGeomGetPosition(b);
    const dReal ab[] = { b_pos[0] - a_pos[0], b_pos[1] - a_pos[1], b_pos[2] - a_pos[2] };
    const dReal x_pos[] = { 1, 0, 0 };

    const dReal cos_theta = getInnerProduct(ab, x_pos) / getLength(ab[0], ab[1]);
    dReal theta = acos(cos_theta);
    if (ab[1] > 0){
        theta = - theta;
    }

    //printf("a_angle=%f, theta=%f, ab[0]=%f, ab[1]=%f\n", a_angle, theta, ab[0], ab[1]);

    dReal angle = theta - a_angle;

    if (angle < - M_PI){
        angle = 2 * M_PI + angle;
    }
    else if (angle > M_PI){
        angle = angle - 2 * M_PI;
    }

    return angle;
}

// 水平距離の計算

static dReal getHorizontalDistance(dGeomID a, dGeomID b){
  const dReal *a_pos = dGeomGetPosition(a);
  const dReal *b_pos = dGeomGetPosition(b);
  dReal dist = sqrt(pow(a_pos[0]-b_pos[0],2) + pow(a_pos[1]-b_pos[1],2));
  return dist;
}

/** 前島くんがここを実装 **/

// モータの回転速度の計算
//
// dist: マスターとの距離[m]
// theta: バディから見たマスターの角度(マイナス:左方向、プラス:右方向)[rad]
// motorSpeed: モータの回転速度(0:右モータ、1:左モータ)[rad/s]

static void calculateMotorSpeed(dReal dist, dReal theta, dReal *motorSpeed){
  dReal rMotorSpeed = 0.0; // 右モータの回転速度[rad/s]
  dReal lMotorSpeed = 0.0; // 左モータの回転速度[rad/s]

  // この後に以下のプログラムを書く
  // (1)距離が近い時は停止する
  // (2)マスターが後方にいる場合、旋回する
  // (3)マスターが前方にいる場合、直進する
  // (4)それ以外の場合、カーブ走行する

  if (dist < 1 ){
    rMotorSpeed = 0;
    lMotorSpeed = 0;
  }

  else if (theta < - M_PI / 2){
    // 後方左にいる時は左旋回
    rMotorSpeed = MAX_MOTOR_SPEED;
    lMotorSpeed = - MAX_MOTOR_SPEED;
  }
  else if (theta > M_PI / 2){
    // 後方右にいる時は右旋回
    rMotorSpeed = - MAX_MOTOR_SPEED;
    lMotorSpeed = MAX_MOTOR_SPEED;
  }

  else if (theta < M_PI / 18.0 && theta > - M_PI / 18.0){
    rMotorSpeed = MAX_MOTOR_SPEED;
    lMotorSpeed = - MAX_MOTOR_SPEED;
  }

  else {
    double a = (dist / (2 * sin(theta))) * (dist / (2 * sin(theta)));
    double b = 0.25 * (LENGTH * LENGTH + WIDTH * WIDTH);
    double c = dist / (2 * sin(theta));
    double d = sqrt(LENGTH * LENGTH + WIDTH * WIDTH);
    double e = WIDTH / sqrt(LENGTH * LENGTH + WIDTH * WIDTH);
    double f = M_PI * RADIUS;
    double wr = theta * sqrt(a + b - c * d * e) / f;
    double wl = theta * sqrt(a + b + c * d * e) / f;

    /*double wr = theta * sqrt((dist / (2 * sin(theta)) * (dist / (2 * sin(theta)))
                            + (0.25 * LENGTH * LENGTH + 0.25 * WIDTH * WIDTH)
                            – (dist /(2 * sin(theta)) * (sqrt(LENGTH * LENGTH + WIDTH * WIDTH))
                            * (WIDTH / sqrt(LENGTH * LENGTH + WIDTH * WIDTH))) / (M_PI * RADIUS);*/
    /*double wl = theta * sqrt((dist / (2 * sin(theta)) * (dist / (2 * sin(theta)))
                            + (0.25 * LENGTH * LENGTH + 0.25 * WIDTH * WIDTH)
                            + (dist / (2 * sin(theta)) * (sqrt(LENGTH * LENGTH + WIDTH * WIDTH))
                            * (WIDTH / sqrt(LENGTH * LENGTH + WIDTH * WIDTH))) / (M_PI * RADIUS);*/

      if (wr < wl) {
        double lower = wr / wl;
        rMotorSpeed = MAX_MOTOR_SPEED * lower;
        lMotorSpeed = MAX_MOTOR_SPEED;
      }
      else {
        double lower = wl / wr;
        rMotorSpeed = MAX_MOTOR_SPEED;
        lMotorSpeed = MAX_MOTOR_SPEED * lower;
      }
  }

  motorSpeed[0] = lMotorSpeed;
  motorSpeed[1] = rMotorSpeed;
}

/** ここまで **/

// マスターを動かす

static void moveMaster(){
    const dReal *pos = dGeomGetPosition(master);
    dReal x = goForward * SPEED_MASTER * STEP_SIZE;
    dReal y = goLeft * SPEED_MASTER * STEP_SIZE;
    dGeomSetPosition(master, pos[0] + x, pos[1] + y, pos[2]);
}

// 現在時点が制御周期か

static bool isControlCycle(){
    return step % (int)(CONTROL_CYCLE / STEP_SIZE) == 0;
}

// シミュレーションのループ実行

static void simLoop (int pause)
{
  int i;

  if (!pause) {
    step++;

    // master
    moveMaster();

    // 制御周期になったら、距離・角度を計測し、それに応じてモータの速度を変更する
    if (isControlCycle())
    {
        dReal dist = getHorizontalDistance(box[0], master);
        dReal theta = getHorizontalAngle(box[0], master);
        calculateMotorSpeed(dist, theta, motorSpeed);

        // 開始からの時間[s]、マスターとの距離[m]、マスターとの角度[rad]([度])、右モータの回転速度[rad/s]、左モータの回転速度[rad/s]の表示
        printf("sec=%6.1f, dist=%0.2f, theta=%0.2f(%3d deg), rMotor=%0.1f, lMotor=%0.1f\n", step*STEP_SIZE, dist, theta, (int)(theta * 180 / M_PI),  motorSpeed[0], motorSpeed[1]);
    }

    // バディを手動操作している場合は、その操作量で動作する
    if (speed != 0 || steer != 0){
        dJointSetHinge2Param (joint[0],dParamVel2,-speed+steer);
        dJointSetHinge2Param (joint[1],dParamVel2,-speed-steer);
        dJointSetHinge2Param (joint[2],dParamVel2,-speed+steer);
        dJointSetHinge2Param (joint[3],dParamVel2,-speed-steer);
    }
    // バディを手動操作していない場合（停止している場合）は、自動制御する
    else {
        dJointSetHinge2Param (joint[0],dParamVel2,-motorSpeed[0]);
        dJointSetHinge2Param (joint[1],dParamVel2,-motorSpeed[1]);
        dJointSetHinge2Param (joint[2],dParamVel2,-motorSpeed[0]);
        dJointSetHinge2Param (joint[3],dParamVel2,-motorSpeed[1]);
    }
    dJointSetHinge2Param (joint[0],dParamFMax2,1.0);
    dJointSetHinge2Param (joint[1],dParamFMax2,1.0);
    dJointSetHinge2Param (joint[2],dParamFMax2,1.0);
    dJointSetHinge2Param (joint[3],dParamFMax2,1.0);

    for (int i = 0; i< 4; i++){
        dJointSetHinge2Param (joint[i],dParamVel,0.0);
        dJointSetHinge2Param (joint[i],dParamFMax,0.1);
        dJointSetHinge2Param (joint[i],dParamLoStop,0);
        dJointSetHinge2Param (joint[i],dParamHiStop,0);
        dJointSetHinge2Param (joint[i],dParamFudgeFactor,0.1);
    }

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,STEP_SIZE);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }

  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,0,0);
  for (i=1; i<=2; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
    dBodyGetRotation(body[i]),0.005f,RADIUS);
  dsSetColor (1,1,1);
  for (i=3; i<=4; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
    dBodyGetRotation(body[i]),0.005f,RADIUS);

  dsSetColor (0,0,1);
  dsSetTexture(DS_NONE);
  dVector3 ss;
  dGeomBoxGetLengths(master, ss);
  dsDrawBox(dGeomGetPosition(master), dGeomGetRotation(master), ss);

  usleep(1000000*STEP_SIZE);

}

// メイン関数

int main (int argc, char **argv)
{
  int i;
  dMass m;

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,GRAVITY);
  ground = dCreatePlane (space,0,0,1,0);

  // chassis body
  body[0] = dBodyCreate (world);
  dBodySetPosition (body[0],0,0,STARTZ);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[0],&m);
  box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);

  // wheel bodies
  for (i=1; i<=4; i++) {
    body[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (body[i],q);
    dMassSetSphere (&m,1,RADIUS);
    dMassAdjust (&m,WMASS);
    dBodySetMass (body[i],&m);
    sphere[i-1] = dCreateSphere (0,RADIUS);
    dGeomSetBody (sphere[i-1],body[i]);
  }
  dBodySetPosition (body[1],0.5*LENGTH,WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[2],0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[3],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[4],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

  // front and back wheel hinges
  for (i=0; i<4; i++) {
    joint[i] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[i],body[0],body[i+1]);
    const dReal *a = dBodyGetPosition (body[i+1]);
    dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[i],0,0,1);
    dJointSetHinge2Axis2 (joint[i],0,1,0);
  }

  // set joint suspension
  for (i=0; i<4; i++) {
    dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
  }

  // lock back wheels along the steering axis
  for (i=1; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
    // the following alternative method is no good as the wheels may get out
    // of alignment:
    //   dJointSetHinge2Param (joint[i],dParamVel,0);
    //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
  }


  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);
  dSpaceAdd (car_space,box[0]);
  dSpaceAdd (car_space,sphere[0]);
  dSpaceAdd (car_space,sphere[1]);
  dSpaceAdd (car_space,sphere[2]);
  dSpaceAdd (car_space,sphere[3]);

  // create master who is the target person
  master = dCreateBox (space,0.1,0.1,HEIGHT_MASTER);
  dGeomSetPosition (master,DIST_FORWARD_MASTER,DIST_LEFTWARD_MASTER,HEIGHT_MASTER/2.0);

  // run simulation
  dsSimulationLoop (argc,argv,352*2,288*2,&fn);

  // terminate
  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dGeomDestroy (sphere[2]);
  dGeomDestroy (sphere[3]);
  dGeomDestroy (master);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
