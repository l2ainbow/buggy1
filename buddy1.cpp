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

#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.5	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass
#define HEIGHT_MASTER 1.8 // height of master
#define MAX_MOTOR_SPEED M_PI / 2 // max rotation speed of motor [rad/s]

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

static dReal speed=0,steer=0;	// user commands



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

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

  static float xyz[3] = {0.8317f*4,-0.9817f*4,0.8000f*2};
  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
	  "\t'z' to decrease speed.\n"
	  "\t',' to steer left.\n"
	  "\t'.' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}


// called when a key pressed

static void command (int cmd)
{
  switch (cmd) {
  case 'e':
    speed = 0.5;
    break;
  case 'd':
    speed = -0.5;
    break;
  case 's':
    steer = 0.5;
    break;
  case 'f':
    steer = -0.5;
    break;
  case ' ':
    speed = 0;
    steer = 0;
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}

// 原点からの距離取得

static dReal getHorizontalDistanceFromOrigin(dGeomID a){
    const dReal *a_pos = dGeomGetPosition(a);
    dReal dist = sqrt(pow(a_pos[0], 2) + pow(a_pos[1], 2));
    return dist;
}

// 内積の計算

static dReal getInnerProduct(dGeomID a, dGeomID b){
  const dReal *a_pos = dGeomGetPosition(a);
  const dReal *b_pos = dGeomGetPosition(b);
  return a_pos[0]*b_pos[0] + a_pos[1]*b_pos[1];
}

// 内積の計算　

static dReal getInnerProduct(const dReal *a, const dReal *b){
    return a[0] * b[0] + a[1] * b[1];
}

static void getProduct(const dReal *a_vec3, const dReal *b_mat3, dReal *ab_vec3){
    ab_vec3[0] = a_vec3[0] * b_mat3[0] + a_vec3[1] * b_mat3[3] + a_vec3[2] * b_mat3[6];
    ab_vec3[1] = a_vec3[0] * b_mat3[1] + a_vec3[1] * b_mat3[4] + a_vec3[2] * b_mat3[7];
    ab_vec3[2] = a_vec3[0] * b_mat3[2] + a_vec3[1] * b_mat3[5] + a_vec3[2] * b_mat3[8];
}

// 行列式の計算

static dReal getDetermineMatrix(const dReal *matrix){
    return matrix[0] * matrix[4] * matrix[8] + matrix[3] * matrix[7] *matrix[2] + matrix[6] * matrix[1] * matrix[5]
            - matrix[0] * matrix[7] * matrix[5] - matrix[6] * matrix[4] * matrix[2] - matrix[3] * matrix[1] * matrix[8] ;
}

//逆行列の計算

static bool getInvMatrix(const dReal *matrix, dReal *inverse)
{
    dReal det = getDetermineMatrix(matrix);
    if (fabs(det) == 0){
        return false;
    }

    dReal inv_det = 1.0 / det;

    inverse[0] = inv_det * (matrix[4] * matrix[8] - matrix[5] * matrix[7]);
    inverse[1] = inv_det * (matrix[2] * matrix[7] - matrix[1] * matrix[8]);
    inverse[2] = inv_det * (matrix[1] * matrix[5] - matrix[2] * matrix[4]);

    inverse[3] = inv_det * (matrix[5] * matrix[6] - matrix[3] * matrix[8]);
    inverse[4] = inv_det * (matrix[0] * matrix[8] - matrix[2] * matrix[6]);
    inverse[5] = inv_det * (matrix[2] * matrix[3] - matrix[0] * matrix[5]);

    inverse[6] = inv_det * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    inverse[7] = inv_det * (matrix[1] * matrix[6] - matrix[0] * matrix[7]);
    inverse[8] = inv_det * (matrix[0] * matrix[4] - matrix[1] * matrix[3]);
}

// 水平角度の計算

static dReal getHorizontalAngle(dGeomID a, dGeomID b){
    const dReal *a_rot = dGeomGetRotation(a);
    dReal invRot[9];
    if (getInvMatrix(a_rot, invRot) == false){
        return -999;
    }
    const dReal *a_pos = dGeomGetPosition(a);
    const dReal *b_pos = dGeomGetPosition(b);
    const dReal ab[] = { b_pos[0] - a_pos[0], b_pos[1] - a_pos[1], b_pos[2] - a_pos[2] };

    dReal c[3];
    getProduct(ab, invRot, c);
    const dReal x_pos[] = { 1, 0, 0 };

    const dReal cos_theta = getInnerProduct(ab, x_pos);
    return acos(cos_theta);
}

// 水平距離の計算

static dReal getHorizontalDistance(dGeomID a, dGeomID b){
  const dReal *a_pos = dGeomGetPosition(a);
  const dReal *b_pos = dGeomGetPosition(b);
  dReal dist = sqrt(pow(a_pos[0]-b_pos[0],2) + pow(a_pos[1]-b_pos[1],2));
  return dist;
}

// モータの回転速度の計算

static void calculateMotorSpeed(dReal dist, dReal theta, dReal *motorSpeed){
  dReal rMotorSpeed = 0.0;
  dReal lMotorSpeed = 0.0;

  if (theta < - M_PI / 2){
    rMotorSpeed = MAX_MOTOR_SPEED;
    lMotorSpeed = - MAX_MOTOR_SPEED;
  }
  else {
    rMotorSpeed = 0.0;
    lMotorSpeed = 0.0;
  }

  motorSpeed[0] = rMotorSpeed;
  motorSpeed[1] = lMotorSpeed;
}

// シミュレーションのループ実行

static void simLoop (int pause)
{
  int i;
  dReal motorSpeed[2] = {0.0, 0.0};

  dReal dist = getHorizontalDistance(box[0], master);
  dReal theta = getHorizontalAngle(box[0], master) * 180 / M_PI;
  calculateMotorSpeed(dist, theta, motorSpeed);

  printf("dist=%f, theta=%f\n", dist, theta);

  if (!pause) {
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed+steer);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[1],dParamVel2,-speed-steer);
    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[2],dParamVel2,-speed+steer);
    dJointSetHinge2Param (joint[2],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[3],dParamVel2,-speed-steer);
    dJointSetHinge2Param (joint[3],dParamFMax2,0.1);

    // steering
    /*
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    */
    //dJointSetHinge2Param (joint[0],dParamVel,v);
    for (int i = 0; i< 4; i++){
        dJointSetHinge2Param (joint[i],dParamVel,0.0);
        dJointSetHinge2Param (joint[i],dParamFMax,0.2);
        dJointSetHinge2Param (joint[i],dParamLoStop,-0.75);
        dJointSetHinge2Param (joint[i],dParamHiStop,0.75);
        dJointSetHinge2Param (joint[i],dParamFudgeFactor,0.1);
    }

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }

  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,1,1);
  for (i=1; i<=4; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
    dBodyGetRotation(body[i]),0.02f,RADIUS);

  dVector3 ss;
  dGeomBoxGetLengths(master, ss);
  dsDrawBox(dGeomGetPosition(master), dGeomGetRotation(master), ss);

  //dVector3 ss;
  //dGeomBoxGetLengths (ground_box,ss);
  //dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);

  /*
  printf ("%.10f %.10f %.10f %.10f\n",
	  dJointGetHingeAngle (joint[1]),
	  dJointGetHingeAngle (joint[2]),
	  dJointGetHingeAngleRate (joint[1]),
	  dJointGetHingeAngleRate (joint[2]));
  */
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
  dWorldSetGravity (world,0,0,-0.5);
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
  master = dCreateBox (space,0.2,0.2,HEIGHT_MASTER);
  dGeomSetPosition (master,3,3,HEIGHT_MASTER/2.0);

  // environment
  /*
  ground_box = dCreateBox (space,2,1.5,1);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,2,0,-0.34);
  dGeomSetRotation (ground_box,R);
  */

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
