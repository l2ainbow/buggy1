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

#define STEP_SIZE 0.01 // 1�X�e�b�v�̎���[s]
#define LENGTH 0.7	   // �o�f�B�̒��� [m]
#define WIDTH 0.5	   // �o�f�B�̕� [m]
#define HEIGHT 0.2	   // �o�f�B�̍��� [m]
#define RADIUS 0.18	   // �^�C���̒��a [m]
#define STARTZ 0.5	   // �o�f�B�̃X�^�[�g���� [m]
#define CMASS 1		   // �o�f�B�̏d�� [kg]
#define WMASS 0.2	   // �^�C���̏d�� [kg]
#define HEIGHT_MASTER 1.8 // �}�X�^�[�̍��� [m]
#define MAX_MOTOR_SPEED (M_PI / 2) // ���[�^�̍ő��]���x [rad/s]

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[5]; // �{�b�N�X�~�P�A�^�C���~�S
static dJointID joint[4];	// �^�C�����ƂɃW���C���g�i0:���O, 1:�E�O, 2:����, 3:�E��j
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[4]; // 4 tires
static dGeomID ground_box;
static dGeomID master; // master of buddy who is the target person

// things that the user controls

static dReal speed=0; // �o�f�B�̒��i��
static dReal steer=0; // �o�f�B�̐����
static int step = 0; // �V�~�����[�V�����J�n����̃X�e�b�v��

// �V�~�����[�V�����J�n�O�ɌĂ΂��֐�
// �Փˊ֌W�̌v�Z

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

  static float xyz[3] = {0,0,10.0f};
  static float hpr[3] = {0,-90,0};
  //static float xyz[3] = {0.8317f*4,-0.9817f*4,0.8000f*2};
  //static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'e' to increase speed.\n"
	  "\t'd' to decrease speed.\n"
	  "\t's' to steer left.\n"
	  "\t'f' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}


// �L�[�{�[�h���������Ƃ��ɌĂяo�����֐�

static void command (int cmd)
{
  switch (cmd) {
  case 'e':
    speed += 1.0;
    break;
  case 'd':
    speed += -1.0;
    break;
  case 's':
    steer += 1.0;
    break;
  case 'f':
    steer += -1.0;
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

// ���ς̌v�Z�@

static dReal getInnerProduct(const dReal *a, const dReal *b){
    return a[0] * b[0] + a[1] * b[1];
}

// �x�N�g��2�v�f�̒���

static dReal getLength(dReal x, dReal y){
    return sqrt(x*x+y*y);
}

// ��]�s�񂩂琅���p�x���v�Z

static dReal getHorizontalAngleFromR(const dReal *R){
    return -atan2(R[4],R[0]);
}

// �����p�x�̌v�Z

static dReal getHorizontalAngle(dGeomID a, dGeomID b){
    const dReal *a_rot = dGeomGetRotation(a);
    const dReal a_angle = getHorizontalAngleFromR(a_rot);

    const dReal *a_pos = dGeomGetPosition(a);
    const dReal *b_pos = dGeomGetPosition(b);
    const dReal ab[] = { b_pos[0] - a_pos[0], b_pos[1] - a_pos[1], b_pos[2] - a_pos[2] };
    const dReal x_pos[] = { 1, 0, 0 };

    const dReal cos_theta = getInnerProduct(ab, x_pos) / getLength(ab[0], ab[1]);
    const dReal theta = - acos(cos_theta);

    dReal angle = theta - a_angle;

    if (angle < - M_PI){
        angle = 2 * M_PI + angle;
    }
    else if (angle > M_PI){
        angle = angle - 2 * M_PI;
    }

    return angle;
}

// ���������̌v�Z

static dReal getHorizontalDistance(dGeomID a, dGeomID b){
  const dReal *a_pos = dGeomGetPosition(a);
  const dReal *b_pos = dGeomGetPosition(b);
  dReal dist = sqrt(pow(a_pos[0]-b_pos[0],2) + pow(a_pos[1]-b_pos[1],2));
  return dist;
}

/** �O�����񂪂��������� **/

// ���[�^�̉�]���x�̌v�Z
//
// dist: �}�X�^�[�Ƃ̋���[m]
// theta: �o�f�B���猩���}�X�^�[�̊p�x(�}�C�i�X:�������A�v���X:�E����)[rad]
// motorSpeed: ���[�^�̉�]���x(0:�E���[�^�A1:�����[�^)[rad/s]

static void calculateMotorSpeed(dReal dist, dReal theta, dReal *motorSpeed){
  dReal rMotorSpeed = 0.0; // �E���[�^�̉�]���x[rad/s]
  dReal lMotorSpeed = 0.0; // �����[�^�̉�]���x[rad/s]

  // ���̌�Ɉȉ��̃v���O����������
  // (1)�������߂����͒�~����
  // (2)�}�X�^�[������ɂ���ꍇ�A���񂷂�
  // (3)�}�X�^�[���O���ɂ���ꍇ�A���i����
  // (4)����ȊO�̏ꍇ�A�J�[�u���s����
  if (theta < - M_PI / 2){
    // (2)�}�X�^�[������ɂ���ꍇ�A���񂷂�
    rMotorSpeed = MAX_MOTOR_SPEED;
    lMotorSpeed = - MAX_MOTOR_SPEED;
  }
  else if (theta > M_PI / 2){
    rMotorSpeed = - MAX_MOTOR_SPEED;
    lMotorSpeed = MAX_MOTOR_SPEED;
  }

  motorSpeed[0] = rMotorSpeed;
  motorSpeed[1] = lMotorSpeed;
}

/** �����܂� **/

// �V�~�����[�V�����̃��[�v���s

static void simLoop (int pause)
{
  int i;
  dReal motorSpeed[2] = {0.0, 0.0};

  dReal dist = getHorizontalDistance(box[0], master);
  dReal theta = getHorizontalAngle(box[0], master);
  calculateMotorSpeed(dist, theta, motorSpeed);

  if (!pause) {
    step++;

    if (step % 100 == 0)
        printf("sec=%6d, dist=%0.2f, theta=%0.2f, rMotor=%0.1f, lMotor=%0.1f\n", (int)(step*STEP_SIZE), dist, theta, motorSpeed[0], motorSpeed[1]);
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed+steer);
    //dJointSetHinge2Param (joint[0],dParamVel2,motorSpeed[0]);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[1],dParamVel2,-speed-steer);
    //dJointSetHinge2Param (joint[1],dParamVel2,motorSpeed[1]);
    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[2],dParamVel2,-speed+steer);
    //dJointSetHinge2Param (joint[2],dParamVel2,motorSpeed[0]);
    dJointSetHinge2Param (joint[2],dParamFMax2,0.1);
    dJointSetHinge2Param (joint[3],dParamVel2,-speed-steer);
    //dJointSetHinge2Param (joint[3],dParamVel2,motorSpeed[1]);
    dJointSetHinge2Param (joint[3],dParamFMax2,0.1);

    for (int i = 0; i< 4; i++){
        dJointSetHinge2Param (joint[i],dParamVel,0.0);
        dJointSetHinge2Param (joint[i],dParamFMax,0.2);
        dJointSetHinge2Param (joint[i],dParamLoStop,-0.75);
        dJointSetHinge2Param (joint[i],dParamHiStop,0.75);
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
  dsSetColor (1,1,1);
  for (i=1; i<=4; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
    dBodyGetRotation(body[i]),0.02f,RADIUS);

  dVector3 ss;
  dGeomBoxGetLengths(master, ss);
  dsDrawBox(dGeomGetPosition(master), dGeomGetRotation(master), ss);

}

// ���C���֐�

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
  dGeomSetPosition (master,5,-3,HEIGHT_MASTER/2.0);

  // environment

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
