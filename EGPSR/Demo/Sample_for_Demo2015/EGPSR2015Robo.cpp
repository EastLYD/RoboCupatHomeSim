
#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>

// The robot must release the Object before to start a new task

double tab_joint_left[5][7] = {{0,0,0,0,0,0,0},{0.5,-0.8,-0.2,-1.5,-0.2,0,0},{-0.25,-1,-0.6,-0.65,-0.3,0,0},{0,-1,-1.3,-1.3,0,0,0},{0,-1,-1,-1,0,0,0}};

double tab_joint_right[5][7] = {{0,0,0,0,0,0,0},{-0.5,-0.5,0.2,-1.5,0,0,0},{0,-1.1,1.27,-0.7,0.1,0,0},{0,-1.1,1.57,-1.57,0.3,0,0},{0,-1,1,-1,0,0,0}};

#define error_angle 0.08
#define error_distance 1.3
#define error_angle_arm 0.05

//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller
{
	public:
		//function to use SigVerse
		void onInit(InitEvent &evt);
		double onAction(ActionEvent&);
		void onRecvMsg(RecvMsgEvent &evt);
		void onCollision(CollisionEvent &evt);

		//function to control the robot
		void stopRobotMove(void);
		bool goTo(Vector3d pos, double rangeToPoint);

		//function to find the best place to catch an object
		int defineCoteTable(std::string &name, std::string &name2);
		Vector3d PointApproachObj(std::string &name, std::string &name2, double rangeToTable);
		double PathinFrontObject(std::string room, std::string object);

		//function use by other function
		void recognizeObjectPosition(Vector3d &pos, std::string &name);
		double getAngularXonVect(Vector3d pos, Vector3d mypos);
		double getDistoObj(Vector3d pos, Vector3d pos2);
		double getDist2D(Vector3d pos, Vector3d pos2);
		double getDist3D(Vector3d pos, Vector3d pos2);
		double getRoll(Rotation rot);
		double getPitch(Rotation rot);
		double getYaw(Rotation rot);

		//function for the left arm
		bool moveLeftArm();
		void grasp_left_hand();
		void release_left_hand();
		void chooze_task_arm_left(int task);

	private:
		RobotObj *my;
		RobotObj *m_my;
		Vector3d go_to;
		double Robot_speed ;
    double Change_Robot_speed;

		//grasping
		bool m_grasp_left;

		//to define the object and the table
		std::string m_object, m_table;

		//define different joint of right and left arm
		double joint_left[7];
		double joint_right[7];
		double m_joint_left[7];
		double m_joint_right[7];

		int m_robotState;
		int m_state;
		int m_frontState;
		int stopfront;

		std::string room_msg;
		std::string object_msg;
		std::string m_pointedObject;  // 取りにいくオブジェクト名

		// onActionの戻り値
		double m_onActionReturn;

		// 移動終了時間
		double m_jointVelocity;    // rotation speed around the joint

		Vector3d m_relayPoint0;
		Vector3d m_relayPoint1;
		Vector3d m_relayPoint2;
		Vector3d m_relayPoint3;
		Vector3d m_relayPoint4;
		Vector3d m_relayPoint5;

		Vector3d m_kitchenPoint;
		Vector3d m_livingPoint;
		Vector3d m_bedroomPoint;
		Vector3d m_lobbyPoint;
		Vector3d m_originPoint;
		Vector3d m_livingroomPoint;
		Vector3d m_lastPoint;

		// access points
		// kitchen
		std::vector<Vector3d> m_frontkitchen;
		Vector3d m_kitchenFront1;
		Vector3d m_kitchenFront2;
		Vector3d m_kitchenFront3;
		Vector3d m_kitchenFront4;
		// bedroom
		std::vector<Vector3d> m_frontbedroom;
		Vector3d m_bedroomFront1;
		Vector3d m_bedroomFront2;
		Vector3d m_bedroomFront3;
		// lobby
		std::vector<Vector3d> m_frontlooby;
		Vector3d m_lobbyFront1;
		Vector3d m_lobbyFront2;
		Vector3d m_lobbyFront3;
		Vector3d m_lobbyFront4;
		// living room
		std::vector<Vector3d> m_frontlivingroom;
		Vector3d m_livingroomFront1;
		Vector3d m_livingroomFront2;
		Vector3d m_livingroomFront3;
    Vector3d m_livingroomFront4;
    Vector3d m_livingroomFront5;
};

/************************************************************************************/
/***************************Move the robot in the world******************************/
/************************************************************************************/

bool RobotController::goTo(Vector3d pos, double rangeToPoint)
{
  double speed;

  Vector3d ownPosition;
  my->getPosition(ownPosition);

  Rotation ownRotation;
  my->getRotation(ownRotation);

  double angle = getAngularXonVect(pos, ownPosition);
  double dist = getDistoObj(pos,ownPosition);
  double roll = getRoll(ownRotation);

  if (angle > 3 || angle < -3) angle = M_PI;

  // error on angle
  if ((angle-roll)>-error_angle && (angle-roll)<error_angle)
  // error on distance
    if (dist-rangeToPoint < error_distance && dist-rangeToPoint > -error_distance)
    {
      stopRobotMove();
      return true;
    }
    else
    {
      speed = dist-rangeToPoint;
      if (dist-rangeToPoint < 5)
        if( dist-rangeToPoint > 0 )
          my->setWheelVelocity(1, 1);
        else
          my->setWheelVelocity(-1, -1);
      else if( dist-rangeToPoint > 0 )
        my->setWheelVelocity(Robot_speed , Robot_speed );
      else
        my->setWheelVelocity(-Robot_speed , -Robot_speed );
      return false;
    }
  else
  {
    speed = fabs(angle-roll)*4;
    if (speed/4 > 0.3)
      if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-0.5, 0.5);
      else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(0.5, -0.5);
      else if (angle < roll)
          my->setWheelVelocity(0.5, -0.5);
      else
        my->setWheelVelocity(-0.5, 0.5);
    else if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-speed, speed);
       else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(speed, -speed);
       else if (angle < roll)
          my->setWheelVelocity(speed, -speed);
       else
        my->setWheelVelocity(-speed, speed);

    return false;
  }
  return false;
}

/************************************************************************************/

void RobotController::stopRobotMove(void)
{
  my->setWheelVelocity(0.0, 0.0);
}

/************************************************************************************/
/*******************Find the point near the object and the table*********************/
/************************************************************************************/

Vector3d RobotController::PointApproachObj(std::string &name, std::string &name2, double rangeToTable)
{
  Vector3d intermediatePoint;
  int obj_side;

  obj_side = defineCoteTable(name, name2);

  SimObj *obj = getObj(name2.c_str());
  Vector3d can_pos;
  Rotation table_rot;

  obj->getPosition(can_pos);

  switch (obj_side)
  {
    case 1 :  intermediatePoint =  Vector3d(can_pos.x() + rangeToTable, 0, can_pos.z());
          break;
    case 2 :  intermediatePoint =  Vector3d(can_pos.x(), 0, can_pos.z() - rangeToTable);
          break;
    case 3 :  intermediatePoint =  Vector3d(can_pos.x() - rangeToTable, 0, can_pos.z());
          break;
    case 4 :  intermediatePoint =  Vector3d(can_pos.x(), 0, can_pos.z() + rangeToTable);
          break;
    default : intermediatePoint =  Vector3d(0, 0, 0);
          break;
  }

  return intermediatePoint;
}

int RobotController::defineCoteTable(std::string &name, std::string &name2)
{
  double obj_roll, table_roll;
  double TabObj_angle;
  int obj_side;

  double temp = M_PI/4;
  double temp2 = 3*M_PI/4;

  //initialisation
  SimObj *table = getObj(name.c_str());
  SimObj *obj = getObj(name2.c_str());
  Vector3d table_pos, can_pos;

  //addition of quaternion and position
  table->getPosition(table_pos);
  obj->getPosition(can_pos);

  //calculate angle vector between table and obj with X-axis
  TabObj_angle = getAngularXonVect(can_pos, table_pos);

  double diff = TabObj_angle;


  if (name == "Couch_table1" || name == "Kitchen Table1" || name == "Dinner table1")
  {
    temp = 3*M_PI/8;
    temp2 = 5*M_PI/8;
  }
  else if ( name == "Buffet1" || name == "Buffet2" || name == "Side table1")
  {
    temp = M_PI/4;
    temp2 = 3*M_PI/4;
  }

  if ((diff >= temp) && (diff < temp2))
    obj_side = 1;
  else if ((diff >= -temp) && (diff < temp))
      obj_side = 4;
     else if ((diff >= -temp2) && (diff < -temp))
        obj_side = 3;
        else
        obj_side = 2;

  if (name == "Side_board1" )
  {

    if ((diff >= 1.33) && (diff < 1.57))
    obj_side = 1;
  else if ((diff >= 0) && (diff < 1.33))
      obj_side = 4;
     else if ((diff >= -1.60) && (diff < 0))
        obj_side = 3;
        else
        obj_side = 2;
  }
  else if (name == "Side board2")
  {

    if ((diff >= 0) && (diff < 1.57))
      obj_side = 1;
    else
      obj_side = 2;
  }
  return obj_side;
}

/************************************************************************************/
/*******Find roll,pitch,yaw, lenght between to point, angle between to point*********/
/************************************************************************************/

double RobotController::getRoll(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double roll  = atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);

  return roll;
}

double RobotController::getPitch(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double pitch = atan2(2*qx*qw - 2*qy*qz, 1 - 2*qx*qx - 2*qz*qz);

  return pitch;
}

double RobotController::getYaw(Rotation rot)
{
    // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double yaw   = asin(2*qx*qy + 2*qz*qw);

  return yaw;
}

double RobotController::getDistoObj(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  // measure actual distance
  double distance = sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());

  return distance;
}

double RobotController::getAngularXonVect(Vector3d pos, Vector3d mypos)
{
  double targetAngle;

  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= mypos;

  // rotation angle from z-axis to vector
  targetAngle = atan(l_pos.x()/l_pos.z());

  // direction
  if(l_pos.z() < 0 && l_pos.x()>=0)
    targetAngle = M_PI+targetAngle;
  else if (l_pos.z()<0 && l_pos.x()<0)
    targetAngle = targetAngle-M_PI;

  return targetAngle;
}

double RobotController::getDist2D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());
}

double RobotController::getDist3D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z()+l_pos.y()*l_pos.y());
}

/************************************************************************************/

/************************************************************************************/
/****************************function for the left arm*******************************/
/************************************************************************************/

bool RobotController::moveLeftArm()
{
  bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;

  if(joint_left[0] != m_joint_left[0] )
  {
    if(joint_left[0] < m_joint_left[0] && m_joint_left[0]-joint_left[0] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT0", 0.2, 0.0);
      joint_left[0] = my->getJointAngle("LARM_JOINT0");
    }
    else if(joint_left[0] > m_joint_left[0] && joint_left[0]-m_joint_left[0] > error_angle_arm)
     {
      my->setJointVelocity("LARM_JOINT0", -0.2, 0.0);
      joint_left[0] = my->getJointAngle("LARM_JOINT0");
     }
    else
    {
      my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
      j0 = true;
    }

  }
  else j0 = true;

  if(joint_left[1] != m_joint_left[1] )
  {
    if(joint_left[1] < m_joint_left[1] && m_joint_left[1]-joint_left[1] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT1", 0.2, 0.0);
      joint_left[1] = my->getJointAngle("LARM_JOINT1");
    }
    else if(joint_left[1] > m_joint_left[1] && joint_left[1]-m_joint_left[1] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT1", -0.2, 0.0);
        joint_left[1] = my->getJointAngle("LARM_JOINT1");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
         j1 = true;
       }
  }
  else j1 = true;

  if(joint_left[2] != m_joint_left[2] )
  {
    if(joint_left[2] < m_joint_left[2] && m_joint_left[2]-joint_left[2] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT3", 0.2, 0.0);
      joint_left[2] = my->getJointAngle("LARM_JOINT3");
    }
    else if(joint_left[2] > m_joint_left[2] && joint_left[2]-m_joint_left[2] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT3", -0.2, 0.0);
        joint_left[2] = my->getJointAngle("LARM_JOINT3");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
         j3 = true;
       }
  }
  else j3 = true;

  if(joint_left[3] != m_joint_left[3] )
  {
    if(joint_left[3] < m_joint_left[3] && m_joint_left[3]-joint_left[3] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT4", 0.2, 0.0);
      joint_left[3] = my->getJointAngle("LARM_JOINT4");
    }
    else if(joint_left[3] > m_joint_left[3] && joint_left[3]-m_joint_left[3] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT4", -0.2, 0.0);
        joint_left[3] = my->getJointAngle("LARM_JOINT4");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
         j4 = true;
       }
  }
  else j4 = true;

  if(joint_left[4] != m_joint_left[4] )
  {
    if(joint_left[4] < m_joint_left[4] && m_joint_left[4]-joint_left[4] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT5", 0.2, 0.0);
      joint_left[4] = my->getJointAngle("LARM_JOINT5");
    }
    else if(joint_left[4] > m_joint_left[4] && joint_left[4]-m_joint_left[4] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT5", -0.2, 0.0);
        joint_left[4] = my->getJointAngle("LARM_JOINT5");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
         j5 = true;
       }
  }
  else j5 = true;

  if(joint_left[5] != m_joint_left[5] )
  {
    if(joint_left[5] < m_joint_left[5] && m_joint_left[5]-joint_left[5] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT6", 0.2, 0.0);
      joint_left[5] = my->getJointAngle("LARM_JOINT6");
    }
    else if(joint_left[5] > m_joint_left[5] && joint_left[5]-m_joint_left[5] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT6", -0.2, 0.0);
        joint_left[5] = my->getJointAngle("LARM_JOINT6");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
         j6 = true;
       }
  }
  else j6 = true;

  if(joint_left[6] != m_joint_left[6] )
  {
    if(joint_left[5] < m_joint_left[6] && m_joint_left[6]-joint_left[5] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT7", 0.2, 0.0);
      joint_left[5] = my->getJointAngle("LARM_JOINT7");
    }
    else if(joint_left[5] > m_joint_left[6] && joint_left[5]-m_joint_left[6] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT7", -0.2, 0.0);
        joint_left[5] = my->getJointAngle("LARM_JOINT7");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT7", 0.0, 0.0);
         j7 = true;
       }
  }
  else j7 = true;

  if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j7 == true)
    return true;
  else
    return false;

  return false;
}

void RobotController::chooze_task_arm_left(int task)
{
  for (int i=0; i<7; i++)
    m_joint_left[i] = tab_joint_left[task][i];
}

void RobotController::grasp_left_hand()
{
	Vector3d hand, object;
	SimObj *obj = getObj(m_object.c_str());

	obj->getPosition(object);
	my->getJointPosition(hand, "LARM_JOINT7");

	double distance = getDist3D(hand,object);

	if(distance < 13 &&  m_grasp_left == false)
	{
		CParts * parts = my->getParts("LARM_LINK7");
		if (parts->graspObj(m_object))
		{
			m_grasp_left = true;
			broadcastMsg("Object_grasped");
		}
	}
}

void RobotController::release_left_hand()
{
	CParts * parts = my->getParts("LARM_LINK7");
	if( m_grasp_left == true)
    broadcastMsg("Object_released");
		parts->releaseObj();
	m_grasp_left = false;
}

/*************************************************************************************/

void RobotController::onInit(InitEvent &evt)
{

	m_grasp_left = false;
  Change_Robot_speed = 6; // to change the robot's velocity
  Robot_speed  = Change_Robot_speed;
	m_state = 0;
	m_robotState = 0;
	m_frontState = 0;

	m_my = getRobotObj(myname());
	m_my->setWheel(10.0, 10.0);
	my = getRobotObj(myname());
	my->setWheel(10.0, 10.0);

	//エージェントの正面の定義はz軸の正の向きを向いていると仮定する
	m_onActionReturn = 0.01;

	room_msg = "";
	object_msg ="";

	stopfront = 0;

	m_relayPoint1 = Vector3d(20, 30, 0);
	m_relayPoint2 = Vector3d(20, 30, 350);
	m_relayPoint3 = Vector3d(150, 30, 320);
	m_relayPoint4 = Vector3d(-400, 30, 0);
	m_relayPoint5 = Vector3d(-400, 30, -130);

	m_originPoint = Vector3d(20, 30, -200);
	m_kitchenPoint = Vector3d(190, 30, 320);
	m_bedroomPoint = Vector3d(-400, 30, 200);
	m_lobbyPoint = Vector3d(-300, 30, -180);
	m_livingroomPoint = Vector3d(300, 30, -130);
	Vector3d m_lastPoint = Vector3d(0, 0, 0);

	//kitchen
	m_kitchenFront1 = Vector3d(180, 30, 320);
	m_kitchenFront2 = Vector3d(390, 30, 320);
	m_kitchenFront3 = Vector3d(390, 30, 140);
	m_kitchenFront4 = Vector3d(160, 30, 140);
	m_frontkitchen.push_back(m_kitchenFront1 );
	m_frontkitchen.push_back(m_kitchenFront2 );
	m_frontkitchen.push_back(m_kitchenFront3 );
	m_frontkitchen.push_back(m_kitchenFront4 );

	// bedroom
	m_bedroomFront1 = Vector3d(-400, 30, 340);
	m_bedroomFront2 = Vector3d(-210, 30, 340);
	m_bedroomFront3 = Vector3d(-400, 30, 340);
	m_frontbedroom.push_back(m_bedroomFront1 );
	m_frontbedroom.push_back(m_bedroomFront2 );
	m_frontbedroom.push_back(m_bedroomFront3 );

	// lobby
  m_frontlooby.clear();
	m_lobbyFront1 = Vector3d(-190, 30, -310);
	m_lobbyFront2 = Vector3d(-220, 30, -220);
	m_lobbyFront3 = Vector3d(-200, 30, -170);
	m_lobbyFront4 = Vector3d(-350, 30, -180);
	m_frontlooby.push_back(m_lobbyFront1 );
	m_frontlooby.push_back(m_lobbyFront2 );
	m_frontlooby.push_back(m_lobbyFront3 );
	m_frontlooby.push_back(m_lobbyFront4 );

	// living room
  m_frontlivingroom.clear();
	m_livingroomFront1 = Vector3d(175, 30,-140);
	m_livingroomFront2 = Vector3d(380, 30,-140);
	m_livingroomFront3 = Vector3d(380, 30,-285);
  m_livingroomFront4 = Vector3d(185, 30,-285);
  m_livingroomFront5 = Vector3d(180, 30,-140);
	m_frontlivingroom.push_back(m_livingroomFront1 );
	m_frontlivingroom.push_back(m_livingroomFront2 );
	m_frontlivingroom.push_back(m_livingroomFront3 );
  m_frontlivingroom.push_back(m_livingroomFront4 );
  m_frontlivingroom.push_back(m_livingroomFront5 );
	srand((unsigned)time( NULL ));
}

/*************************************************************************************/

double RobotController::onAction(ActionEvent &evt)
{

    switch(m_state)
    {
	case 100:	{
					if (goTo(go_to, 0) == true) m_state = 4;
					break;
				}
	case 4: 	{ //preparation of the arm for grasp
					recognizeObjectPosition(go_to, m_object);
					if (goTo(go_to, 45) == true) m_state = 5;
					break;
				}
	case 5: 	{ //preparation of the arm for grasp
					chooze_task_arm_left(1);
					if (moveLeftArm() == true) m_state = 6;
					break;
				}
	case 6: 	{ //move to the object
					Robot_speed  = 1;
					if (goTo(go_to, 39) == true) m_state = 7;
					break;
				}
	case 7: 	{ //move arm to grasp the object
					chooze_task_arm_left(2);
					grasp_left_hand();
					if (moveLeftArm() == true) m_state = 8;
					break;
				}
	case 8: 	{
					// broadcastMsg("Object_grasped");
					//LOG_MSG(("Object_grasped"));
					m_state = 9;
					break;
				}
	case 9: 	{ //move arm to place in good position for moving
					grasp_left_hand();
					chooze_task_arm_left(3);
					if (moveLeftArm() == true)
					m_state = 3333;
					break;
				}
	case 3333: 	{
					m_my->setWheelVelocity(-0.7,-0.7);
					chooze_task_arm_left(4);
					if (moveLeftArm() == true)
					{
						Robot_speed  = Change_Robot_speed;
						m_state = m_frontState-10;
					}
					break;
				}
	case 109: 	{

					if(room_msg=="kitchen")
					{
						m_state = 50;
						room_msg = "";
						object_msg = "";
					}
					else if(room_msg=="lobby")
					{
						m_state = 60;
						room_msg = "";
						object_msg = "";
					}
					else if(room_msg=="bed room")
					{
						m_state = 70;
						room_msg = "";
						object_msg = "";
					}
					else if(room_msg=="living room")
					{
						m_state = 80;
						room_msg = "";
						object_msg = "";
					}
					break;
				}
	case 200: 	{
					// ゴミ箱に到着
					m_my->setWheelVelocity(0.0, 0.0);
					// grasp中のパーツを取得します getting grasped tarts
					release_left_hand();
					// releaseします

					// ゴミが捨てられるまで少し待つ
					sleep(1);
					m_grasp_left = false;
					m_state++;
					break;
				}
	case 201: 	{
					broadcastMsg("Task_finished");
					LOG_MSG(("Task_finished"));
					sleep(1);
					m_state = 1;
					break;
				}
printf("the actual stat is  %d\n", m_state);
	// move toward the kitchen

	case 10: 	{ // set rotation for relay point
					if (goTo(m_relayPoint1, 0) == true) m_state = 11;
					break;
				}
	case 11: 	{ // rotate toward relay point
					if (goTo(m_relayPoint2, 0) == true) m_state = 12;
					break;
				}
	case 12: 	{ // move toward relay point
					if (goTo(m_relayPoint3, 0) == true) m_state = 13;
					break;
				}
	case 13: 	{ // rotate toward relay point
					if (goTo(m_kitchenPoint, 0) == true) m_state = 14;
					break;
				}
	case 14: 	{ // move toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 15;
					break;
				}
	case 15: 	{ // rotate toward relay point
					broadcastMsg("Room_reached");
					LOG_MSG(("Room_reached"));
					sleep(1);

					m_lastPoint= m_kitchenPoint;
					PathinFrontObject(room_msg,m_pointedObject);
					break;
				}

	// move toward the lobby
	case 20: 	{ // set rotation for relay point
					if (goTo(m_relayPoint1, 0) == true) m_state = 21;
					break;
				}
	case 21: 	{ // rotate toward relay point
					if (goTo(m_relayPoint4, 0) == true) m_state = 22;
					break;
				}
	case 22: 	{ // move toward relay point
					if (goTo(m_relayPoint5, 0) == true) m_state = 23;
					break;
				}
	case 23: 	{ // rotate toward relay point
					if (goTo(m_lobbyPoint, 0) == true) m_state = 24;
					break;
				}
	case 24: 	{ // move toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 25;
					break;
				}
	case 25: 	{ // rotate toward relay point
					broadcastMsg("Room_reached");
					LOG_MSG(("Room_reached"));
					m_lastPoint= m_lobbyPoint;
					PathinFrontObject(room_msg,m_pointedObject);
					break;
				}

	// move toward the bed room

	case 30: 	{ // set rotation for relay point
					if (goTo(m_relayPoint1, 0) == true) m_state = 31;
					break;
				}
	case 31: 	{ // rotate toward relay point
					if (goTo(m_relayPoint4, 0) == true) m_state = 32;
					break;
				}
	case 32: 	{ // move toward relay point
					if (goTo(m_bedroomPoint, 0) == true) m_state = 33;
					break;
				}
	case 33: 	{ // rotate toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 34;
					break;
				}
	case 34: 	{ // move toward relay point
					broadcastMsg("Room_reached");
					LOG_MSG(("Room_reached"));
					m_lastPoint = m_bedroomPoint;
					PathinFrontObject(room_msg,m_pointedObject);
					break;
				}

	// move toward the bed room

	case 40: 	{ //set rotation for relay point

					if (goTo(m_livingroomPoint, 0) == true) m_state = 41;
					break;
				}
	case 41: 	{ // rotate toward bed room point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 42;
					break;
				}
	case 42: 	{ // move toward bed room point
					broadcastMsg("Room_reached");
					LOG_MSG(("Room_reached"));
					m_lastPoint = m_livingroomPoint;
					PathinFrontObject(room_msg,m_pointedObject);
					break;
				}

	// move back to the kitchen

	case 50: 	{ // set rotation for relay point
					if (goTo(m_relayPoint3, 0) == true) m_state = 51;
					break;
				}
	case 51: 	{ // rotate toward relay point
					if (goTo(m_relayPoint2, 0) == true) m_state = 52;
					break;
				}
	case 52: 	{ // move toward relay point
					if (goTo(m_relayPoint1, 0) == true) m_state = 55;
					break;
				}
	case 55: 	{ // rotate toward relay point
					if (goTo(m_originPoint, 0) == true) m_state = 56;
					break;
				}
	case 56: 	{ // move toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 200;
					m_lastPoint = m_originPoint;
					break;
				}

	// move back to the lobby

	case 60: 	{ // set rotation for relay point
					if (goTo(m_relayPoint5, 0) == true) m_state = 61;
					break;
				}
	case 61: 	{ // rotate toward relay point
					if (goTo(m_relayPoint4, 0) == true) m_state = 62;
					break;
				}
	case 62: 	{ // move toward relay point
					if (goTo(m_relayPoint1, 0) == true) m_state = 63;
					break;
				}
	case 63: 	{ // rotate toward relay point
					if (goTo(m_originPoint, 0) == true) m_state = 64;
					break;
				}
	case 64: 	{ // move toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 200;
					m_lastPoint = m_originPoint;
					break;
				}

	// move back to the bed room

	case 70: 	{ // set rotation for relay point
					if (goTo(m_relayPoint4, 0) == true) m_state++;
					break;
				}
	case 71: 	{ // rotate toward relay point
					if (goTo(m_relayPoint1, 0) == true) m_state++;
					break;
				}
	case 72: 	{ // move toward relay point
					if (goTo(m_originPoint, 0) == true) m_state++;
					break;
				}
	case 73: 	{ // rotate toward relay point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 200;
					break;
				}

	// move back from the bed room

	case 80: 	{ //set rotation for relay point
					if (goTo(m_originPoint, 0) == true) m_state++;
					break;
				}
	case 81: 	{ // rotate toward bed room point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 200;
					break;
				}

	case 300: 	{ // set rotation for relay point
					// rotate toward relay point
					if (goTo(m_kitchenFront1, 0) == true)
					{
						if(stopfront == 1)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 310: 	{ // move toward relay point
					if (goTo(m_kitchenFront2, 0) == true)
					{
						if(stopfront == 2)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 320: 	{ // move toward relay point
					if (goTo(m_kitchenFront3, 0) == true)
					{
						if(stopfront == 3)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}

					break;
				}
	case 330: 	{ // rotate toward relay point
					if (goTo(m_kitchenFront4, 0) == true)
					{
						if(stopfront == 4)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 340: 	{ // move toward kitchen point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 0;
					m_lastPoint= m_kitchenPoint;
					m_state = 109;
					break;
				}

	case 400: 	{ // set rotation for relay point
					if (goTo(m_lobbyFront1, 0) == true)
					{
						if(stopfront == 1)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 410: 	{ // rotate toward relay point
					if (goTo(m_lobbyFront2, 0) == true)
					{
						if(stopfront == 2)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 420: 	{ // move toward relay point
					if (goTo(m_lobbyFront3, 0) == true)
					{
						if(stopfront == 3)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 430:	{ // rotate toward relay point
					if (goTo(m_lobbyFront4, 0) == true)
					{

						if(stopfront == 4)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 440: 	{ // move toward kitchen point
					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 0;
					m_lastPoint= m_lobbyPoint;
					m_state = 109;
					break;
				}

	case 500: 	{ // set rotation for relay point
					if (goTo(m_bedroomFront1, 0) == true)
					{
						if(stopfront == 1)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 510: 	{ // rotate toward relay point
					if (goTo(m_bedroomFront2, 0) == true)
					{
						if(stopfront == 2)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 520: 	{ // move toward relay point
					if (goTo(m_bedroomFront3, 0) == true)
					{
						if(stopfront == 3)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 530: 	{ // move toward kitchen point
					m_my->setWheelVelocity(0.0, 0.0);
					m_lastPoint= m_bedroomPoint;
					m_state = 109;
					stopfront = 0;
					break;
				}

	case 600: 	{ // set rotation for relay point
					if (goTo(m_livingroomFront1, 0) == true)
					{
						if(stopfront == 1)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 610: 	{ // rotate toward relay point
					if (goTo(m_livingroomFront2, 0) == true)
					{
						if(stopfront == 2)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
	case 620: 	{ // move toward relay point
					if (goTo(m_livingroomFront3, 0) == true)
					{
						if(stopfront == 3)
						{
							stopfront = 0;
							m_frontState = m_state+10;
							m_state = 100;
						}
						else
						{
							m_state = m_state+10;
						}
					}
					break;
				}
          case 630:   { // move toward relay point
          if (goTo(m_livingroomFront4, 0) == true)
          {
            if(stopfront == 4)
            {
              stopfront = 0;
              m_frontState = m_state+10;
              m_state = 100;
            }
            else
            {
              m_state = m_state+10;
            }
          }
          break;
        }
                  case 640:   { // move toward relay point
          if (goTo(m_livingroomFront5, 0) == true)
          {
            if(stopfront == 5)
            {
              stopfront = 0;
              m_frontState = m_state+10;
              m_state = 100;
            }
            else
            {
              m_state = m_state+10;
            }
          }
          break;
        }
	case 650: 	{ // move toward kitchen point

					m_my->setWheelVelocity(0.0, 0.0);
					m_state = 0;
					m_lastPoint= m_livingPoint;
					m_state = 109;
					stopfront = 0;
					break;
				}

	}
	return m_onActionReturn;
}

/*************************************************************************************/

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();
	// 送信者がゴミ認識サービスの場合

	std::size_t found=0;
	std::size_t found2=0;
	std::size_t found3=0;
	std::string task;
	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	bool dest = false;
	if(sender == "man_000")
	{
    Robot_speed  = Change_Robot_speed;
		found = msg.find("Go to the ",0);
		if (found != std::string::npos)
		{
			task = msg.substr(found+10);
		}
		found2 = task.find("  grasp the ",0);
		if (found3 != std::string::npos)
		{
			room_msg = task.substr(0,found2);
		}
		found3 = task.find(" and come back here",found);
		if (found3 != std::string::npos)
		{
			object_msg = task.substr(found2+12,found3-found2-12);
		}

	// kitchen message
		if(room_msg == "living room" )
		{
			m_state = 40; // move to the kitchen

			if(object_msg == "apple" )
				m_pointedObject = "apple_0";
			if(object_msg == "can" )
				m_pointedObject = "can_0";
			if(object_msg == "mugcup" )
				m_pointedObject = "mugcup_0";
			if(object_msg == "petbottle" )
				m_pointedObject = "petbottle_0";
		}

		// lobby message
		if(room_msg == "bed room" )
		{
			m_state = 30; // move to the kitchen

			if(object_msg == "apple" )
				m_pointedObject = "apple_3";
			if(object_msg == "can" )
				m_pointedObject = "can_3";
			if(object_msg == "mugcup" )
				m_pointedObject = "mugcup_3";
			if(object_msg == "petbottle" )
				m_pointedObject = "petbottle_3";
		}

		if(room_msg == "lobby" )
		{
			m_state = 20; // move to the kitchen

			if(object_msg == "apple" )
				m_pointedObject = "apple_1";
			if(object_msg == "can" )
				m_pointedObject = "can_1";
			if(object_msg == "mugcup" )
				m_pointedObject = "mugcup_1";
			if(object_msg == "petbottle" )
				m_pointedObject = "petbottle_1";
		}

		if(room_msg == "kitchen" )
		{
			m_state = 10; // move to the kitchen

			if(object_msg == "apple" )
				m_pointedObject = "apple_2";
			if(object_msg == "can" )
				m_pointedObject = "can_2";
			if(object_msg == "mugcup" )
				m_pointedObject = "mugcup_2";
			if(object_msg == "petbottle" )
				m_pointedObject = "petbottle_2";
		}
	}

if(msg == "Task_end" && sender == "moderator_0" )
{
m_state = 0;
         
         if(m_grasp_left == true)
     {
         //   CParts * parts = m_my->getParts("LARM_LINK7");
           // parts->releaseObj();
            m_grasp_left = false;

     }
         m_my->setJointVelocity("LARM_JOINT0", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT1", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT3", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT4", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT5", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT6", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT7", 0.0,0.0);
         m_my->setWheelVelocity(0,0);
}
if(msg == "Mission_complete" && sender == "moderator_0" )
{
m_state = 0;
         m_my->setJointVelocity("LARM_JOINT0", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT1", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT3", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT4", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT5", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT6", 0.0,0.0);
         m_my->setJointVelocity("LARM_JOINT7", 0.0,0.0);
         m_my->setWheelVelocity(0,0);
}
}

/*************************************************************************************/

void RobotController::onCollision(CollisionEvent &evt)
{

}

/*************************************************************************************/

void RobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
  // get object of trash selected
  SimObj *trash = getObj(name.c_str());

  // get trash's position
  trash->getPosition(pos);
}

/*************************************************************************************/

double RobotController::PathinFrontObject(std::string room, std::string object)
{

	Vector3d obj_pos;
	Vector3d dist;
	double distance;
	distance = 0;
	double distance2;
	distance2 = 10000;
	obj_pos =  Vector3d(20, 30, -200);
	SimObj *obj = getObj(object.c_str());
	// get trash's position
	obj->getPosition(obj_pos);

	m_object = object;
	if(room == "kitchen" )
	{
		m_state = 300;

		m_table = "Kitchen Table1";
		m_object = object;
		go_to = PointApproachObj(m_table, m_object, 60);

		for(int i=0;i<m_frontkitchen.size();i++)
		{

			dist = go_to;
			dist  -= m_frontkitchen[i];
			distance = dist.length();
			if(distance < distance2)
			{
				distance2 = distance;
				stopfront = i+1;
			}

		}
		distance2 = 1000;
		distance = 0;
	}

	else if (room == "lobby" )
	{
		std::string m_table1 =  "Buffet2";
		std::string m_table2 = "Side board1";

		SimObj *obj = getObj(m_object.c_str());
		SimObj *tab1 = getObj(m_table1.c_str());
		SimObj *tab2 = getObj(m_table2.c_str());
		Vector3d obj_pos, tab1_pos, tab2_pos;
		obj->getPosition(obj_pos);
		tab1->getPosition(tab1_pos);
		tab2->getPosition(tab2_pos);

		double dist1 = getDist2D(obj_pos, tab1_pos);
		double dist2 = getDist2D(obj_pos, tab2_pos);
		if (dist1 < dist2)
		  m_table = m_table1;
		else
		  m_table = m_table2;

		m_object = object;
		go_to = PointApproachObj(m_table, m_object, 60);
		m_state = 400;
		for(int j=0;j<m_frontlooby.size();j++)
		{
			dist = go_to;
			dist  -= m_frontlooby[j];

			distance = dist.length();
			if(fabs(distance)< fabs(distance2))
			{
				 distance2 = distance;
				 stopfront = j+1;
			}
		}
		distance2 = 1000;
		distance = 0;
	}

	else if (room == "bed room" )
	{
		std::string m_table1 =  "Side board2";
		std::string m_table2 = "Buffet1";

		SimObj *obj = getObj(m_object.c_str());
		SimObj *tab1 = getObj(m_table1.c_str());
		SimObj *tab2 = getObj(m_table2.c_str());
		Vector3d obj_pos, tab1_pos, tab2_pos;
		obj->getPosition(obj_pos);
		tab1->getPosition(tab1_pos);
		tab2->getPosition(tab2_pos);

		double dist1 = getDist2D(obj_pos, tab1_pos);
		double dist2 = getDist2D(obj_pos, tab2_pos);
		if (dist1 < dist2)
		  m_table = m_table1;
		else
		  m_table = m_table2;

		m_object = object;
		go_to = PointApproachObj(m_table, m_object, 45);

		m_state = 500;
		for(int k=0;k<m_frontbedroom.size();k++)
		{
			dist = obj_pos;
			dist  -= m_frontbedroom[k];
			distance = dist.length();

			if(fabs(distance)< fabs(distance2))
			{
				 distance2 = distance;
				 stopfront = k+1;
			}
		}
		distance2 = 1000;
		distance = 0;
	}

	else if (room == "living room" )
	{
		m_table = "Dinner table1";

		std::string m_table1 =  "Dinner table1";
		std::string m_table2 = "Couch_table1";
		std::string m_table3 = "Buffet1";

		SimObj *obj = getObj(m_object.c_str());
		SimObj *tab1 = getObj(m_table1.c_str());
		SimObj *tab2 = getObj(m_table2.c_str());
		SimObj *tab3 = getObj(m_table3.c_str());

		Vector3d obj_pos, tab1_pos, tab2_pos,tab3_pos ;
		obj->getPosition(obj_pos);
		tab1->getPosition(tab1_pos);
		tab2->getPosition(tab2_pos);
		tab3->getPosition(tab3_pos);
		double dist1 = getDist2D(obj_pos, tab1_pos);
		double dist2 = getDist2D(obj_pos, tab2_pos);
		double dist3 = getDist2D(obj_pos, tab3_pos);
		if (dist1 < dist2 && dist1 < dist3)
			m_table = m_table1;
		else if (dist2 < dist1 && dist2 < dist3)
		{
			m_table = m_table2;
		}
		else if (dist3 < dist1 && dist3 < dist2)
			m_table = m_table3;

		m_object = object;
		go_to = PointApproachObj(m_table, m_object, 45);

		m_state = 600;
		for(int m=0;m<m_frontlivingroom.size();m++)
		{
			dist = obj_pos;
			dist  -= m_frontlivingroom[m];
			distance = dist.length();
			if(fabs(distance)< fabs(distance2))
			{
				 distance2 = distance;
				 stopfront = m+1;
			}
		}
		distance2 = 1000;
		distance = 0;
	}
}


//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}
