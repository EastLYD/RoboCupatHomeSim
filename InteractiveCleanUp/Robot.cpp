#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


// The robot must release the Object before to start a new task

double tab_joint_left[6][7] = {{0,0,0,0,0,0,0},{0.5,-0.8,-0.2,-1.5,-0.2,0,0},{-0.25,-0.6,-0.9,-0.65,-0.3,0,0},{0,-1,-1.3,-1.3,0,0,0},{0,-1,-1,-1,0,0,0},{0,0,0,-1.5,0,0,0}};

double tab_joint_right[6][7] = {{0,0,0,0,0,0,0},{-0.5,-0.5,0.2,-1.5,0,0,0},{0,-1.1,1.27,-0.7,0.1,0,0},{0,-1.1,1.57,-1.57,0.3,0,0},{0,-1,1,-1,0,0,0},{0,0,0,-1.5,0,0,0}};

#define error_angle 0.12
#define error_distance 1.3
#define error_angle_arm 0.07


//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

	void   onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void   onRecvMsg(RecvMsgEvent &evt);
	void   onCollision(CollisionEvent &evt);
	void   stopRobotMove(void);

	//function use by other function
	void   recognizeObjectPosition(Vector3d &pos, std::string &name);
	double getAngularXonVect(Vector3d pos, Vector3d mypos);
	double getDistoObj(Vector3d pos, Vector3d pos2);
	double getDist3D(Vector3d pos, Vector3d pos2);
	double getRoll(Rotation rot);
	
	
	//function for the left arm
	bool moveLeftArm();
	void grasp_left_hand();
	void release_left_hand();
	bool goTo(Vector3d pos, double rangeToPoint);
	void chooze_task_arm_left(int task);
	bool moveRightArm();
	void chooze_task_arm_right(int task);
	void get_Kinect_Data();
	void Kinect_Data_Off();
	void Record_Kinect_Data(char* all_msg);
    void PrintPosture();


private:
	// New defifinitions
	RobotObj *my;
	//define different joint of right and left arm
	double joint_left[7];
	double joint_right[7];
	double m_joint_left[7];
	double m_joint_right[7];
	
	//grasping
	bool m_grasp_left;
	double joint_veloc;
	

	double m_time;
	int m_state;

	int cycle;
	//   Vector3d go_to;
	double Robot_speed ;
	double Change_Robot_speed;

	Vector3d m_BottleFront ;
	Vector3d m_MuccupFront;
	Vector3d m_CanFront;

	Vector3d m_BurnableFront;
	Vector3d m_UnburnableFront;
	Vector3d m_RecycleFront;
	Vector3d m_relayPoint1;
	Vector3d m_relayPoint2;
	Vector3d m_relayFrontTable;
	Vector3d m_relayFrontTrash;
	Vector3d m_Object_togo;
	Vector3d m_Trash_togo;
	Vector3d m_relayFrontTable_reset;

	// 取りにいくオブジェクト名
	std::string m_pointedObject;


	// pointed trash
	std::string m_pointedtrash;

	// onActionの戻り値
	double m_onActionReturn;

	// ゴミ候補オブジェクト
	std::vector<std::string> m_trashes;
	// ゴミ箱オブジェクト
	std::vector<std::string> m_trashboxs;


	// 車輪半径
	double m_radius;

	// 車輪間距離
	double m_distance;

	// grasp中かどうか
	bool m_grasp;
	bool take_action;



//////////////  Kinect Data  //////////////////

	struct Joint_Coorditate {
		std::string Name;
		double qw;
		double qx;
		double qy;
		double qz;
	};

	struct Posture_Coordinates {
		double  time;
		std::vector <Joint_Coorditate> posture;

	};

	std::vector <Posture_Coordinates> Record_Postures;

////////////////////////////////////////////


///////////Flag On_message ////////////////

	bool Kinect_data_flag;

//////////////////////////////////////////

};


void RobotController::stopRobotMove(void)
{
	my->setWheelVelocity(0.0, 0.0);
}


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
	if (l_pos.z() < 0 && l_pos.x()>=0)
		targetAngle = M_PI+targetAngle;
	else if (l_pos.z()<0 && l_pos.x()<0)
		targetAngle = targetAngle-M_PI;

	return targetAngle;
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

	if (joint_left[0] != m_joint_left[0] )
		{
			if (joint_left[0] < m_joint_left[0] && m_joint_left[0]-joint_left[0] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT0", joint_veloc, 0.0);
					joint_left[0] = my->getJointAngle("LARM_JOINT0");
				}
			else if (joint_left[0] > m_joint_left[0] && joint_left[0]-m_joint_left[0] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT0", -joint_veloc, 0.0);
					joint_left[0] = my->getJointAngle("LARM_JOINT0");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
					j0 = true;
				}

		}
	else j0 = true;

	if (joint_left[1] != m_joint_left[1] )
		{
			if (joint_left[1] < m_joint_left[1] && m_joint_left[1]-joint_left[1] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT1", joint_veloc, 0.0);
					joint_left[1] = my->getJointAngle("LARM_JOINT1");
				}
			else if (joint_left[1] > m_joint_left[1] && joint_left[1]-m_joint_left[1] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT1", -joint_veloc, 0.0);
					joint_left[1] = my->getJointAngle("LARM_JOINT1");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
					j1 = true;
				}
		}
	else j1 = true;

	if (joint_left[2] != m_joint_left[2] )
		{
			if (joint_left[2] < m_joint_left[2] && m_joint_left[2]-joint_left[2] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT3", joint_veloc, 0.0);
					joint_left[2] = my->getJointAngle("LARM_JOINT3");
				}
			else if (joint_left[2] > m_joint_left[2] && joint_left[2]-m_joint_left[2] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT3", -joint_veloc, 0.0);
					joint_left[2] = my->getJointAngle("LARM_JOINT3");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
					j3 = true;
				}
		}
	else j3 = true;

	if (joint_left[3] != m_joint_left[3] )
		{
			if (joint_left[3] < m_joint_left[3] && m_joint_left[3]-joint_left[3] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT4", joint_veloc, 0.0);
					joint_left[3] = my->getJointAngle("LARM_JOINT4");
				}
			else if (joint_left[3] > m_joint_left[3] && joint_left[3]-m_joint_left[3] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT4", -joint_veloc, 0.0);
					joint_left[3] = my->getJointAngle("LARM_JOINT4");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
					j4 = true;
				}
		}
	else j4 = true;

	if (joint_left[4] != m_joint_left[4] )
		{
			if (joint_left[4] < m_joint_left[4] && m_joint_left[4]-joint_left[4] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT5", joint_veloc, 0.0);
					joint_left[4] = my->getJointAngle("LARM_JOINT5");
				}
			else if (joint_left[4] > m_joint_left[4] && joint_left[4]-m_joint_left[4] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT5", -joint_veloc, 0.0);
					joint_left[4] = my->getJointAngle("LARM_JOINT5");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
					j5 = true;
				}
		}
	else j5 = true;

	if (joint_left[5] != m_joint_left[5] )
		{
			if (joint_left[5] < m_joint_left[5] && m_joint_left[5]-joint_left[5] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT6", joint_veloc, 0.0);
					joint_left[5] = my->getJointAngle("LARM_JOINT6");
				}
			else if (joint_left[5] > m_joint_left[5] && joint_left[5]-m_joint_left[5] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT6", -joint_veloc, 0.0);
					joint_left[5] = my->getJointAngle("LARM_JOINT6");
				}
			else
				{
					my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
					j6 = true;
				}
		}
	else j6 = true;

	if (joint_left[6] != m_joint_left[6] )
		{
			if (joint_left[5] < m_joint_left[6] && m_joint_left[6]-joint_left[5] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT7", joint_veloc, 0.0);
					joint_left[5] = my->getJointAngle("LARM_JOINT7");
				}
			else if (joint_left[5] > m_joint_left[6] && joint_left[5]-m_joint_left[6] > error_angle_arm)
				{
					my->setJointVelocity("LARM_JOINT7", -joint_veloc, 0.0);
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





/************************************************************************************/

/************************************************************************************/
/****************************function for the right arm*******************************/
/************************************************************************************/

bool RobotController::moveRightArm()
{
	bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;

	if (joint_right[0] != m_joint_right[0] )
		{
			if (joint_right[0] < m_joint_right[0] && m_joint_right[0]-joint_right[0] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT0",joint_veloc, 0.0);
					joint_right[0] = my->getJointAngle("RARM_JOINT0");
				}
			else if (joint_right[0] > m_joint_right[0] && joint_right[0]-m_joint_right[0] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT0",-joint_veloc, 0.0);
					joint_right[0] = my->getJointAngle("RARM_JOINT0");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
					j0 = true;
				}

		}
	else j0 = true;

	if (joint_right[1] != m_joint_right[1] )
		{
			if (joint_right[1] < m_joint_right[1] && m_joint_right[1]-joint_right[1] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT1",joint_veloc, 0.0);
					joint_right[1] = my->getJointAngle("RARM_JOINT1");
				}
			else if (joint_right[1] > m_joint_right[1] && joint_right[1]-m_joint_right[1] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT1",-joint_veloc, 0.0);
					joint_right[1] = my->getJointAngle("RARM_JOINT1");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
					j1 = true;
				}
		}
	else j1 = true;

	if (joint_right[2] != m_joint_right[2] )
		{
			if (joint_right[2] < m_joint_right[2] && m_joint_right[2]-joint_right[2] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT3",joint_veloc, 0.0);
					joint_right[2] = my->getJointAngle("RARM_JOINT3");
				}
			else if (joint_right[2] > m_joint_right[2] && joint_right[2]-m_joint_right[2] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT3",-joint_veloc, 0.0);
					joint_right[2] = my->getJointAngle("RARM_JOINT3");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
					j3 = true;
				}
		}
	else j3 = true;

	if (joint_right[3] != m_joint_right[3] )
		{
			if (joint_right[3] < m_joint_right[3] && m_joint_right[3]-joint_right[3] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT4", joint_veloc, 0.0);
					joint_right[3] = my->getJointAngle("RARM_JOINT4");
				}
			else if (joint_right[3] > m_joint_right[3] && joint_right[3]-m_joint_right[3] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT4",-joint_veloc, 0.0);
					joint_right[3] = my->getJointAngle("RARM_JOINT4");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
					j4 = true;
				}
		}
	else j4 = true;

	if (joint_right[4] != m_joint_right[4] )
		{
			if (joint_right[4] < m_joint_right[4] && m_joint_right[4]-joint_right[4] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT5", joint_veloc, 0.0);
					joint_right[4] = my->getJointAngle("RARM_JOINT5");
				}
			else if (joint_right[4] > m_joint_right[4] && joint_right[4]-m_joint_right[4] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT5", -joint_veloc, 0.0);
					joint_right[4] = my->getJointAngle("RARM_JOINT5");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT5", 0.0, 0.0);
					j5 = true;
				}
		}
	else j5 = true;

	if (joint_right[5] != m_joint_right[5] )
		{
			if (joint_right[5] < m_joint_right[5] && m_joint_right[5]-joint_right[5] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT6", joint_veloc, 0.0);
					joint_right[5] = my->getJointAngle("RARM_JOINT6");
				}
			else if (joint_right[5] > m_joint_right[5] && joint_right[5]-m_joint_right[5] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT6", -joint_veloc, 0.0);
					joint_right[5] = my->getJointAngle("RARM_JOINT6");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT6", 0.0, 0.0);
					j6 = true;
				}
		}
	else j6 = true;

	if (joint_right[6] != m_joint_right[6] )
		{
			if (joint_right[5] < m_joint_right[6] && m_joint_right[6]-joint_right[5] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT7", joint_veloc, 0.0);
					joint_right[5] = my->getJointAngle("RARM_JOINT7");
				}
			else if (joint_right[5] > m_joint_right[6] && joint_right[5]-m_joint_right[6] > error_angle_arm)
				{
					my->setJointVelocity("RARM_JOINT7", -joint_veloc, 0.0);
					joint_right[5] = my->getJointAngle("RARM_JOINT7");
				}
			else
				{
					my->setJointVelocity("RARM_JOINT7", 0.0, 0.0);
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


void RobotController::chooze_task_arm_right(int task)
{
	for (int i=0; i<7; i++)
		m_joint_right[i] = tab_joint_right[task][i];
}




void RobotController::grasp_left_hand()
{
	Vector3d hand, object;
	SimObj *obj = getObj(m_pointedObject.c_str());

	obj->getPosition(object);
	my->getJointPosition(hand, "LARM_JOINT7");

	double distance = getDist3D(hand,object);

	if (distance < 22 &&  m_grasp_left == false)
		{
			CParts * parts = my->getParts("LARM_LINK7");
			if (parts->graspObj(m_pointedObject))
				{
					m_grasp_left = true;
					//  broadcastMsg("Object_grasped");
				}
		}
}


void RobotController::release_left_hand()
{
	CParts * parts = my->getParts("LARM_LINK7");
	if ( m_grasp_left == true){
		printf("I'm releasing \n");
	}
	parts->releaseObj();
	m_grasp_left = false;
}
/*************************************************************************************/

/*************************************************************************************/

void RobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
}

/*************************************************************************************/


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
					if ( dist-rangeToPoint > 0 )
						my->setWheelVelocity(1, 1);
					else
						my->setWheelVelocity(-1, -1);
				else if ( dist-rangeToPoint > 0 )
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







void RobotController::PrintPosture()
{

	for(int i = 0; i < Record_Postures.size(); i++){

	std::cout << "The Time .....  " <<  Record_Postures[i].time << std::endl;
		   std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
		   	   std::cout << "Time :    " << Record_Postures[i].time << std::endl;
					for(int j = 0; j < Record_Postures[i].posture.size(); j++){
	   std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
	   std::cout << "the name" <<  Record_Postures[i].posture[j].Name << " qw "  << Record_Postures[i].posture[j].qw << " qx "  << Record_Postures[i].posture[j].qx << " qy "  << Record_Postures[i].posture[j].qy <<  " qz "  << Record_Postures[i].posture[j].qz << std::endl;

								}

		}

}




void RobotController::onInit(InitEvent &evt)
{
	joint_veloc = 0.8;

	my = getRobotObj(myname());

	// 車輪間距離
	m_distance = 10.0;

	// 車輪半径
	m_radius  = 10.0;

	// 車輪の半径と車輪間距離設定
	my->setWheel(m_radius, m_distance);

	m_grasp_left = false;
	Change_Robot_speed = 5; // to change the robot's velocity
	Robot_speed  = Change_Robot_speed;
	m_state = 0;
	// bjects
	m_BottleFront = Vector3d(40.0, 30, -40.0);
	m_MuccupFront = Vector3d(0.0, 30, -40.0);
	m_CanFront = Vector3d(-40.0, 30, -40.0);



	// trash
	m_BurnableFront = Vector3d(-120.0, 30, -60);
	m_UnburnableFront = Vector3d(-120.0, 30, 70);
	m_RecycleFront = Vector3d(-60.0, 30, -100);



	m_relayPoint1 = Vector3d(100, 30, -70);
	m_relayPoint2 = Vector3d(0, 30, -70);
	m_relayFrontTable = Vector3d(0, 30,-20);

	m_relayFrontTable_reset = Vector3d(-80, 30,-50);
	m_relayFrontTrash = Vector3d(-80, 30, -80);

	cycle = 3;
	//エージェントの正面の定義はz軸の正の向きを向いていると仮定する

	m_onActionReturn = 0.01;


	srand((unsigned)time( NULL ));


	// grasp初期化
	m_grasp = false;


	m_trashes.push_back("petbottle");

	m_trashes.push_back("mugcup");

	m_trashes.push_back("can");


	// ゴミ箱登録
	m_trashboxs.push_back("recycle");
	m_trashboxs.push_back("burnable");
	m_trashboxs.push_back("unburnable");
}


double RobotController::onAction(ActionEvent &evt)
{
	//std::cout << "m_state " <<  m_state << std::endl;
	//std::cout << "the size of Vector " <<  Record_Postures.size() << std::endl;
	switch(m_state)
		{
		case 1: {
			Robot_speed  = Change_Robot_speed;
			chooze_task_arm_left(5);
			chooze_task_arm_right(5);
			//   printf("got it in case!1 flag1 \n");
			if (goTo(m_relayPoint1, 0) == true && moveLeftArm() == true && moveRightArm() == true)
				{
					m_state = 2;
					//  printf("got it in case!1 \n");

				}
			break;
		}
		case 2:   { 
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayPoint2, 0) == true) m_state = 3;
			break;
		}
		case 3: {
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTable, 0) == true) m_state = 4;
			break;
		}
		case 4: {  // Test if the cycle is finished or not
			if (cycle > 0)
				{
					m_state = 41;
					broadcastMsg("Show_me");
					get_Kinect_Data();
					m_time = evt.time() + 5;
					break;
				}
			else {
					m_state = 49;
					break;
			}
		}
		case 41: {
			if(evt.time() > m_time) m_state = 42;
			break;
		}
		case 42:{
				Kinect_Data_Off(); // finished analysing data
				m_pointedObject = "petbottle";
				m_pointedtrash = "recycle";
				std::cout << "Task started Robot ........ "  << std::endl;
			
				m_state = 5;
			}
		case 49:
			{
				break;
			}

		case 5:   {  //Optional case  !!!
			
			Robot_speed  = Change_Robot_speed;
			if (m_pointedObject=="petbottle")
				{
					if (goTo(m_BottleFront, 0) == true) m_state = 6;
				}

			else if (m_pointedObject=="mugcup")
				{
					if (goTo(m_MuccupFront, 0) == true) m_state = 6;
				}
			else if (m_pointedObject=="can")
				{
					if (goTo(m_CanFront, 0) == true)  m_state = 6;
				}
			break;

		}
		case 6:   { //preparation of the arm for grasp
			Robot_speed  = Change_Robot_speed;
			recognizeObjectPosition(m_Object_togo, m_pointedObject);
			if (goTo(m_Object_togo, 70) == true)
				{
					m_state = 7;

				}
			break;
		}

		case 7:   { //preparation of the arm for grasp
			chooze_task_arm_left(1);
			if (moveLeftArm() == true) m_state = 8;
			break;
		}
		case 8:   { //move to the object
			Robot_speed  = 1;
			if (goTo(m_Object_togo, 38) == true) m_state = 9;
			break;
		}
		case 9:   { //move arm to grasp the object
			chooze_task_arm_left(2);
			grasp_left_hand();
			if (moveLeftArm() == true) m_state = 10;
			break;
		}
		case 10:   {
			m_state = 11;
			break;
		}
		case 11:   { //move arm to place in good position for moving
			grasp_left_hand();
			chooze_task_arm_left(3);
			if (moveLeftArm() == true)
				m_state = 12;
			break;
		}
		case 12:  {
			my->setWheelVelocity(-1.4,-1.4);
			chooze_task_arm_left(5);
			if (moveLeftArm() == true)
				{
					Robot_speed  = Change_Robot_speed;
					m_state = 13;
					sleep(1);
				}
			break;
		}
		case 13:   { //move to the object
			// Robot_speed  = 1;
			if (goTo(m_relayFrontTrash, 0) == true)
				{

					m_state = 14;
				}
			break;
		}


		case 14: {

			if (goTo(m_relayFrontTable_reset, 0) == true)
				{


					sleep(2);
					m_state = 15;
				}
			break;
		}

		case 99:   { //move to the object
			// Robot_speed  = 1;
			break;
		}

		case 15:   { //Optional case  !!!
			Robot_speed  = Change_Robot_speed;
			if (m_pointedtrash=="recycle")
				{
					if (goTo(m_RecycleFront, 0) == true)  m_state = 16;
				}
			else if (m_pointedtrash=="burnable")
				{
					if (goTo(m_BurnableFront, 0) == true) m_state = 16;
				}
			else if (m_pointedtrash=="unburnable")
				{
					if (goTo(m_UnburnableFront, 0) == true) m_state = 16;
				}
			break;
		}
		case 16:   { //preparation of the arm for grasp
			recognizeObjectPosition(m_Trash_togo, m_pointedtrash);
			if (goTo(m_Trash_togo, 50) == true)
				{
					m_state = 17;

				}
			break;
		}

		case 17:   { //preparation of the arm for grasp
			chooze_task_arm_left(1);
			if (moveLeftArm() == true) m_state = 18;
			break;
		}
		case 18:   {
			Robot_speed  = 1;
			if (goTo(m_Trash_togo, 60) == true) m_state = 19;
			break;
		}
		case 19:   { //move arm to grasp the object
			chooze_task_arm_left(3);

			if (moveLeftArm() == true)
				{
					m_state = 20;
					release_left_hand();
				}
			break;
		}
		case 20:  {
			my->setWheelVelocity(-2.5,-2.5);
			chooze_task_arm_left(2);
			if (moveLeftArm() == true)
				{

					Robot_speed  = Change_Robot_speed;
					m_state = 21;
				}
			break;
		}
		case 21:   { //move to the object
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTrash, 0) == true) m_state = 22;
			break;
		}
		case 22:   { //preparation of the arm for grasp
			chooze_task_arm_left(5);
			if (moveLeftArm() == true) m_state = 23;
			break;
		}
		case 23:   { //move to the object
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTable, 0) == true)
				{
					cycle = cycle-1;
					m_state = 4;
					broadcastMsg("Task_finished");
					m_pointedtrash = "";
					m_pointedObject = "";
					m_state = 0;
				}
			break;
		}
			

		}
	return m_onActionReturn;
}


void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();

	char *all_msg = (char*)evt.getMsg();
	std::string msg;
	msg= evt.getMsg();

/////////////////////////////////// On_Message //////////////////////////////
	std::string ss = all_msg;
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find("  ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if (msg == "Task_start" && m_state == 0)
		{
			m_state = 1 ;      
			Kinect_data_flag = false;
			Record_Postures.clear();
		}

	char* m_msg = strtok(all_msg,"  ");

	if (strcmp(m_msg,"KINECT_DATA_Sensor") == 0 && Kinect_data_flag == true) {
		char* kinect_msg = (char*)msg.c_str();
		Record_Kinect_Data(kinect_msg);
	}

/////////////////////////////////////////////////////////////////////////////

}



void RobotController::Record_Kinect_Data(char* all_msg)  //   
{
	float qw ;
	float qx ;
	float qy ;
	float qz ;
	std::string name;
	float m_time;				
	char* m_msg = strtok(all_msg,"  ");
	Posture_Coordinates m_posture;

	if (strcmp(m_msg,"KINECT_DATA_Sensor") == 0 ) {
		int i = 0;
		while (true) {
			Joint_Coorditate m_joint;
			char *type = strtok(NULL,":");

			if (strcmp(type,"END") == 0) {
				m_time = atof(strtok(NULL,".."));
				m_posture.time = m_time;
				break;
			}
			else {
				m_joint.Name = type;
				m_joint.qw = atof(strtok(NULL,","));
				m_joint.qx = atof(strtok(NULL,","));
				m_joint.qy = atof(strtok(NULL,","));
				m_joint.qz = atof(strtok(NULL," "));
			}
			m_posture.posture.push_back(m_joint);
		}
		Record_Postures.push_back(m_posture);
	}
}


void RobotController::onCollision(CollisionEvent &evt)
{
	if (m_grasp == false){
		//typedef CollisionEvent::WithC C; //TODO: What's this? should be removed.
		//触れたエンティティの名前を得ます
		const std::vector<std::string> & with = evt.getWith();
		// 衝突した自分のパーツを得ます
		const std::vector<std::string> & mparts = evt.getMyParts();
		//　衝突したエンティティでループします
		for(int i = 0; i < with.size(); i++){
			//右手に衝突した場合
			if (mparts[i] == "RARM_LINK7"){
				//自分を取得
				SimObj *my = getObj(myname());
				//自分の手のパーツを得ます
				CParts * parts = my->getParts("RARM_LINK7");
				//if (parts->graspObj(with[i])){
				//	m_grasp = true;
				//}
			}
		}
	}
}


void RobotController::get_Kinect_Data()
{
	sendMsg("Kinect_000","Get_Data");
	Kinect_data_flag = true;
	std::cout << "Start recording " <<  std::endl;
}

void RobotController::Kinect_Data_Off()
{
	sendMsg("Kinect_000","Data_Off");
	Kinect_data_flag = false;
	std::cout << "Stop recording " <<  std::endl;
}


//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
	return new RobotController;
}

