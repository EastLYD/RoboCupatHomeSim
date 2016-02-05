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

#define AVATAR_NAME	"man_000"

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
	bool   recognizeTrash(Vector3d &pos, std::string &name);
	void   stopRobotMove(void);

    //function use by other function
    void   recognizeObjectPosition(Vector3d &pos, std::string &name);
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
    void grasp_right_hand();
    void release_right_hand();
    bool goTo(Vector3d pos, double rangeToPoint);
    void chooze_task_arm_left(int task);
    bool moveRightArm();
    void chooze_task_arm_right(int task);
    void get_Kinect_Data();
    void Kinect_Data_Off();
    void Record_Kinect_Data(char* all_msg);


	/* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	 * @param  pos 回転したい方向の位置
	 * @param  vel 回転速度
	 * @param  now 現在時間
	 * @return 回転終了時間
	 */
	double rotateTowardObj(Vector3d pos, double vel, double now);

	/* @brief  位置を指定しその方向に進みます
	 * @param  pos   行きたい場所
	 * @param  vel   移動速度
	 * @param  range 半径range以内まで移動
	 * @param  now   現在時間
	 * @return 到着時間
	 */
	double goToObj(Vector3d pos, double vel, double range, double now);

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
    //to define the object and the table
    std::string m_object, m_table;

    int m_robotState;
    int m_state;
    int m_frontState;
    int stopfront;


	int cycle;
	//   Vector3d go_to;
    double Robot_speed ;
    double Change_Robot_speed;

	Vector3d m_BottleFront ;
	Vector3d m_MuccupFront;
	Vector3d m_CanFront;

	Vector3d m_WagonFront;
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




	Vector3d mO_RightFront;
	Vector3d mO_CenterFront;
	Vector3d mO_LeftFront;

	Vector3d mT_RightFront;
	Vector3d mT_CenterFront;
	Vector3d mT_LeftFront;



	// Takeitの対称から除外するオブジェクトの名前
	std::vector<std::string> exobj;
	std::vector<std::string> extrash;
	RobotObj *m_my;
	Vector3d m_tpos;

	//状態
	/***
	 * @brief ロボットの状態
	 * 0 "TAKEIT"の指示を待っている状態
	 * 1 取りに行くオブジェクトを特定中
	 * 2 オブジェクトに向けて移動中
	 * 3 オブジェクトを取得し戻る途中
	 */
	// int m_robotState;
	// int m_state;

	// エージェントの正面の方向ベクトル通常はz軸の正の方向と考える
	Vector3d m_defaultNormalVector;

	// 取りにいくオブジェクト名
	std::string m_pointedObject;


	// pointed trash
	std::string m_pointedtrash;
	// 指示を受けたアバター名
	std::string m_avatar;

	// onActionの戻り値
	double m_onActionReturn;

	// 1stepあたりの移動距離
	double m_speedDelta;

	// ゴミの名前
	std::string m_tname;
	// ゴミ候補オブジェクト
	std::vector<std::string> m_trashes;
	// ゴミ箱オブジェクト
	std::vector<std::string> m_trashboxs;

	// 車輪の角速度
	double m_vel;

	// 関節の回転速度
	double m_jvel;

	// 車輪半径
	double m_radius;

	// 車輪間距離
	double m_distance;

	// 移動終了時間
	double m_time;
	double m_time_LA1;
	double m_time_LA4;
	double m_time_RA1;
	double m_time_RA4;

	// 初期位置
	Vector3d m_inipos;

	Vector3d pos_a;
	Vector3d pos_b;

	// grasp中かどうか
	bool m_grasp;

	// ゴミ認識サービス
	BaseService *m_recogSrv;

	// サービスへのリクエスト複数回送信を防ぐ
	bool m_sended;

	std::string msg_ob;
	std::string msg_trash;
	Vector3d  m_BottleReset;
	Vector3d  m_MuccupReset;
	Vector3d  m_CanReset;
	Vector3d  m_RobotPos;
	Vector3d  m_Table ;
	Vector3d  Object_reset;
	Rotation rot;
	bool reset_op;
	bool reset_out;
	bool take_action;
	bool put_action;


//////////////  Kinect Data  //////////////////

struct Joint_Coorditate {
   std::string Name;
   double qw;
   double qx;
   double qy;
   double qz;
} ;

struct Posture_Coordinates {
  double  time;
  std::vector <Joint_Coorditate> posture;

} ;


/////////

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
	if (l_pos.z() < 0 && l_pos.x()>=0)
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

void RobotController::grasp_right_hand()
{
	Vector3d hand, object;
	SimObj *obj = getObj(m_pointedObject.c_str());

	obj->getPosition(object);
	my->getJointPosition(hand, "RARM_JOINT7");

	double distance = getDist3D(hand,object);

	if (distance < 13 &&  m_grasp_left == false)
		{
			CParts * parts = my->getParts("RARM_LINK7");
			if (parts->graspObj(m_pointedObject))
				{
					m_grasp_left = true;
					//  broadcastMsg("Object_grasped");
				}
		}
}


void RobotController::release_left_hand()
{
	CParts * parts = m_my->getParts("LARM_LINK7");
	if ( m_grasp_left == true)
		printf("I'm releasing \n");
    parts->releaseObj();
	m_grasp_left = false;
}


void RobotController::release_right_hand()
{
	CParts * parts = m_my->getParts("RARM_LINK7");
	if ( m_grasp_left == true)
		printf("I'm releasing \n");
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





void RobotController::onInit(InitEvent &evt)
{
	m_robotState = 0;
	m_frontState = 0;
	joint_veloc = 0.8;

	m_my = getRobotObj(myname());
	m_my->setWheel(10.0, 10.0);
	my = getRobotObj(myname());
	my->setWheel(10.0, 10.0);
	stopfront = 0;

	m_grasp_left = false;
	Change_Robot_speed = 5; // to change the robot's velocity
	Robot_speed  = Change_Robot_speed;
	m_state = 0;
	// bjects
	m_BottleFront = Vector3d(40.0, 30, -40.0);
	m_MuccupFront = Vector3d(0.0, 30, -40.0);
	m_CanFront = Vector3d(-40.0, 30, -40.0);

	mO_RightFront = Vector3d(40.0, 30, -10.0);
	mO_CenterFront = Vector3d(0.0, 30, -10.0);
	mO_LeftFront = Vector3d(-40.0, 30, -10.0);


	// trash
	//m_WagonFront = Vector3d(-120.0, 30, -120);
	m_BurnableFront = Vector3d(-120.0, 30, -60);
	m_UnburnableFront = Vector3d(-120.0, 30, 70);
	m_RecycleFront = Vector3d(-60.0, 30, -100);

	mT_RightFront = Vector3d(-120.0, 30, -190);
	mT_CenterFront = Vector3d(-120.0, 30, -60);
	mT_LeftFront = Vector3d(-120.0, 30, 70);




	m_relayPoint1 = Vector3d(100, 30, -70);
	m_relayPoint2 = Vector3d(0, 30, -70);
	m_relayFrontTable = Vector3d(0, 30,-20);

	m_relayFrontTable_reset = Vector3d(-80, 30,-50);
	m_relayFrontTrash = Vector3d(-80, 30, -80);






	cycle = 3;
	m_robotState = 0;
	//エージェントの正面の定義はz軸の正の向きを向いていると仮定する
	m_defaultNormalVector = Vector3d(0,0,1);
	m_onActionReturn = 1.0;
	m_speedDelta = 2.0;

	m_avatar = AVATAR_NAME;

	m_my = getRobotObj(myname());

	// 初期位置取得
	//m_my->getPosition(m_inipos);
	m_my->getPartsPosition(m_inipos,"RARM_LINK2");

	pos_a = Vector3d(0, 30, -80);//
	pos_b = Vector3d(0, 30, -10);  ///

	// 車輪間距離
	m_distance = 10.0;

	// 車輪半径
	m_radius  = 10.0;

	// 移動終了時間初期化
	m_time = 0.0;
	m_time_LA1 = 0.0;
	m_time_LA4 = 0.0;
	m_time_RA1 = 0.0;
	m_time_RA4 = 0.0;

	// 車輪の半径と車輪間距離設定
	m_my->setWheel(m_radius, m_distance);
	// m_state = 20;

	srand((unsigned)time( NULL ));

	// 車輪の回転速度
	m_vel = 0.3;

	// 関節の回転速度
	m_jvel = 0.6;

	// grasp初期化
	m_grasp = false;
	m_recogSrv = NULL;
	m_sended = false;


	m_trashes.push_back("petbottle");

	m_trashes.push_back("mugcup");

	m_trashes.push_back("can");


	// ゴミ箱登録
	m_trashboxs.push_back("recycle");
	m_trashboxs.push_back("burnable");
	m_trashboxs.push_back("unburnable");
	m_trashboxs.push_back("wagon");

	m_BottleReset = Vector3d(40.0, 59.15, 50.0);
	m_MuccupReset = Vector3d(0.0, 52.15, 50.0);
	m_CanReset = Vector3d(-40.0, 54.25, 50.0);
	m_RobotPos = Vector3d(100.0, 30.0,-100.0);
	m_Table = Vector3d(0.0, 24.0,70.0);
	rot.setQuaternion(1, 0, 0, 0);
	reset_op = false;
	reset_out = false;

	take_action = true;
	put_action = true;
}


double RobotController::onAction(ActionEvent &evt)
{
	//std::cout << "m_state " <<  m_state << std::endl;
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
			reset_op = false;
			take_action = true;
			if (cycle > 0)
				{
					m_state = 30;
					sendMsg("man_000","Show_me");
					get_Kinect_Data();
					break;
				}
			else
				{

					m_state = 60;
					break;
                }
		}


		case 30:
			{
				// printf("Start New task \n");

			}

		case 60:
			{
				// printf("Restart the cycle");

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
			// broadcastMsg("Object_grasped");
			//LOG_MSG(("Object_grasped"));
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
			m_my->setWheelVelocity(-1.4,-1.4);
			chooze_task_arm_left(5);
			if (moveLeftArm() == true)
				{
					Robot_speed  = Change_Robot_speed;
				//	sendMsg("VoiceReco_Service"," Now I will go to the trashboxes ");
					
					//sendMsg("moderator_0","Object_Grasped");
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
			else if (m_pointedtrash=="wagon")
				{
					if (goTo(m_WagonFront, 0) == true) m_state = 16;

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
			m_my->setWheelVelocity(-2.5,-2.5);
			chooze_task_arm_left(2);
			if (moveLeftArm() == true)
				{

					Robot_speed  = Change_Robot_speed;
					m_state = 21;
				}
			break;
        }
		case 21:   { //move to the object
			// Robot_speed  = 1;
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
			// Robot_speed  = 1;
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTable, 0) == true)
				{
					cycle = cycle-1;
					m_state = 4;
					sendMsg("moderator_0","Task_finished");
					m_pointedtrash = "";
					m_pointedObject = "";
					m_state = 0;
				}
			break;
        }
			

		}
	return 0.01;
}


void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();

	// 送信者がゴミ認識サービスの場合
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


//////////////////////////////////////////////////////////////////////////////








	if (msg == "Start_Cycle" && m_state == 0)
		{
			m_state = 1 ;      
			Kinect_data_flag = false;
			Record_Postures.clear();
		//	sendMsg("VoiceReco_Service","Let's start the clean up task\n");
		//	printf("got it in go \n");
		}
  





	else if (msg == "Start_Task" && m_state == 30 ) 
		{
            sleep(5);
            Kinect_Data_Off(); // finished analysing data
			m_pointedObject = "petbottle";
			m_pointedtrash = "recycle";
			std::cout << "Task started Robot ........ "  << std::endl;
			
			m_state = 5;
		}




////////////////////////////////////    On Message  /////////////////////////////////////////////////


   char* m_msg = strtok(all_msg,"  ");

if (strcmp(m_msg,"KINECT_DATA_Sensor") == 0 && Kinect_data_flag == true) {

Record_Kinect_Data(all_msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

}



void RobotController::Record_Kinect_Data(char* all_msg)  //   
{
float qw ;
float qx ;
float qy ;
float qz ;
std::string name;
float m_time;				
//RobotObj *my = this->getObj(this->myname());
   char* m_msg = strtok(all_msg,"  ");
   Posture_Coordinates m_posture;

if (strcmp(m_msg,"KINECT_DATA_Sensor") == 0 ) {
		int i = 0;
		while (true) {


Joint_Coorditate m_joint;

			
			  char *type = strtok(NULL,":");

            if (strcmp(type,"END") == 0) {
         	m_time = atof(strtok(NULL,"."));
            m_posture.time = m_time;
				break;
			}
			else
			{

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


bool RobotController::recognizeTrash(Vector3d &pos, std::string &name)
{
	// 候補のゴミが無い場合
	if (m_trashes.empty()){
		return false;
	}

	// ここでは乱数を使ってゴミを決定します
	int trashNum = rand() % m_trashes.size();

	// ゴミの名前と位置を取得します
	name = m_trashes[trashNum];
	SimObj *trash = getObj(name.c_str());

	// ゴミの位置取得
	trash->getPosition(pos);
	return true;
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
				//   if (parts->graspObj(with[i])){
				//    m_grasp = true;
				//   }
			}
		}
	}
}


double RobotController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
	// 自分の位置の取得
	Vector3d myPos;
	//m_my->getPosition(myPos);
	m_my->getPartsPosition(myPos,"RARM_LINK2");

	// 自分の位置からターゲットを結ぶベクトル
	Vector3d tmpp = pos;
	tmpp -= myPos;

	// y方向は考えない
	tmpp.y(0);

	// 自分の回転を得る
	Rotation myRot;
	m_my->getRotation(myRot);

	// エンティティの初期方向
	Vector3d iniVec(0.0, 0.0, 1.0);

	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();

	double theta = 2*acos(fabs(qw));

	if (qw*qy < 0)
		theta = -1*theta;

	// z方向からの角度
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if (tmpp.x() > 0) targetAngle = -1*targetAngle;
	targetAngle += theta;

	if (targetAngle == 0.0){
		return 0.0;
	}
	else {
		// 回転すべき円周距離
		double distance = m_distance*M_PI*fabs(targetAngle)/(2*M_PI);

		// 車輪の半径から移動速度を得る
		double vel = m_radius*velocity;

		// 回転時間(u秒)
		double time = distance / vel;

		// 車輪回転開始
		if (targetAngle > 0.0) {
			m_my->setWheelVelocity(velocity, -velocity);
		}
		else {
			m_my->setWheelVelocity(-velocity, velocity);
		}
		return now + time;
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


// object まで移動
double RobotController::goToObj(Vector3d pos, double velocity, double range, double now)
{
	Vector3d myPos;
	//m_my->getPosition(myPos);
	m_my->getPartsPosition(myPos,"RARM_LINK2");

	// 自分の位置からターゲットを結ぶベクトル
	pos -= myPos;

	// y方向は考えない
	pos.y(0);

	// 距離計算
	double distance = pos.length() - range;

	// 車輪の半径から移動速度を得る
	double vel = m_radius*velocity;

	// 移動開始
	m_my->setWheelVelocity(velocity, velocity);

	// 到着時間取得
	double time = distance / vel;

	return now + time;
}


//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
	return new RobotController;
}
