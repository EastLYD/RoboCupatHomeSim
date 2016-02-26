#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


// The robot must release the Object before to start a new task

double tab_joint_left[6][8] = {{0,0,0,0,0,0,0,0},{0.5,-0.8,0,-0.2,-1.5,-0.2,0,0},{-0.25,-0.6,0,-0.9,-0.65,-0.3,0,0},{0,-1,0,-1.3,-1.3,0,0,0},{0,-1,0,-1,-1,0,0,0},{0,0,0,0,-1.5,0,0,0}};

double tab_joint_right[6][8] = {{0,0,0,0,0,0,0,0},{-0.5,-0.5,0,0.2,-1.5,0,0,0},{0,-1.1,0,1.27,-0.7,0.1,0,0},{0,-1.1,0,1.57,-1.57,0.3,0,0},{0,-1,0,1,-1,0,0,0},{0,0,0,0,-1.5,0,0,0}};

#define ERROR_ANGLE 0.12
#define ERROR_DISTANCE 1.3
#define ERROR_ANGLE_ARM 0.07
#define MAX_WHEEL_VELOCITY 0.5
#define GRASPABLE_DISTANCE 22
#define LEFT_ARM 0
#define RIGHT_ARM 1

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
	
	bool moveArm(int left_or_right);
	void choose_task_arm(int task, int left_or_right);
	//function for the left arm
	void grasp_left_hand();
	void release_left_hand();
	bool goTo(Vector3d pos, double rangeToPoint);

	void get_Kinect_Data();
	void Kinect_Data_Off();
	void Record_Kinect_Data(std::string all_msg);
	void PrintPosture();

	void initialize();

private:
	// New defifinitions
	RobotObj *my;

	//define different joint of right and left arm
	double armJoint[2][8];
	double targetArmJoint[2][8];
	
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

	struct Joint_Coordinate {
		std::string Name;
		double qw;
		double qx;
		double qy;
		double qz;
	};

	struct Posture_Coordinates {
		double  time;
		std::vector <Joint_Coordinate> posture;
	};

	std::vector <Posture_Coordinates> Record_Postures;

	bool Kinect_data_flag;

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


bool RobotController::moveArm(int left_or_right)
{
	std::string jointName;
	bool finishFlag[8] = {false,false,false,false,false,false,false,false};
	if(left_or_right == LEFT_ARM)
	{
		jointName = "LARM_JOINT";
	}
	else if(left_or_right == RIGHT_ARM)
	{
		jointName = "RARM_JOINT";
	}
	for ( int i = 0; i < 8; i++ )
	{
		if (armJoint[left_or_right][i] != targetArmJoint[left_or_right][i] )
		{
			std::stringstream ss;
			ss << jointName << i;
			if (armJoint[left_or_right][i] < targetArmJoint[left_or_right][i] && targetArmJoint[left_or_right][i]-armJoint[left_or_right][i] > ERROR_ANGLE_ARM)
			{
				my->setJointVelocity(ss.str().c_str(), joint_veloc, 0.0);
				armJoint[left_or_right][i] = my->getJointAngle(ss.str().c_str());
			}
			else if (armJoint[left_or_right][i] > targetArmJoint[left_or_right][i] && armJoint[left_or_right][i]-targetArmJoint[left_or_right][i] > ERROR_ANGLE_ARM)
			{
				my->setJointVelocity(ss.str().c_str(), -joint_veloc, 0.0);
				armJoint[left_or_right][i] = my->getJointAngle(ss.str().c_str());
			}
			else
			{
				my->setJointVelocity(ss.str().c_str(), 0.0, 0.0);
				finishFlag[i] = true;
			}
		}
		else finishFlag[i] = true;
	}
	for( int i = 0; i < 8; i++)
	{
		if(!finishFlag[i]) return false;
	}
	return true;
}

void RobotController::choose_task_arm(int task, int left_or_right)
{
	if( left_or_right == LEFT_ARM)
	{
		for (int i=0; i<8; i++)
			targetArmJoint[left_or_right][i] = tab_joint_left[task][i];
	}
	else if( left_or_right == RIGHT_ARM)
	{
		for (int i=0; i<8; i++)
			targetArmJoint[left_or_right][i] = tab_joint_right[task][i];
	}
}

void RobotController::grasp_left_hand()
{
	Vector3d hand, object;
	SimObj *obj = getObj(m_pointedObject.c_str());

	obj->getPosition(object);
	my->getJointPosition(hand, "LARM_JOINT7");

	double distance = getDist3D(hand,object);

	if (distance < GRASPABLE_DISTANCE &&  m_grasp_left == false)
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

void RobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
}


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
	if ((angle-roll)>-ERROR_ANGLE && (angle-roll)<ERROR_ANGLE)
	{
		// error on distance
		if (dist-rangeToPoint < ERROR_DISTANCE && dist-rangeToPoint > -ERROR_DISTANCE)
		{
			stopRobotMove();
			return true;
		}
		else
		{
			speed = dist-rangeToPoint;
			if (dist-rangeToPoint < 5)
			{
				if ( dist-rangeToPoint > 0 )
					my->setWheelVelocity(1, 1);
				else
					my->setWheelVelocity(-1, -1);
			}
			else if ( dist-rangeToPoint > 0 )
				my->setWheelVelocity(Robot_speed , Robot_speed );
			else
				my->setWheelVelocity(-Robot_speed , -Robot_speed );
			return false;
		}
	}
	else
	{
		speed = fabs(angle-roll)*4;
		if (speed/4 > 0.3)
			if (angle < -M_PI_2 && roll > M_PI_2)
				my->setWheelVelocity(-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
			else if (angle > M_PI_2 && roll < -M_PI_2)
				my->setWheelVelocity(MAX_WHEEL_VELOCITY, -MAX_WHEEL_VELOCITY);
			else if (angle < roll)
				my->setWheelVelocity(MAX_WHEEL_VELOCITY, -MAX_WHEEL_VELOCITY);
			else
				my->setWheelVelocity(-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
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
			std::cout 
				<< "the name" <<  Record_Postures[i].posture[j].Name 
				<< " qw "  << Record_Postures[i].posture[j].qw 
				<< " qx "  << Record_Postures[i].posture[j].qx 
				<< " qy "  << Record_Postures[i].posture[j].qy 
				<<  " qz "  << Record_Postures[i].posture[j].qz 
				<< std::endl;
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

	// objects position
	m_BottleFront = Vector3d(40.0, 30, -40.0);
	m_MuccupFront = Vector3d(0.0, 30, -40.0);
	m_CanFront = Vector3d(-40.0, 30, -40.0);

	// trashBoxs position
	m_BurnableFront = Vector3d(-120.0, 30, -60);
	m_UnburnableFront = Vector3d(-120.0, 30, 70);
	m_RecycleFront = Vector3d(-60.0, 30, -100);

	m_relayPoint1 = Vector3d(100, 30, -70);
	m_relayPoint2 = Vector3d(0, 30, -70);
	m_relayFrontTable = Vector3d(0, 30,-20);

	m_relayFrontTable_reset = Vector3d(-80, 30,-50);
	m_relayFrontTrash = Vector3d(-80, 30, -80);

	cycle = 3;

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


void RobotController::initialize()
{
	m_state = 0;
	m_grasp_left = false;
	for( int i = 0; i<2; i++)
	{
		for( int j = 0; j<8; j++)
		{
			armJoint[i][j] = 0;
		}
	}
}


double RobotController::onAction(ActionEvent &evt)
{
	//std::cout << "m_state " <<  m_state << std::endl;
	//std::cout << "the size of Vector " <<  Record_Postures.size() << std::endl;
	switch(m_state)
	{
		case 10: 
		{
			Robot_speed  = Change_Robot_speed;
			choose_task_arm(5, LEFT_ARM);
			choose_task_arm(5, RIGHT_ARM);
			//   printf("got it in case!1 flag1 \n");
			if (goTo(m_relayPoint1, 0) == true && moveArm(LEFT_ARM) == true && moveArm(RIGHT_ARM) == true)
			{
				m_state = 20;
				//  printf("got it in case!1 \n");
			}
			break;
		}
		case 20:   
		{ 
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayPoint2, 0) == true) m_state = 30;
			break;
		}
		case 30: 
		{
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTable, 0) == true) m_state = 40;
			break;
		}
		case 40:   // Test if the cycle is finished or not
		{
			if (cycle > 0)
			{
				m_state = 41;
				broadcastMsg("Show_me");
				get_Kinect_Data();
				m_time = evt.time() + 5;
				break;
			}
			else 
			{
				m_state = 49;
				break;
			}
		}
		case 41: 
		{
			if(evt.time() > m_time) m_state = 42;
			break;
		}
		case 42:
		{
			Kinect_Data_Off(); // finished analysing data
			m_pointedObject = "petbottle";
			m_pointedtrash = "recycle";
			std::cout << "Task started Robot ........ "  << std::endl;
			//PrintPosture();
			m_state = 50;
		}
		case 49:
		{
			break;
		}
		case 50:     //Optional case  !!!
		{
			Robot_speed  = Change_Robot_speed;
			if (m_pointedObject=="petbottle")
			{
				if (goTo(m_BottleFront, 0) == true) m_state = 60;
			}
			else if (m_pointedObject=="mugcup")
			{
				if (goTo(m_MuccupFront, 0) == true) m_state = 60;
			}
			else if (m_pointedObject=="can")
			{
				if (goTo(m_CanFront, 0) == true)  m_state = 60;
			}
			break;
		}
		case 60:    //preparation of the arm for grasp
		{
			Robot_speed  = Change_Robot_speed;
			recognizeObjectPosition(m_Object_togo, m_pointedObject);
			if (goTo(m_Object_togo, 70) == true)
			{
				m_state = 70;
			}
			break;
		}
		case 70:    //preparation of the arm for grasp
		{
			choose_task_arm(1, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true) m_state = 80;
			break;
		}
		case 80:    //move to the object
		{
			Robot_speed  = 1;
			if (goTo(m_Object_togo, 38) == true) m_state = 90;
			break;
		}
		case 90:    //move arm to grasp the object
		{
			choose_task_arm(2, LEFT_ARM);
			grasp_left_hand();
			if (moveArm(LEFT_ARM) == true) m_state = 100;
			break;
		}
		case 100:
		{
			m_state = 110;
			break;
		}
		case 110:    //move arm to place in good position for moving
		{
			grasp_left_hand();
			choose_task_arm(3, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true)
				m_state = 120;
			break;
		}
		case 120:  
		{
			my->setWheelVelocity(-1.4,-1.4);
			choose_task_arm(5, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true)
			{
				Robot_speed  = Change_Robot_speed;
				m_state = 130;
				sleep(1);
			}
			break;
		}
		case 130:    //move to the object
		{
			// Robot_speed  = 1;
			if (goTo(m_relayFrontTrash, 0) == true)
			{
				m_state = 140;
			}
			break;
		}
		case 140: 
		{
			if (goTo(m_relayFrontTable_reset, 0) == true)
			{
				sleep(2);
				m_state = 150;
			}
			break;
		}
		case 990:    //move to the object
		{
			// Robot_speed  = 1;
			break;
		}
		case 150:    //Optional case  !!!
		{
			Robot_speed  = Change_Robot_speed;
			if (m_pointedtrash=="recycle")
			{
				if (goTo(m_RecycleFront, 0) == true)  m_state = 160;
			}
			else if (m_pointedtrash=="burnable")
			{
				if (goTo(m_BurnableFront, 0) == true) m_state = 160;
			}
			else if (m_pointedtrash=="unburnable")
			{
				if (goTo(m_UnburnableFront, 0) == true) m_state = 160;
			}
			break;
		}
		case 160:    //preparation of the arm for grasp
		{
			recognizeObjectPosition(m_Trash_togo, m_pointedtrash);
			if (goTo(m_Trash_togo, 50) == true)
			{
				m_state = 170;
			}
			break;
		}
		case 170:    //preparation of the arm for grasp
		{
			choose_task_arm(1, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true) m_state = 180;
			break;
		}
		case 180:   
		{
			Robot_speed  = 1;
			if (goTo(m_Trash_togo, 60) == true) m_state = 190;
			break;
		}
		case 190:    //move arm to grasp the object
		{
			choose_task_arm(3, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true)
			{
				m_state = 200;
				release_left_hand();
			}
			break;
		}
		case 200:  
		{
			my->setWheelVelocity(-2.5,-2.5);
			choose_task_arm(2, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true)
			{
				Robot_speed  = Change_Robot_speed;
				m_state = 210;
			}
			break;
		}
		case 210:    //move to the object
		{
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTrash, 0) == true) m_state = 220;
			break;
		}
		case 220:    //preparation of the arm for grasp
		{
			choose_task_arm(5, LEFT_ARM);
			if (moveArm(LEFT_ARM) == true) m_state = 230;
			break;
		}
		case 230:    //move to the object
		{
			Robot_speed  = Change_Robot_speed;
			if (goTo(m_relayFrontTable, 0) == true)
			{
				cycle = cycle-1;
				m_state = 40;
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
	std::string msg;
	msg= evt.getMsg();
	std::stringstream ss;
	ss << msg;
	std::string header;
	ss >> header;

	if (msg == "Task_start" && m_state == 0)
	{
		m_state = 10;      
		Kinect_data_flag = false;
		Record_Postures.clear();
	}

	if (msg == "Task_end") 
	{
		initialize();
	}

	if ( header == "KINECT_DATA_Sensor" && Kinect_data_flag == true)
	{
		Record_Kinect_Data(msg);
	}

}


void RobotController::Record_Kinect_Data(std::string all_msg)  //   
{
	Posture_Coordinates myPosture;
	std::stringstream ss;
	ss << all_msg;
	std::string header;
	ss >> header;

	while(ss){
		std::string jointInformation;
		ss >> jointInformation;
		std::replace(jointInformation.begin(),jointInformation.end(),':',' ');
		std::replace(jointInformation.begin(),jointInformation.end(),',',' ');
		std::stringstream jointss;
		jointss << jointInformation;
		Joint_Coordinate myJoint;
		jointss >> myJoint.Name;
		if(myJoint.Name == "END"){
			jointss >> myPosture.time;
			break;
		}
		else{
			jointss >> myJoint.qw >> myJoint.qx >> myJoint.qy >> myJoint.qz;
			myPosture.posture.push_back(myJoint);
		}
	}
	Record_Postures.push_back(myPosture);
}


void RobotController::onCollision(CollisionEvent &evt)
{
	if (m_grasp == false){
		//typedef CollisionEvent::WithC C; //TODO: What's this? should be removed.
		//触れたエンティティの名前を得ます
		const std::vector<std::string> & with = evt.getWith();
		// 衝突した自分のパーツを得ます
		const std::vector<std::string> & mparts = evt.getMyParts();

		//LOG_MSG(("<debug> with size : %d", with.size()));
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

