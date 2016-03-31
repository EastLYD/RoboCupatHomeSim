#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <iomanip>

#define JOINT_NUM 7
#define MAX_TRIAL 10
#define RANDOM_TRIAL 10
#define MAX_SCORE 1200
#define PENALTY "-100"
#define SCORE_OPERATION_1 "+400"
#define SCORE_OPERATION_2 "+400"
#define SCORE_OPERATION_3 "+400"

enum SideOfTable{
  UP,
  DOWN,
  LEFT,
  RIGHT
};

typedef struct target{
	std::string name;
	float x;
	float y;
	float z;
	float height; //bb on y-axis
	float length; //bb on x-axis
	float width;  //bb on z-axis
} Target;

typedef struct table{
	std::string name;
	float x;
	float y;
	float z;
	float length; //bb on x-axis
	float height; //bb on y-axis
	float width;  //bb on z-axis
	int reachable[4];
} Table;


class MyController : public Controller {
public:
	void   onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void   onCheckRoom();
	void   onCheckObject();
	void   onRecvMsg(RecvMsgEvent &evt);
	void   onCheckPositionfrontHuman();
	bool   checkAvailablePos(float posObj, Target obj, int sideOfTableIndex, std::vector<Target> vec);
	int    contains(std::vector<Table> vec, std::string key);
	int    contains(std::vector<Target> vec, std::string key);
	template<typename Type>
	int    contains(std::vector<Type> vec, Type key);
	int    getNextSideOfTable(int sideOfTableIndex, Table table);
	void   getNextTable(int* tableIndex, int* sideOfTableIndex, Table* table, std::vector<Table> vecTable);
	void   performChange(int* tableIndex, int* sideOfTableIndex, Table* table, std::vector<Table> vecTable);
	float  mapRange(float s, float a1, float a2, float b1, float b2);
	bool   checkRobotFinished();
	void   initRoomsObjects();
	void   resetRobotCondition();
	void   reposObjects();
	void   breakTask();

private:
	FILE* fp;             // File to keep the number of trials
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification
	bool   colState;      // Collision state
	bool  pcolState;      // Collision state
	std::string roboName; // Robot's name
	std::string mdName;   // Moderator's name
	std::string humanName;   // Human's name
	std::vector<std::string> jointName;
	double prv1JAng_r[JOINT_NUM];
	double crrJAng_r[JOINT_NUM];

	std::vector<std::string> m_entities;
	std::vector<std::string> m_entNames;
	int entNum;

	std::vector<std::string> m_rooms;
	int m_roomState;

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	std::string room_msg;
	std::string object_msg ;

	double take_time  ;
	bool init ;

	bool time_display ;

	Vector3d Obj_pos;
	Vector3d PrObj_pos;

	int trialCount;

	bool   isCleaningUp;
	double startTime;
	double endTime;

	bool Task_st;

	int penalty;

	std::string m_pointedObject;

	double rsLen;
	bool grasping;

	bool unable_collision;

	std::map< std::string, std::vector<Target> > targetsMap; //key is the name of the room
	std::map< std::string, std::vector<Table> > tablesMap; //key is the name of the room
	Vector3d robotInitialPos;
	Vector3d humanPos;
	float radius;
};


void MyController::onInit(InitEvent &evt)
{
	m_rooms.push_back("livingroom"); //0
	m_rooms.push_back("kitchen");    //1
	m_rooms.push_back("lobby");      //2
	m_rooms.push_back("bedroom");    //3
	m_roomState = 6;

	m_pointedObject = "";
	initRoomsObjects();

	int i, entityNum;
	Task_st = true;
	time_display = true;
	retValue = 0.01;
	colState = false;
	pcolState = false;
	roboName = "robot_000";
	mdName   = "moderator_0";
	humanName = "man_000";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	Obj_pos   = Vector3d(0,0,0);
	PrObj_pos = Vector3d(0,0,0);

	room_msg = "";
	object_msg = "" ;
	grasping = false;

	unable_collision = false;

	getObj(roboName.c_str())->getPosition(robotInitialPos);

	getObj(humanName.c_str())->getPosition(humanPos);
	radius = 200;

	getAllEntities(m_entities);

	entityNum = m_entities.size();

	for (i=0;i<entityNum;i++) {
		if ((m_entities[i] != mdName) &&
		   (m_entities[i] != roboName) &&
		   (m_entities[i] != "apple_0") &&
		   (m_entities[i] != "apple_1") &&
		   (m_entities[i] != "apple_2") &&
		   (m_entities[i] != "apple_3") &&
		   (m_entities[i] != "petbottle_0") &&
		   (m_entities[i] != "petbottle_1") &&
		   (m_entities[i] != "petbottle_2") &&
		   (m_entities[i] != "petbottle_3") &&
		   (m_entities[i] != "mugcup_0") &&
		   (m_entities[i] != "mugcup_1") &&
		   (m_entities[i] != "mugcup_2") &&
		   (m_entities[i] != "mugcup_3") &&
		   (m_entities[i] != "can_0") &&
		   (m_entities[i] != "can_1") &&
		   (m_entities[i] != "can_2") &&
		   (m_entities[i] != "can_3") ) {
			m_entNames.push_back(m_entities[i]);
		}

	}
	entNum = m_entNames.size();

	// Set the trial number from file "trialnum.txt"
	trialCount = 0;
	if ((fp = fopen("trialnum.txt", "r")) == NULL) {
		LOG_MSG(("trialnum.txt not found. Set trialCount = 0"));
	}
	else {
		fscanf(fp, "%d", &trialCount);
		LOG_MSG(("Set taskNum: %d", trialCount));
		fclose(fp);
		if (trialCount<0 || trialCount >= MAX_TRIAL) {
			LOG_MSG(("trialnum is wrong (%d). Set trialCount = 0", trialCount));
			trialCount = 0;
		}
	}

	penalty = 0;

	reposObjects();

	isCleaningUp = false;

	startTime =  0.0;
	endTime   = 600.0; // [sec]

	take_time = 0.0;
	init = false;
}


double MyController::onAction(ActionEvent &evt)
{
	// check whether Referee service is available or not
	bool available = checkService("RoboCupReferee");
	if (!available && m_ref != NULL) m_ref = NULL;
	else if (available && m_ref == NULL) {
		m_ref = connectToService("RoboCupReferee");
	}

	// get information about the robot and renew it
	//----------------------------------
	SimObj *r_my = getObj(roboName.c_str());

	// for body configuration
	r_my->getPosition(crrPos);
	r_my->getRotation(crrRot);

	Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
	rsLen = rsLenVec.length();

	if (Task_st == true && trialCount < MAX_TRIAL)
		{
			broadcastMsg("Task_start");
			Task_st = false;

			std::stringstream trial_ss;
			trial_ss << "RoboCupReferee/trial/";
			trial_ss << trialCount + 1 << "/";
			trial_ss << MAX_TRIAL;

			std::stringstream start_ss;
			start_ss << "RoboCupReferee/start/Trial ";
			start_ss << trialCount + 1 << " /";

			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(trial_ss.str().c_str());
				m_ref->sendMsgToSrv(start_ss.str().c_str());
			}
			else {
				LOG_MSG((trial_ss.str().c_str()));
				LOG_MSG((start_ss.str().c_str()));
			}
		}
	if (Task_st == true && trialCount >= MAX_TRIAL)
		{
			LOG_MSG(("Mission_complete"));
			broadcastMsg("Mission_complete");
			if (m_ref != NULL) {
				m_ref->sendMsgToSrv("RoboCupReferee/time/- END -");
			}
			time_display = false;
			Task_st = false;
		}

	for (int k=0;k<entNum;k++) {
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();

		if ( unable_collision == false)
			{
				if (state) {
					colState=true;       // collided with main body of robot

					if (rsLen == 0.0) {
						r_my->setRotation(prv1Rot);
					} else {
						r_my->setPosition(prv1Pos);
					}

					penalty += atof(PENALTY);
					if (penalty + MAX_SCORE >= 0){
						std::string msg = "RoboCupReferee/Collision with [" + m_entNames[k] + "]/" + (std::string)PENALTY;
						if (m_ref != NULL) {
							m_ref->sendMsgToSrv(msg.c_str());
						}
						else {
							LOG_MSG((msg.c_str()));
						}
					}

					break;
				}
			}
	}

	if (!colState) {
		prv1Pos=crrPos;
		prv1Rot=crrRot;
	} else if (pcolState) {
		for (int i=0;i<JOINT_NUM;i++) prv1JAng_r[i]=crrJAng_r[i];
		pcolState = false;
	} else {
		//Do nothing on "collided" condition
		colState = false;
	}

	std::stringstream time_ss;

	if (init == true)
		{
			take_time = evt.time();
			init = false;
		}

	double elapsedTime = evt.time() - startTime - take_time;

	if ( time_display == true)
		{
			if (elapsedTime > endTime) {
				LOG_MSG(("Time_over"));
				broadcastMsg("Time_over");
				breakTask();
			}
			else {
				double remainedTime = endTime - elapsedTime;
				int min, sec, msec;
				sec = (int)remainedTime;
				min = sec / 60;
				sec %= 60;
				msec = (int)((remainedTime - sec) * 100);
				time_ss.str("");
				time_ss <<  "RoboCupReferee/time/";
				time_ss << std::setw(2) << std::setfill('0') << min << ":";
				time_ss << std::setw(2) << std::setfill('0') << sec;
			}
			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(time_ss.str().c_str());
			}
			else {
				LOG_MSG((time_ss.str().c_str()));
			}
		}

	return retValue;
}



void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();

	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	// 送信者がゴミ認識サービスの場合
	char *all_msg = (char*)evt.getMsg();
	std::size_t found=0;
	std::size_t found2=0;
	std::size_t found3=0;
	std::string task;

	bool dest = false;

	if (sender == "man_000")
		{
			found = msg.find("Go to the ",0);
			if (found != std::string::npos) {
				task = msg.substr(found+10);
			}

			found2 = task.find(", grasp the ",0);
			if (found3 != std::string::npos) {
				room_msg = task.substr(0,found2);
			}

			found3 = task.find(" and come back here",found);
			if (found3 != std::string::npos) {
				object_msg = task.substr(found2+12,found3-found2-12);
			}

			if (room_msg == "living room" ) {

				m_roomState = 3;
				if (object_msg == "apple" )
					{
						m_pointedObject = "apple_0";
					}
				if (object_msg == "can" )
					{
						m_pointedObject = "can_0";
					}
				if (object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_0";
					}

				if (object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_0";
					}
			}


			// lobby message
			if (room_msg == "bed room" ) {

				m_roomState = 2;
				if (object_msg == "apple" )
					{
						m_pointedObject = "apple_3";
					}
				if (object_msg == "can" )
					{
						m_pointedObject = "can_3";
					}
				if (object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_3";
					}

				if (object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_3";
					}
			}

			if (room_msg == "lobby" ) {
				if (object_msg == "apple" )
					{
						m_pointedObject = "apple_1";
					}
				if (object_msg == "can" )
					{
						m_pointedObject = "can_1";
					}
				if (object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_1";
					}

				if (object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_1";
					}

				m_roomState = 1;
			}

			if (room_msg == "kitchen" ) {

				m_roomState = 0;

				if (object_msg == "apple" )
					{
						m_pointedObject = "apple_2";
					}
				if (object_msg == "can" )
					{
						m_pointedObject = "can_2";
					}
				if (object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_2";
					}

				if (object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_2";
					}
			}
		}




	if (((msg=="Room_reached") || (msg=="room_reached")) && sender == "robot_000")
		{
			onCheckRoom();
		}

	if (((msg=="Object_grasped") || (msg=="object_grasped")) && sender == "robot_000")
		{
			onCheckObject();
			grasping = true;
		}
	if (((msg=="Object_released") || (msg=="object_released")) && sender == "robot_000")
		{
			grasping = false;
		}
	if (msg == "Start grasping process")
		{
			SimObj *trash = getObj(m_pointedObject.c_str());
			// get trash's position
			trash->getPosition(Obj_pos);
			PrObj_pos = Obj_pos;
			unable_collision = true;
		}

    if (msg == "End grasping process")
		{
			unable_collision = false;
		}



		if (sender == "robot_000" && msg == "Task_finished") {
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		LOG_MSG(("before onCheckPositionfronHuman"));
		onCheckPositionfrontHuman();
		LOG_MSG(("after onCheckPositionfronHuman"));
		sleep(1);
		startTime = 0.0;
        Task_st = true;
		breakTask();
	}

	if ( (sender == "robot_000" || sender == "RoboCupReferee")  && msg == "Give_up")
	{
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		startTime = 0.0;
		breakTask();
	}
	
	if (msg == "init_time")
		{
			init = true;
		}
}

void MyController::onCheckPositionfrontHuman()
{
	Vector3d robot_pos;
	Vector3d human_pos;
	Vector3d dist;
	double distance;
	distance = 0;
	std::string final;
	final = "Front Human";
	robot_pos =  Vector3d(0, 0, 0);
	human_pos =  Vector3d(0, 0, 0);
	dist =  Vector3d(0, 0, 0);
	SimObj *avatar = getObj(humanName.c_str());
	SimObj *robot = getObj(roboName.c_str());

	// get trash's position
	robot->getPosition(robot_pos);
	avatar ->getPosition(human_pos);
	dist = robot_pos;
	dist -= human_pos;
	distance = dist.length();
	LOG_MSG(("The distance to human is %f", distance));
	if (distance <  radius )
		{
			std::string msg = "RoboCupReferee/Robot is in [" + final + "]/" + (std::string)SCORE_OPERATION_3;

			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else {
				LOG_MSG((msg.c_str()));
			}

		}
}


void MyController::onCheckObject()
{
	SimObj *trash = getObj(m_pointedObject.c_str());
	// get trash's position
	trash->getPosition(Obj_pos);

	if (Obj_pos.x()== PrObj_pos.x() && Obj_pos.y()== PrObj_pos.y() && Obj_pos.z()== PrObj_pos.z())
		{

		}
	else
		{
			std::string msg = "RoboCupReferee/Robot grasp the [" + m_pointedObject + "]/" + (std::string)SCORE_OPERATION_2;

			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else {
				LOG_MSG((msg.c_str()));
			}
		}

}

void MyController::onCheckRoom()
{
	int x, z;
	int num = 4;
	x =crrPos.x();
	z =crrPos.z();

	LOG_MSG(("Check robot position, select from 4 rooms"));
	if (x>-100&&x<500&&z>-425&&z<75)	// living room
		{
			num=0;
			if (m_roomState==3) {
				std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]/" + (std::string)SCORE_OPERATION_1;
				m_roomState=0;
				if (m_ref != NULL) {
					m_ref->sendMsgToSrv(msg.c_str());
				}
				else {
					LOG_MSG((msg.c_str()));
				}
			}
	}
	else if (x>100&&x<500&&z>75&&z<425) { // kitchen
		num=1;
		if (m_roomState==0) {
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]/" + (std::string)SCORE_OPERATION_1;
			m_roomState=1;
			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else {
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else if (x>-500&&x<-100&&z>-425&&z<-75) { // lobby
		num=2;
		if (m_roomState==1) {
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]/" + (std::string)SCORE_OPERATION_1;
			m_roomState=2;
			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else {
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else if (x>-500&&x<-100&&z>75&&z<425) { // bed room
		num=3;
		if (m_roomState==2) {
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]/" + (std::string)SCORE_OPERATION_1;
			m_roomState=3;
			if (m_ref != NULL) {
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else {
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else {
		LOG_MSG(("Robot is not staying at any room"));
	}

}


/*
  check if the position, posObj, where we want to put the object, obj, is
  available compared to the already placed objects contained in the vector, vec
*/
bool MyController::checkAvailablePos(float posObj, Target obj, int sideOfTableIndex, std::vector<Target> vec)
{
	bool available = true;

	if (vec.size() > 0) {
		if ( sideOfTableIndex == DOWN || sideOfTableIndex == UP ) {
			for (std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it) {
				float xInf = it->x - 0.5 * it->length - 20;
				float xSup = it->x + 0.5 * it->length + 20;
				float xInfObj = posObj - 0.5 * obj.length;
				float xSupObj = posObj + 0.5 * obj.length;

				available = (xInfObj < xInf && xSupObj < xInf ||
														 xInfObj > xSup && xSupObj > xSup);
			}
		}
		else {
			for (std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it) {
				float zInf = it->z - 0.5 * it->width - 20;
				float zSup = it->z + 0.5 * it->width + 20;
				float zInfObj = posObj - 0.5 * obj.width;
				float zSupObj = posObj + 0.5 * obj.width;

				available = (zInfObj < zInf && zSupObj < zInf ||
														 zInfObj > zSup && zSupObj > zSup);
			}
		}
	}

	return available;
}



int MyController::getNextSideOfTable(int sideOfTableIndex, Table table) {
	int j;
	for (j=(sideOfTableIndex+1) % 4; j != sideOfTableIndex && table.reachable[j] == -1; j = (j + 1) % 4);

	j--;

	if (j != sideOfTableIndex) {
		if (j == 0)
			j = 3;
		else
			j--;
	}

	else
		j = -1;

	return j;
}


void MyController::getNextTable(int* tableIndex, int* sideOfTableIndex, Table* table, std::vector<Table> vecTable) {
	*tableIndex = (*tableIndex + 1) % vecTable.size();
	*table = vecTable[*tableIndex];

	bool posAvailable = false;

	while (!posAvailable) {
		*sideOfTableIndex = rand() % 4;
		posAvailable = table->reachable[*sideOfTableIndex] != -1;
	}
}

void MyController::performChange(int* tableIndex, int* sideOfTableIndex, Table* table, std::vector<Table> vecTable) {
	int randChange = rand() % 2; // rand to determine if we look for another position or another table

	if (randChange == 1) {
		int j = getNextSideOfTable(*sideOfTableIndex, *table);

		if (j == -1) {
			getNextTable(tableIndex, sideOfTableIndex, table, vecTable);
		}

		else {
			*sideOfTableIndex = j;
		}
	}

	else {
		getNextTable(tableIndex, sideOfTableIndex, table, vecTable);
	}
}

void MyController::resetRobotCondition()
{
	//reset robot position
	RobotObj* robot = getRobotObj(roboName.c_str());

     if (grasping == true){
            CParts * lparts = robot->getParts("LARM_LINK7");
            lparts->releaseObj();
			CParts * rparts = robot->getParts("RARM_LINK7");
			rparts->releaseObj();
            grasping = false;

	}
	robot->setWheelVelocity(0.0,0.0);
	robot->setPosition(robotInitialPos);
	Rotation rot;
	rot.setQuaternion(1, 0, 0, 0);
    robot->setRotation(rot);
	robot->setJointVelocity("HEAD_JOINT0", 0.0, 0.0);
	robot->setJointVelocity("HEAD_JOINT1", 0.0, 0.0);

	robot->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
	robot->setJointVelocity("LARM_JOINT7", 0.0, 0.0);

	robot->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT5", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT6", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT7", 0.0, 0.0);

	robot->setJointVelocity("WAIST_JOINT0", 0.0, 0.0);
	robot->setJointVelocity("WAIST_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("WAIST_JOINT2", 0.0, 0.0);


	robot->setJointAngle("HEAD_JOINT0", 0.0);
	robot->setJointAngle("HEAD_JOINT1", 0.0);

	robot->setJointAngle("LARM_JOINT0", 0.0);
	robot->setJointAngle("LARM_JOINT1", 0.0);
	robot->setJointAngle("LARM_JOINT3", 0.0);
	robot->setJointAngle("LARM_JOINT4", 0.0);
	robot->setJointAngle("LARM_JOINT5", 0.0);
	robot->setJointAngle("LARM_JOINT6", 0.0);
	robot->setJointAngle("LARM_JOINT7", 0.0);

	robot->setJointAngle("RARM_JOINT0", 0.0);
	robot->setJointAngle("RARM_JOINT1", 0.0);
	robot->setJointAngle("RARM_JOINT3", 0.0);
	robot->setJointAngle("RARM_JOINT4", 0.0);
	robot->setJointAngle("RARM_JOINT5", 0.0);
	robot->setJointAngle("RARM_JOINT6", 0.0);
	robot->setJointAngle("RARM_JOINT7", 0.0);

	robot->setJointAngle("WAIST_JOINT0", 0.0);
	robot->setJointAngle("WAIST_JOINT1", 0.0);
	robot->setJointAngle("WAIST_JOINT2", 0.0);
}

void MyController::reposObjects()
{
	// Set random seed according to the number of trials. It provides the same random condition for all the competior
	srand (trialCount);

	for (std::map< std::string, std::vector<Target> >::iterator it = targetsMap.begin(); it != targetsMap.end(); ++it) {
		std::vector<Table> vecTable = tablesMap[it->first];
		std::vector<Target> placedObjects;
		int tablesNum = vecTable.size();
		float yObj;
		float xObj;
		float zObj;

		for (std::vector<Target>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
			int tableIndex = rand() % tablesNum;
			Table table = vecTable[tableIndex];
			bool posAvailable = false;
			int sideOfTableIndex;
			int triesNum;

			/*
			  we took a random table among the available ones and now we take a random
			  position on this table (UP, DOWN, LEFT, RIGHT) if it's available
			*/
			while (!posAvailable) {
				sideOfTableIndex = rand() % 4;
				posAvailable = table.reachable[sideOfTableIndex] != -1;
			}

			/*
			  This loop is made mainly to avoid placing different objects at the same
			  spot. If we try to place the object 10 times and it doesn't succeed in
			  finding an available spot then we try to find another position on the
			  current table or change the table directly
			*/
			do {
				triesNum = 0;

				yObj = table.y + 0.5 * table.height + 0.5 * it2->height + 1.75;

				if ( sideOfTableIndex == DOWN || sideOfTableIndex == UP ) {
					float xOffset = 10;
					float xInf = table.x - 0.5 * table.length + xOffset;
					float xSup = table.x + 0.5 * table.length - xOffset;
					float zOffset = it2->width;

					do {
						xObj = mapRange(rand(), 0,  RAND_MAX, xInf, xSup);
						triesNum++;
					} while (!checkAvailablePos(xObj, *it2, sideOfTableIndex, placedObjects) && triesNum < RANDOM_TRIAL);

					if (triesNum >= RANDOM_TRIAL) {
						performChange(&tableIndex, &sideOfTableIndex, &table, vecTable);
					}

					else {

						if (sideOfTableIndex == DOWN) {
							zObj = table.z + 0.5 * table.width - zOffset;
						}

						else {
							zObj = table.z - 0.5 * table.width + zOffset;
						}
					}
				}

				else {
					float zOffset = 10;
					float zSup = table.z - 0.5 * table.width + zOffset;
					float zInf = table.z + 0.5 * table.width - zOffset;
					float xOffset = it2->length;

					do {
						zObj = mapRange(rand(), 0,  RAND_MAX, zInf, zSup);
						triesNum++;
					} while (!checkAvailablePos(zObj, *it2, sideOfTableIndex, placedObjects) && triesNum < RANDOM_TRIAL);

					if (triesNum >= RANDOM_TRIAL) {
						performChange(&tableIndex, &sideOfTableIndex, &table, vecTable);
					}

					else {
						if (sideOfTableIndex == RIGHT) {
							xObj = table.x + 0.5 * table.length - xOffset;
						}

						else {
							xObj = table.x - 0.5 * table.length + xOffset;
						}
					}
				}
			} while (triesNum >= RANDOM_TRIAL);

			SimObj* target = getObj(it2->name.c_str());
			target->setPosition(Vector3d(xObj, yObj, zObj));
			Rotation rot;
			rot.setQuaternion(1, 0, 0, 0);
            target->setRotation(rot);
			it2->x = xObj;
			it2->y = yObj;
			it2->z = zObj;

			placedObjects.push_back(*it2);
		}

		placedObjects.clear();
	}
         
}


void MyController::breakTask()
{
	LOG_MSG(("start of breakTask"));
	isCleaningUp = false;
	//takeAwayObjects();

	std::stringstream end_ss;
			end_ss << "RoboCupReferee/end/Trial ";
			end_ss << trialCount + 1 << " /";
	if (m_ref != NULL) {
		m_ref->sendMsgToSrv(end_ss.str().c_str());
	}
	else {
		LOG_MSG((end_ss.str().c_str()));
	}
		
	trialCount++;
	penalty = 0;

	if (trialCount < MAX_TRIAL)
		{
			broadcastMsg("Task_end");
			resetRobotCondition();
			reposObjects();
		}
	if (trialCount >= MAX_TRIAL) {
		LOG_MSG(("Mission_complete"));
		broadcastMsg("Mission_complete");
		if (m_ref != NULL) {
			std::string msg = "RoboCupReferee/time/- END -";
            m_ref->sendMsgToSrv(msg.c_str());
		}
		Task_st = false;
		time_display = false;
	}
	else {
		Task_st = true;
	}
	LOG_MSG(("end of breakTask"));
}


void MyController::initRoomsObjects()
{
	std::vector<Table> tableVector;
	Table table;
	std::vector<Target> targetVector;
	Target target;
    Vector3d posf;
    SimObj* entity ;

	float margin = 10.0;    

	table.name = "Buffet1";
	table.length = 32.1;
	table.width  = 54.2;
	table.height = 59.5;
	table.reachable[UP]    = 1;
	table.reachable[DOWN]  = 1;
	table.reachable[RIGHT] = 1;
	table.reachable[LEFT]  = -1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x() + margin;
	table.y = posf.y();
	table.z = posf.z();

	tableVector.push_back(table);
    
	table.name = "Dinner table1";
	table.length = 135;
	table.width  = 75.9;
	table.height = 59.2;
	table.reachable[UP]    = 1;
	table.reachable[DOWN]  = 1;
	table.reachable[RIGHT] = 1;
	table.reachable[LEFT]  = 1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x();
	table.y = posf.y();
	table.z = posf.z();

	tableVector.push_back(table);
     
	table.name = "Couch_table1";
	table.length = 128;
	table.width  = 65.8;
	table.height = 59.8;
	table.reachable[UP]    = 1;
	table.reachable[DOWN]  = 1;
	table.reachable[RIGHT] = 1;
	table.reachable[LEFT]  = 1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x();
	table.y = posf.y();
	table.z = posf.z();

	tableVector.push_back(table);

	target.name = "apple_0";
	target.length = 6.51;
	target.width  = 6.86;
	target.height = 7.75;

	targetVector.push_back(target);

	target.name = "petbottle_0";
	target.length = 7.11;
	target.width  = 7.11;
	target.height = 22.3;

	targetVector.push_back(target);

	target.name = "can_0";
	target.length = 5.14;
	target.width  = 5.14;
	target.height = 9.07;

	targetVector.push_back(target);

	target.name = "mugcup_0";
	target.length = 11.7;
	target.width  = 11.7;
	target.height = 7.98;

	targetVector.push_back(target);

	tablesMap ["livingroom"] = tableVector;
	targetsMap["livingroom"] = targetVector;

	tableVector.clear();
	targetVector.clear();

	table.name = "Buffet2";
	table.length = 32.1;
	table.width  = 54.2;
	table.height = 59.5;
	table.reachable[UP]    =  1;
	table.reachable[DOWN]  =  1;
	table.reachable[RIGHT] = -1;
	table.reachable[LEFT]  =  1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x() - margin;
	table.y = posf.y();
	table.z = posf.z();
	tableVector.push_back(table);

	table.name = "Side board1";
	table.length = 165;
	table.width  = 50.8;
	table.height = 59.7;
	table.reachable[UP]    =  1;
	table.reachable[DOWN]  = -1;
	table.reachable[RIGHT] = -1;
	table.reachable[LEFT]  =  1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x() - margin;
	table.y = posf.y();
	table.z = posf.z();
	tableVector.push_back(table);

	target.name = "apple_1";
	target.length = 6.51;
	target.width  = 6.86;
	target.height = 7.75;

	targetVector.push_back(target);

	target.name = "petbottle_1";
	target.length = 7.11;
	target.width  = 7.11;
	target.height = 22.3;

	targetVector.push_back(target);

	target.name = "can_1";
	target.length = 5.14;
	target.width  = 5.14;
	target.height = 9.07;

	targetVector.push_back(target);

	target.name = "mugcup_1";
	target.length = 11.7;
	target.width  = 11.7;
	target.height = 7.98;

	targetVector.push_back(target);

	tablesMap ["lobby"] = tableVector;
	targetsMap["lobby"] = targetVector;

	tableVector.clear();
	targetVector.clear();

	table.name = "Kitchen Table1";
	table.length = 140;
	table.width  = 85;
	table.height = 59.4;
	table.reachable[UP]    = 1;
	table.reachable[DOWN]  = 1;
	table.reachable[RIGHT] = 1;
	table.reachable[LEFT]  = 1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x();
	table.y = posf.y();
	table.z = posf.z();

	tableVector.push_back(table);

	target.name = "apple_2";
	target.length = 6.51;
	target.width  = 6.86;
	target.height = 7.75;

	targetVector.push_back(target);

	target.name = "petbottle_2";
	target.length = 7.11;
	target.width  = 7.11;
	target.height = 22.3;

	targetVector.push_back(target);

	target.name = "can_2";
	target.length = 5.14;
	target.width  = 5.14;
	target.height = 9.07;

	targetVector.push_back(target);

	target.name = "mugcup_2";
	target.length = 11.7;
	target.width  = 11.7;
	target.height = 7.98;

	targetVector.push_back(target);

	tablesMap ["kitchen"] = tableVector;
	targetsMap["kitchen"] = targetVector;

	tableVector.clear();
	targetVector.clear();

	table.name = "Side table1";
	table.length = 32.1;
	table.width  = 34.2;
	table.height = 59.5;
	table.reachable[UP]    =  1;
	table.reachable[DOWN]  = -1;
	table.reachable[RIGHT] = -1;
	table.reachable[LEFT]  = -1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);

    table.x = posf.x() - margin;
	table.y = posf.y();
	table.z = posf.z() - margin;
	tableVector.push_back(table);

	table.name = "Side board2";
	table.length = 167;
	table.width  = 24.9;
	table.height = 58.8;
	table.reachable[UP]    =  1;
	table.reachable[DOWN]  = -1;
	table.reachable[RIGHT] = -1;
	table.reachable[LEFT]  = -1;
	entity = getObj(table.name.c_str());
	entity->getPosition(posf);
	table.x = posf.x() + margin;
	table.y = posf.y();
	table.z = posf.z() - margin;
	tableVector.push_back(table);

	target.name = "apple_3";
	target.length = 6.51;
	target.width  = 6.86;
	target.height = 7.75;

	targetVector.push_back(target);

	target.name = "petbottle_3";
	target.length = 7.11;
	target.width  = 7.11;
	target.height = 22.3;

	targetVector.push_back(target);

	target.name = "can_3";
	target.length = 5.14;
	target.width  = 5.14;
	target.height = 9.07;

	targetVector.push_back(target);

	target.name = "mugcup_3";
	target.length = 11.7;
	target.width  = 11.7;
	target.height = 7.98;

	targetVector.push_back(target);

	tablesMap ["bedroom"] = tableVector;
	targetsMap["bedroom"] = targetVector;
}


bool MyController::checkRobotFinished()
{
	Vector3d posR;
	getObj(roboName.c_str())->getPosition(posR);

	return pow( ( posR.x() - humanPos.x() ), 2) + pow( ( posR.z() - humanPos.z() ), 2) < (radius * radius);
}


template<typename Type>
int MyController::contains(std::vector<Type> vec, Type key)
{
	bool found = false;
	int i;
	for (i = 0 ; i < vec.size() && !found; i++) {
		found = vec[i] == key;
	}

	if (!found)
		i = -1;
	else
		i--;

	return i;
}


int MyController::contains(std::vector<Target> vec, std::string key)
{
	bool found = false;
	int i;
	for (i = 0 ; i < vec.size() && !found; i++) {
		found = vec[i].name == key;
	}

	if (!found)
		i = -1;
	else
		i--;

	return i;
}


int MyController::contains(std::vector<Table> vec, std::string key)
{
	bool found = false;
	int i;
	for (i = 0 ; i < vec.size() && !found; i++) {
		found = vec[i].name == key;
	}

	if (!found)
		i = -1;
	else
		i--;

	return i;
}


float MyController::mapRange(float s, float a1, float a2, float b1, float b2)
{
	return b1 + ( (s-a1) * (b2-b1) ) / (a2 - a1);
}

extern "C" Controller * createController() {
	return new MyController;
}
