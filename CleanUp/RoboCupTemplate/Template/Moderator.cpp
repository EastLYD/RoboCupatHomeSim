#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sstream>
#include <iomanip>

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	//void onCollision(CollisionEvent &evt);

private:
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification
	bool   colState;      // Collision state
	bool   pcolState;     // Collision state
	std::string roboName; // Robot's name
	std::string mdName;   // Moderator's name

	const static unsigned int jtnum=7;
	std::vector<std::string> jointName;
	double prv1JAng_r[jtnum];
	double crrJAng_r[jtnum];

	std::vector<std::string> m_entities;
	std::vector<std::string> m_entNames;
	int entNum;

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	double rsLen;

	int trialCount;

	bool   isCleaningUp;
	double startTime;
	double endTime;

	std::vector<std::string> targetEntities;
	std::vector<double>      targetEntitiesHeight;

	void startTask();
	void resetEntitiesPosition();
	void resetRobotCondition();
	void breakTask();

	double getCurrentTime();
};

void MyController::onInit(InitEvent &evt)
{
	int i, cnt;

	retValue  = 0.08;
	colState  = false;
	pcolState = false;
	roboName  = "robot_000";
	mdName    = "moderator_0";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	getAllEntities(m_entities);

	// select objects to be observed
	cnt = m_entities.size();
	for(i=0;i<cnt;i++){
		if((m_entities[i] != mdName) &&
		   (m_entities[i] != roboName) &&
		   (m_entities[i] != "trashbox_0") &&
		   (m_entities[i] != "trashbox_1") &&
		   (m_entities[i] != "trashbox_2") &&
		   (m_entities[i] != "petbottle_0") &&
		   (m_entities[i] != "petbottle_1") &&
		   (m_entities[i] != "petbottle_2") &&
		   (m_entities[i] != "petbottle_3") &&
		   (m_entities[i] != "petbottle_4") &&
		   (m_entities[i] != "banana") &&
		   (m_entities[i] != "chigarette") &&
		   (m_entities[i] != "chocolate") &&
		   (m_entities[i] != "mayonaise_0") &&
		   (m_entities[i] != "mayonaise_1") &&
		   (m_entities[i] != "mugcup") &&
		   (m_entities[i] != "can_0") &&
		   (m_entities[i] != "can_1") &&
		   (m_entities[i] != "can_2") &&
		   (m_entities[i] != "can_3") &&
		   (m_entities[i] != "apple") &&
		   (m_entities[i] != "clock") &&
		   (m_entities[i] != "kettle") ){
			m_entNames.push_back(m_entities[i]);
		}
	}
	entNum = m_entNames.size();

	trialCount = 0;

	isCleaningUp = false;

	startTime =  0.0;
	endTime   = 70.0; // [sec]

	// push target entitiy information
	targetEntities.push_back("can_0");
	targetEntitiesHeight.push_back(5.6);
	targetEntities.push_back("petbottle_1");
	targetEntitiesHeight.push_back(11.2);
	targetEntities.push_back("apple");
	targetEntitiesHeight.push_back(3.925);

	//srand(time(NULL));
	srand(2);
}

double MyController::onAction(ActionEvent &evt)
{
	if(trialCount >= 10){
		return retValue;
	}

	// reset clean-up task
	if(!isCleaningUp){
		double deadTime = 3.0;
		resetRobotCondition();
		resetEntitiesPosition();

		usleep(deadTime * 1000000);

		// broadcast start message
		broadcastMsg("Task_start");

		startTime = evt.time() + deadTime;
		isCleaningUp = true;
	}

	// check whether Referee service is available or not
	bool available = checkService("CleanUpReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("CleanUpReferee");
	}

	// get information about the robot and renew it
	//----------------------------------
	SimObj *r_my = getObj(roboName.c_str());
	// for body configuration
	r_my->getPosition(crrPos);
	r_my->getRotation(crrRot);

	Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
	rsLen = rsLenVec.length();

	// for arm configuration
	//for(int j=0;j<jtnum;j++) crrJAng_r[j] = r_my->getJointAngle(jointName[j].c_str());

	// 
	for(int k=0;k<entNum;k++){
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();

		if(state){
			colState=true;       // collided with main body of robot
			if(rsLen == 0.0) {
				//pcolState=true;
				r_my->setRotation(prv1Rot);
			} else {
				//pcolState=false;
				r_my->setPosition(prv1Pos);
			}

			std::string msg = "CleanUpReferee/Collision with [" + m_entNames[k] + "]" "/-100";
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
			break;
		}
	}

	//if(!colState && !pcolState){
	if(!colState){
		prv1Pos=crrPos;
		prv1Rot=crrRot;
		//colState=false;     // reset collision condition
	} else if(pcolState){
		for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
			pcolState = false;
		} else{
		//Do nothing on "collided" condition
		//LOG_MSG((colPtName.c_str()));
		colState = false;
	}

	std::stringstream time_ss;
	double elapsedTime = evt.time() - startTime;
	//if(evt.time() - startTime > endTime){
	if(elapsedTime > endTime){
		LOG_MSG(("Time_over"));
		broadcastMsg("Time_over");
		breakTask();
		time_ss << "CleanUpReferee/time/00:00:00";
	}
	else{
		double remainedTime = endTime - elapsedTime;
		int min, sec, msec;
		sec = (int)remainedTime;
		min = sec / 60;
		sec %= 60;
		msec = (int)((remainedTime - sec) * 100);
		time_ss <<  "CleanUpReferee/time/";
		time_ss << std::setw(2) << std::setfill('0') << min << ":";
		time_ss << std::setw(2) << std::setfill('0') << sec;// << ":";
		//time_ss << std::setw(2) << std::setfill('0') << msec;
	}
	if(m_ref != NULL){
		m_ref->sendMsgToSrv(time_ss.str().c_str());
	}
	else{
		LOG_MSG((time_ss.str().c_str()));
	}

	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();
	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	if(sender == "robot_000" && msg == "End_of_task"){
		breakTask();
	}
}


/*
void MyController::onCollision(CollisionEvent &evt) {
}
*/

void MyController::resetRobotCondition()
{
	RobotObj *robot;

	// reset robot condition
	robot = getRobotObj(roboName.c_str());
	robot->setWheelVelocity(0.0, 0.0);
	robot->setRotation(Rotation(1,0,0,0));
	robot->setPosition(100.0, 30.0, 10.0);
	robot->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
	robot->setJointAngle("RARM_JOINT1", 0.0);
	robot->setJointAngle("RARM_JOINT4", 0.0);
}

void MyController::resetEntitiesPosition()
{
	SimObj   *obj;
	double table1_x_min = 240.0; // right edge of a glass table
	double table1_x_max = 330.0; // left edge of a glass table
	double table1_y     =  62.4; // height of table
	double table1_z     =  -5.0;
	double pos_x, pos_y, pos_z;

	// reset target entity position
	unsigned int numberEntities = targetEntities.size();
	for(unsigned int i=0; i<numberEntities; ++i){
		obj = getObj(targetEntities[i].c_str());
		obj->setPosition(10000000, 10000000, 10000000);
	}

	usleep(100000);

	// select a target entity, set its position at random
	int index = rand() % numberEntities;
	obj = getObj(targetEntities[index].c_str());
	pos_x = table1_x_min + rand() % (int)(table1_x_max - table1_x_min);
	pos_y = table1_y + targetEntitiesHeight[index];
	pos_z = table1_z;
	obj->setPosition(pos_x, pos_y, pos_z);

	LOG_MSG(("Entities were relocated."));

}

void MyController::startTask()
{
	// broadcast start message
	broadcastMsg("Task_start");
}

void MyController::breakTask()
{
	isCleaningUp = false;
	trialCount++;

	if(trialCount == 10){
		resetRobotCondition();
		LOG_MSG(("End of all tasks"));
		broadcastMsg("End of all tasks");
	}
}

extern "C" Controller * createController() {
	return new MyController;
}
