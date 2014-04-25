#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sys/time.h>

char start_msg[]  = "start";
char end_msg[]    = "end";
char giveup_msg[] = "giveup";

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	//void onCollision(CollisionEvent &evt);

private:
	BaseService *m_ref;       // Referee service
	double retValue;          // Refresh rate of the modification
	bool   colState;          // Collision state
	bool   pcolState;         // Collision state
	std::string roboName;     // Robot's name
	std::string operatorName; // operator's name
	std::string manName;      // man's name
	std::string mdName;       // Moderator's name

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	Vector3d initialPosition;
	Rotation initialRotation;
	Vector3d initialPosition_operator;
	Rotation initialRotation_operator;
	Vector3d initialPosition_man;
	Rotation initialRotation_man;
	const static unsigned int jtnum=7;
	std::vector<std::string> jointName;
	double prv1JAng_r[jtnum];
	double crrJAng_r[jtnum];

	double rsLen;

	bool task;
	int  trialCount;
	void initialize();

	void startTask();
	void resetCondition();
	void breakTask();

	// 
	struct timeval t0, t1;
};

void MyController::onInit(InitEvent &evt)
{
	int i, size;

	retValue     = 0.08;
	roboName     = "robot_004";
	mdName       = "moderator_0";
	operatorName = "operator";
	manName      = "man_001";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	//
	gettimeofday(&t1, NULL);

	task = false;
	trialCount = 0;

	SimObj *obj;
	obj = getObj(roboName.c_str());
	obj->getPosition(initialPosition);
	obj->getRotation(initialRotation);

	obj = getObj(operatorName.c_str());
	obj->getPosition(initialPosition_operator);
	obj->getRotation(initialRotation_operator);

	obj = getObj(manName.c_str());
	obj->getPosition(initialPosition_man);
	obj->getRotation(initialRotation_man);
}

double MyController::onAction(ActionEvent &evt)
{
	if(trialCount >= 10){
		return retValue;
	}

	/*
	// processing time
	gettimeofday(&t0, NULL);
	int sec, msec;
	if (t0.tv_usec < t1.tv_usec) {
		sec = t0.tv_sec - t1.tv_sec - 1;
		msec= 1000000 + t0.tv_usec - t1.tv_usec;
	}
	else {
		sec = t0.tv_sec - t1.tv_sec;
		msec = t0.tv_usec - t1.tv_usec;
	}
	//LOG_MSG(("%d.%d",sec,msec));
	t1 = t0;
	*/

	// reset task
	if(!task){
		double deadTime = 3.0;
		resetCondition();

		usleep(deadTime * 1000000);

		// broadcast start message
		broadcastMsg(start_msg);
		LOG_MSG(("trial count: %d",trialCount));

		//startTime = evt.time() + deadTime;
		task = true;
	}

	// check whether Referee service is available or not
	bool available = checkService("FollowMeReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("FollowMeReferee");
	}

	// get information about the robot and renew it
	SimObj *r_my = getObj(roboName.c_str());

	// for body configuration
	r_my->getPosition(crrPos);
	r_my->getRotation(crrRot);

	Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
	rsLen = rsLenVec.length();


	std::map<std::string, CParts *> partsm = r_my->getPartsCollection();
	for (SimObj::PartsM::const_iterator i=partsm.begin(); i!=partsm.end(); i++) {
		CParts *parts = r_my->getParts(i->first.c_str());
		bool state = parts->getCollisionState();

		if(state){
			colState=true;       // collided with main body of robot
			if(rsLen == 0.0) {
				r_my->setRotation(prv1Rot);
			} else {
				r_my->setPosition(prv1Pos);
			}

			if(!colState){
				prv1Pos=crrPos;
				prv1Rot=crrRot;
			} else if(pcolState){
				for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
					pcolState = false;
				} else{
				colState = false;
			}

			std::string msg = "FollowMeReferee/Collision" "/-100";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
			break;
		}
	}

	if(!colState){
		prv1Pos=crrPos;
		prv1Rot=crrRot;
		//colState=false;     // reset collision condition
	}
	else if(pcolState){
		for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
			pcolState = false;
	}
	else{
		//Do nothing on "collided" condition
		//LOG_MSG((colPtName.c_str()));
		colState = false;
	}

	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();
	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	if(/*sender == roboName.c_str() &&*/ msg == end_msg){
		breakTask();
	}
	if(msg == giveup_msg){
		broadcastMsg(giveup_msg);
		sendMsg(operatorName, giveup_msg);
		breakTask();
	}
}

void MyController::initialize()
{
}

void MyController::startTask()
{
	// broadcast start message
	broadcastMsg(start_msg);
}
void MyController::breakTask()
{
	task = false;
	trialCount++;

	if(trialCount == 10){
		resetCondition();
		LOG_MSG(("End of all tasks"));
		broadcastMsg("End of all tasks");
	}
}

void MyController::resetCondition()
{
	RobotObj *robot;

	// reset robot condition
	robot = getRobotObj(roboName.c_str());
	robot->setWheelVelocity(0.0, 0.0);
	robot->setRotation(initialRotation);
	robot->setPosition(initialPosition);
	robot->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
	robot->setJointAngle("RARM_JOINT1", 0.0);
	robot->setJointAngle("RARM_JOINT4", 0.0);

	SimObj* obj;
	obj = getObj(operatorName.c_str());
	obj->setRotation(initialRotation_operator);
	obj->setPosition(initialPosition_operator);

	obj = getObj(manName.c_str());
	obj->setRotation(initialRotation_man);
	obj->setPosition(initialPosition_man);
}

extern "C" Controller * createController() {
	return new MyController;
}
