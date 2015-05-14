#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <sstream>
#include <iomanip>
//#include <sys/time.h>
#include <unistd.h>

#define MAX_TRIAL 20
#define MAX_SCORE 1000
#define PENALTY "-100"

char start_msg[]  = "Task_start";
char end_msg[]    = "Task_end";
char finish_msg[] = "Task_finished";
char giveup_msg[] = "Give_up";
char next_msg[]   = "Task_next";

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	//void onCollision(CollisionEvent &evt);

private:
	FILE* fp;

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
	int  taskNum;
	void initialize();

	//void startTask();
	void resetCondition();
	void breakTask();

	// 
	double startTime;
	double endTime;
	struct timeval t0, t1;

	double penalty;

	bool sendEndMsg();
	bool echoEndMsg;
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
	//gettimeofday(&t1, NULL);

	task = false;
	taskNum = 0;

	penalty = 0;

	if((fp = fopen("tasknum.txt", "r")) == NULL) {
		printf("File do not exist:trial.txt\n");
		exit(0);
	}
	else{
		fscanf(fp, "%d", &taskNum);
		LOG_MSG(("Set taskNum: %d",taskNum));
		fclose(fp);
	}

	startTime =  0.0;
	endTime   = 240.0; // [sec]

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
	if(taskNum >= MAX_TRIAL){
		return retValue;
	}


	// check whether Referee service is available or not
	bool available = checkService("RoboCupReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("RoboCupReferee");
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
	LOG_MSG(("%d.%d",sec,msec));
	t1 = t0;
	*/

	// reset task
	if(!task){
		int intervalTime = 3;
		resetCondition();
		//sendMsg("operator", "Reset_position");
		broadcastMsg("Reset_position");

		sleep(intervalTime);

		// broadcast start message
		broadcastMsg(start_msg);

		std::stringstream trial_ss;
		trial_ss << "RoboCupReferee/trial/";
		trial_ss << taskNum + 1 << "/";
		trial_ss << MAX_TRIAL;
		if (m_ref != NULL){
			m_ref->sendMsgToSrv(trial_ss.str().c_str());
		}
		else{
			LOG_MSG((trial_ss.str().c_str()));
		}

		if(m_ref != NULL){
			m_ref->sendMsgToSrv("RoboCupReferee/start");
		}
		LOG_MSG(("RoboCupReferee/start"));

		LOG_MSG(("task num: %d",taskNum+1));

		if((fp = fopen("tasknum.txt", "w")) == NULL) {
			printf("File do not exist:trial.txt\n");
			//exit(0);
		}
		else{
			//fscanf(fp, "%d", &taskNum);
			fprintf(fp,"%d",taskNum+1);
			LOG_MSG(("update tasknum.txt: %d",taskNum+1));
			fclose(fp);
		}


		startTime = evt.time() + intervalTime;
		task = true;
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

			penalty += atof(PENALTY);
			if (penalty + MAX_SCORE >= 0){
				std::string msg = "RoboCupReferee/Collision" "/" + (std::string)PENALTY;

				if (m_ref != NULL){
					m_ref->sendMsgToSrv(msg.c_str());
				}
				else{
					LOG_MSG((msg.c_str()));
				}
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
	
	std::stringstream time_ss;
	double elapsedTime = evt.time() - startTime;
	//if(evt.time() - startTime > endTime){
	if(elapsedTime > endTime){
		LOG_MSG(("Time_over"));
		broadcastMsg("Time_over");
		if(m_ref != NULL){
			m_ref->sendMsgToSrv("RoboCupReferee/end");
		}
		LOG_MSG(("RoboCupReferee/end"));
		task = false;
		sleep(3);
		breakTask();
		time_ss << "RoboCupReferee/time/00:00:00";
	}
	else{
		double remainedTime = endTime - elapsedTime;
		int min, sec, msec;
		sec = (int)remainedTime;
		min = sec / 60;
		sec %= 60;
		msec = (int)((remainedTime - sec) * 100);
		time_ss <<  "RoboCupReferee/time/";
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

	if(msg == giveup_msg){
		//sendMsg(operatorName, giveup_msg);
		//if(task){
			broadcastMsg(end_msg);
			/*echoEndMsg = false;
			while(!echoEndMsg){
				usleep(100000);
				broadcastMsg(end_msg);
			}*/
			if(m_ref != NULL){
				m_ref->sendMsgToSrv("RoboCupReferee/end");
				LOG_MSG(("RoboCupReferee/end"));
			}
			task = false;
			sleep(3);
			breakTask();
		//}
	}
	if(msg == finish_msg){
		broadcastMsg(end_msg);
		/*echoEndMsg = false;
		while(!echoEndMsg){
			usleep(100000);
			broadcastMsg(end_msg);
		}*/
		if(m_ref != NULL){
			m_ref->sendMsgToSrv("RoboCupReferee/end");
			LOG_MSG(("RoboCupReferee/end"));
		}
		task = false;
		sleep(3);
		breakTask();
	}
	if(msg == "Get_end_msg"){
		LOG_MSG(("get: Get_end_msg"));
		echoEndMsg = true;
	}
	if(msg == next_msg){
		broadcastMsg(end_msg);
		if(m_ref != NULL){
			m_ref->sendMsgToSrv("RoboCupReferee/end");
			LOG_MSG(("RoboCupReferee/end"));
		}
		task = false;
		sleep(3);
		breakTask();
	}
}

bool MyController::sendEndMsg(){
	broadcastMsg(end_msg);
}

/*void MyController::initialize()
{
}*/

/*void MyController::startTask()
{
	// broadcast start message
	broadcastMsg(start_msg);
	if(m_ref != NULL){
		m_ref->sendMsgToSrv("RoboCupReferee/start");
	}
	LOG_MSG(("RoboCupReferee/start"));
}*/
void MyController::breakTask()
{
	//task = false;
	taskNum++;
	penalty = 0;
	if(taskNum == MAX_TRIAL){
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
	//robot->setRotation(initialRotation);
	//robot->setPosition(initialPosition);
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
	robot->setJointAngle("RARM_JOINT1", 0.0);
	robot->setJointAngle("RARM_JOINT4", 0.0);

	/*SimObj* obj;
	obj = getObj(operatorName.c_str());
	obj->setRotation(initialRotation_operator);
	obj->setPosition(initialPosition_operator);

	obj = getObj(manName.c_str());
	obj->setRotation(initialRotation_man);
	obj->setPosition(initialPosition_man);*/
}

extern "C" Controller * createController() {
	return new MyController;
}
