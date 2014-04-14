#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>

// Convert degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

#define WALKING_PERSON  1.0 //人が歩き始める場所
#define CHECK_POINT1    2.0 //1st Sectionの点数が入る場所
#define ELEVATOR        3.0 //エレベータ内の待機場所
#define ELEVATOR_CLEAR  4.0 //2nd Sectionの点数が入る場所
#define CROWD           5.0 //人ごみを抜けた後の待機場所
#define CROWD_END       6.0 //3rd Sectionの点数が入る場所
#define END            99.0 //終了地点

struct coordinate{
	double x[255];
	double y[255];
	double z[255];
	double flag[255];
};

char robotName[] = "robot_004";

using namespace std;

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

private:
	SimObj *my;
	std::vector<std::string> m_entities;

	coordinate node; 

	bool first;
	bool start;
	bool elevator;
	bool crowd;
	bool end;
	bool stop;
	bool doorClose;
	bool leaveElevator;
	//bool flg3;
	//bool ok;
	bool elevator_end;

	bool sentMsg_Man;
	bool sendMsg_CheckPoint1;
	bool sentMsg_Elevator;
	bool sentMsg_CheckPoint2;
	bool sentMsg_Crowd;
	bool sentMsg_Finishline;

	double dx,dy,dz;
	int i; 

	bool follow;
	bool walking;

	FILE* fp;
	float stepWidth;
	int sleeptime;
	const static int SIZE = 30;
	int motionNum;
	float HEIGHT[SIZE];
	float LARM_JOINT1[SIZE]; // left shoulder
	float LARM_JOINT3[SIZE]; // left elbow
	float RARM_JOINT1[SIZE]; // right shoulder
	float RARM_JOINT3[SIZE]; // right elbow
	float LLEG_JOINT2[SIZE]; // left groin(leg)
	float LLEG_JOINT4[SIZE]; // left knee
	float LLEG_JOINT6[SIZE]; // left ankle
	float RLEG_JOINT2[SIZE]; // right groin(leg)
	float RLEG_JOINT4[SIZE]; // right knee
	float RLEG_JOINT6[SIZE]; // right ankle

	int count;
	int step;

	double interval;

	void initCondition();
};

void MyController::onInit(InitEvent &evt) 
{
	initCondition();

	my = getObj(myname());

	// Put arms down
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

	stepWidth = 85;

	if((fp = fopen("motion.txt", "r")) == NULL) {
		printf("File do not exist.\n");
		exit(0);
	}
	else{
		fscanf(fp, "%d", &motionNum);
		fscanf(fp, "%d", &sleeptime);
		for(int i=0; i<motionNum; i++){
			fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f",
					   &HEIGHT[i],
					   &LARM_JOINT1[i],
					   &LARM_JOINT3[i],
					   &RARM_JOINT1[i],
					   &RARM_JOINT3[i],
					   &LLEG_JOINT2[i],
					   &LLEG_JOINT4[i],
					   &LLEG_JOINT6[i],
					   &RLEG_JOINT2[i],
					   &RLEG_JOINT4[i],
					   &RLEG_JOINT6[i]);
		}
	}

	interval = sleeptime / 1000000;
}

void MyController::initCondition()
{
	start = false;
	elevator = false;
	crowd = false;
	end = false;
	stop = true;
	doorClose = false;
	//flg3 = false;
	first = false;
	leaveElevator=false;
	elevator_end = false;
	i=0;

	follow = true;
	walking = false;

	sentMsg_Man = false;
	sendMsg_CheckPoint1 = false;
	sentMsg_Elevator = false;
	sentMsg_CheckPoint2 = false;
	sentMsg_Crowd = false;
	sentMsg_Finishline = false;

	count = 0;
	step = 0;

	double x=0;
	double y=0;
	double z=0;
	double flag=0; //Check point

	dx=0;
	dy=0;
	dz=0;

	if((fp = fopen("node.txt", "r")) == NULL) {
		LOG_MSG(("File do not exist."));
		exit(0);
	}
	while(fscanf(fp, "%lf,%lf,%lf,%lf", &x,&y,&z,&flag) != EOF) {
		node.x[i]=x;
		node.y[i]=y;
		node.z[i]=z;
		node.flag[i]=flag;
		i++;
	}
	fclose(fp);
	i=0;

}

double MyController::onAction(ActionEvent &evt)
{
	if(!end && start){
		
		Vector3d pos;
		double angle;

		my->getPosition(pos);

		// 歩いていないとき(node上にいるとき)
		if(!walking){
			dx=(node.x[i]-pos.x());
			dz=(node.z[i]-pos.z());
			angle = atan2(dx,dz);
			
			my->setAxisAndAngle(0,1.0, 0, angle);
			
			double r = sqrt(pow(dx,2)+pow(dz,2));
			
			step = 2 * (int)r / stepWidth;
			
			dx /= step*motionNum;
			dz /= step*motionNum;
			
			walking = true;

			// 
			/*if(!leaveElevator && !follow && node.flag[i-1]>0){
				stop = true;
				LOG_MSG(("wait"));
			}*/
		}

		// 歩く
		if(!stop){
			double addx = 0.0;
			double addz = 0.0;
			
			if(count%2){
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
					my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[j]));
					my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			else{
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
					my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[j]));
					my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			count++;

			// 目標点に到着
			if(step==count){
				// チェックポイントなら停止
				if(node.flag[i]){
					stop = true;
				}
				count = 0;
				step = 0;
				walking = false;
				i++;
			}
		}

		//LOG_MSG(("%lf",checkPoint));

		/*if(follow && !checkPoint){
			stop = false;
		}*/

		double checkPoint = node.flag[i-1];

		if(follow && !elevator){
			// 人が歩き始める
			if(checkPoint == WALKING_PERSON && !sentMsg_Man){
				sendMsg("man_001", "point1");
				LOG_MSG(("point1"));
				sentMsg_Man = true;
			}
			// チェックポイント1
			else if(checkPoint == CHECK_POINT1 && !sendMsg_CheckPoint1){
				sendMsg("score","check1");
				LOG_MSG(("check point 1"));
				sendMsg_CheckPoint1 = true;
			}
			// 人ごみを通過
			else if(checkPoint == CROWD_END && !sentMsg_Crowd){
				std::string msg5 = "crowd";
				sendMsg("score", msg5);
				LOG_MSG(("crowd"));
				sentMsg_Crowd = true;
			}

			// チェックポイント以外
			//else if(!checkPoint){
				stop = false;
			//}
		}

		// エレベータ奥
		if(checkPoint == ELEVATOR && !elevator){
			if(!leaveElevator){
				stop = true;
				elevator = true;
			}
		}

		// エレベータ＆ドアを閉めてＯＫ(ロボットが入り終わった)
		if(elevator && doorClose && !sentMsg_Elevator){
			sendMsg("wall_008", "elevator_close");
			LOG_MSG(("elevator close"));
			sentMsg_Elevator = true;
		}

		// ロボットが退出したらエレベータから出る
		if(leaveElevator && !elevator_end){
			elevator = false;
			stop = false;
			elevator_end = true;
		}

		// エレベータクリア
		if(checkPoint==ELEVATOR_CLEAR && !sentMsg_CheckPoint2){
			leaveElevator = false;
			sendMsg("score","elevator");
			LOG_MSG(("elevator_clear"));
			sentMsg_CheckPoint2 = true;
		}

		// 終了点に着いた
		if(checkPoint==END && follow){
			end = true;
			//broadcastMsg("end");
		}
	}

	/*// タスクの終了
	if(node.flag[i-1] == END && follow){
		broadcastMsg("end");
	}*/

	return interval;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();

	// タスク開始
	if(msg == "Task_start"){
		initCondition();
		start = true;
		stop = false;
	}
	// ロボットがエレベータに入り終わった
	else if(msg == "entered"){
		LOG_MSG(("entered"));
		doorClose = true;
	}
	// エレベータのドアが開いた
	else if(msg == "elevator_open"){
		//sendMsg(robotName,"leave the elevator");
		broadcastMsg("leave the elevator");
	}
	// ロボットがエレベータから退出した
	else if(msg == "ok"){
		leaveElevator = true;
		sendMsg("score","elevator_clear");
	}

	// 追跡できているか
	if(msg == "NotFollowing"){
		follow = false;
	}
	else if(msg == "Following"){
		follow = true;
	}
}

void MyController::onCollision(CollisionEvent &evt)
{

}

extern "C" Controller * createController() {
	return new MyController;  
}


