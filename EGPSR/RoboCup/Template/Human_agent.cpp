#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include <map>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define MAX_CHARS_PER_LINE 512

#define NUMBER_OF_REPETITION   10

double tab_joint_left[3][8] = {{0,0,-1.57,0,0,0,0,0},{-0.68,0,0,0,0,0,0,0},{-1.3,0,0,0,0,0,0,0}};
double tab_joint_right[3][8] = {{0,0,1.57,0,0,0,0,0},{0.68,0,0,0,0,0,0},{1.3,0,0,0,0,0,0,0}};
double tab_joint_head[5] = {0, -0.68, -1, 0.68, 1};
double tab_joint_body[2] = {0,1.2};


class UserController : public Controller
{
public:
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	void parseFile(const std::string fileNam_my);
	void choice_movement(std::string room);
	void moveLeftArm(int task);
	void moveRightArm(int task);
	void moveHead(int task);
	void moveBody(int task);

private:

	//移動速度
	ViewService *m_view;
	std::string msg ;

	//初期位置
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;

	//データ数（関節数）最大値
	int m_maxsize;
	std::vector<std::string> rooms;
	std::vector<std::string> objects;
	std::string msg_toSend;
	// ロボットの名前
	std::string robotName;
	int cycle;
	int m_state;
	double m_message;
	std::string message;
	std::string brodcast_msg;
	bool Mission_complete;
};


void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";
    
	//m_kinect = connectToService("SIGKINECT");
	//m_hmd = connectToService("SIGHMD");
	parseFile("command.txt");
    Mission_complete = false;
	//printf("Reslutat %s", rooms[2]);
	rooms.clear();
	objects.clear();
	m_message = 10 ;

	cycle = 0;
    m_state =20;
	srand(time(NULL));

	//初期位置の設定
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	m_maxsize = 15;
}


//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{
	switch(m_state){
	case 0: {
		m_state = 20;
		break;
    }
    case 1: {
		break;
    }
	}
	/*
	  switch(m_message){
	  //initialization
	  case 1: { // wait next message
	  msg_toSend = rooms[cycle]+"."+object[cycle];

	  sendMsg("moderator_0",msg_toSend.c_str());
	  sendMsg("robot_000",msg_toSend.c_str());
	  m_message=2;
	  break;
	  }*/
	return 1.5;
}


void UserController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();
	//自分自身の取得
	SimObj *my = getObj(myname());

	if(msg == "Task_start"   && sender == "moderator_0"  && cycle < NUMBER_OF_REPETITION && Mission_complete == false) {

		parseFile("command.txt");

		message = ":"+ rooms[cycle] +";"+ objects[cycle]+".";
		brodcast_msg = "Go to the "+ rooms[cycle] +", grasp the "+ objects[cycle]+" and come back here";
		choice_movement(rooms[cycle]);
		//broadcastMsgToSrv(brodcast_msg);
		//sendMsg("moderator_0","init_time");
		//broadcastMsg(message);
		LOG_MSG((brodcast_msg.c_str()));
		//LOG_MSG(("%s: %s",sender.c_str(), brodcast_msg.c_str()));
		sendMsg("robot_000",brodcast_msg);
		sendMsg("moderator_0",brodcast_msg);
		sendMsg("SIGViewer",brodcast_msg);
		// m_state  = 0;
		//broadcastMsg(brodcast_msg);
       cycle = cycle+1;
	}
		if(msg == "Task_finished" && sender == "robot_000" && Mission_complete == false )
		{

	  choice_movement("finish");
		}

	if(msg == "Task_end" && sender == "moderator_0" && Mission_complete == false ) {
		
		sendMsg("moderator_0","init_time");
        choice_movement("finish");
	}
	
	if(msg == "Mission_complete" && sender == "moderator_0" && Mission_complete == false ) {
		
     Mission_complete = true;
     choice_movement("finish");
	}	


}


void UserController::parseFile(const std::string fileNam_my)
{
	std::ifstream fin;
	fin.open(fileNam_my.c_str());
	bool dest = false;
	std::string room, object;
	std::size_t found=0;
	std::size_t found2=0;
	while(!fin.eof()){
		std::size_t found=0;
		std::size_t found2=0;
		char Cbuf[MAX_CHARS_PER_LINE];
		fin.getline(Cbuf, MAX_CHARS_PER_LINE);
		std::string buf = std::string(Cbuf);


		found = buf.find(":",found2);
		if (found != std::string::npos){
			room = buf.substr(found2,found-found2);
			rooms.push_back(room);
		}


		found2 = buf.find(".",found);
		if (found2 != std::string::npos){
			object = buf.substr(found+1,found2-found-1);
			//buf = buf.substr(found+1);
			objects.push_back(object);
			// printf("found2 %d",found2);
		}
	}
	std::stringstream ss;
	std::stringstream sv;
	msg = "";

	for(int i = 0; i < rooms.size(); i++)
		{
			//strcat(kk,Data_Bayes[i].c_str());
			ss << rooms[i];
			//printf("Hey KK : %s\n",kk);
		}
	// printf("valeurs rooms \n");
	msg = ss.str();

	// printf("The information vector is %s\n",msg.c_str());
	// sendMsg("Bayes_Service",ss.str());
	ss.clear();
	msg = "";


	for(int i = 0; i < objects.size(); i++)
		{
			//strcat(kk,Data_Bayes[i].c_str());
			sv << objects[i];
			//printf("Hey KK : %s\n",kk);
		}

	//  printf("valeurs objects \n");
	msg = sv.str();

	// printf("The information vector is %s\n",msg.c_str());
	// sendMsg("Bayes_Service",ss.str());
	sv.clear();
	msg = "";
}

void UserController::choice_movement(std::string room)
{
	int task_right = 0;
	int task_left = 0;
	int task_head = 0;
	int task_body = 0;

	if (room == "lobby")
	{
		task_left = 0;
		task_right = 1;
		task_head = 2;
		task_body = 0;
	}
	else if (room == "kitchen")
	{
		task_left = 2;
		task_right = 0;
		task_head = 3;
		task_body = 0;
	}
	else if (room == "living room")
	{
		task_left = 1;
		task_right = 0;
		task_head = 4;
		task_body = 0;
	}
	else if (room == "bed room")
	{
		task_left = 0;
		task_right = 2;
		task_head = 1;
		task_body = 0;
	}
	else if (room == "finish")
	{
		task_left = 0;
		task_right = 0;
		task_head = 0;
		task_body = 1;
	}
	else
	{
		task_left = 0;
		task_right = 0;
		task_head = 0;
		task_body = 0;
	}

	moveLeftArm(task_left);
	moveRightArm(task_right);
	moveHead(task_head);
	moveBody(task_body);
}

void UserController::moveLeftArm(int task)
{
	SimObj *my = getObj(myname());

	my->setJointAngle("LARM_JOINT0", tab_joint_left[task][0]);
	my->setJointAngle("LARM_JOINT1", tab_joint_left[task][1]);
	my->setJointAngle("LARM_JOINT2", tab_joint_left[task][2]);
	my->setJointAngle("LARM_JOINT3", tab_joint_left[task][3]);
	my->setJointAngle("LARM_JOINT4", tab_joint_left[task][4]);
	my->setJointAngle("LARM_JOINT5", tab_joint_left[task][5]);
	my->setJointAngle("LARM_JOINT6", tab_joint_left[task][6]);
	my->setJointAngle("LARM_JOINT7", tab_joint_left[task][7]);
}

void UserController::moveRightArm(int task)
{
	SimObj *my = getObj(myname());

	my->setJointAngle("RARM_JOINT0", tab_joint_right[task][0]);
	my->setJointAngle("RARM_JOINT1", tab_joint_right[task][1]);
	my->setJointAngle("RARM_JOINT2", tab_joint_right[task][2]);
	my->setJointAngle("RARM_JOINT3", tab_joint_right[task][3]);
	my->setJointAngle("RARM_JOINT4", tab_joint_right[task][4]);
	my->setJointAngle("RARM_JOINT5", tab_joint_right[task][5]);
	my->setJointAngle("RARM_JOINT6", tab_joint_right[task][6]);
	my->setJointAngle("RARM_JOINT7", tab_joint_right[task][7]);
}

void UserController::moveHead(int task)
{
	SimObj *my = getObj(myname());

	my->setJointAngle("HEAD_JOINT0", tab_joint_head[task]);
}

void UserController::moveBody(int task)
{
	SimObj *my = getObj(myname());

	my->setJointAngle("WAIST_JOINT1", tab_joint_body[task]);
}


extern "C" Controller * createController ()
{
	return new UserController;
}
