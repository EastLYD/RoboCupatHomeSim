#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:
	void moveBodyByKINECT(std::string msg);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	bool slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion dest);

private:
	//初期位置
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	
	//データ数（関節数）最大値
	int m_maxsize;
	
	// 体全体の角度
	//double m_qw, m_qy, m_qx, m_qz;
	
	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;
	

	double m_qw, m_qy, m_qx, m_qz;
	double o_qw, o_qx, o_qy, o_qz;
	double po_qw, po_qx, po_qy, po_qz;


	// ロボットの名前
	std::string robotName;

	dQuaternion bodypartsQ_pre[5], bodypartsQ_now[5], bodypartsQ_middle[5];
	bool Mission_complete;
};


void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";

	srand(time(NULL));
	
	//初期位置の設定
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	//m_maxsize = 15;
	m_maxsize = 19;//20141126-tome-ikeda
	double qw = my->qw();
	double qy = my->qy();
	m_yrot = acos(fabs(qw))*2;
	if(qw*qy > 0)
		m_yrot = -1*m_yrot;
	
	// 体全体の向き
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;


	
	o_qw = 1.0;
	o_qx = 0.0;
	o_qy = 0.0;
	o_qz = 0.0;



	// Add by inamura on 28th June 2013
	my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));

	// Add by inamura on 5th March 2014
	for (int i=0; i<5; i++) {
		for (int j=1; j<4; j++) {
			// Initial quaternion
			bodypartsQ_pre[i][0] = 1.0;
			bodypartsQ_pre[i][j] = 0.0;
		}
	}

	pyaw = ppitch = proll = 0.0;
	Mission_complete = false;
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{
	return 1.0;
}





void UserController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();

	std::stringstream ss;
	ss << msg;
	std::string header;
	ss >> header;

	if(msg == "Show_me" && sender == robotName) {
		std::cout << "Show_me"  << std::endl;
		sendMsg("moderator_0","Start_motion");
//		sendMsg("robot_000","Start_Task");
	}

	if(msg == "Task_end" && sender == "moderator_0" && Mission_complete == false ) {
		sendMsg("moderator_0","init_time");
	//	broadcastMsg("Start_Cycle");
	}
	
	if(msg == "Mission_complete" && sender == "moderator_0" && Mission_complete == false ) {
		Mission_complete = true;
	}	

	if(header == "KINECT_DATA") {
		//KINECTデータによる頭部以外の体の動き反映
		moveBodyByKINECT(msg);
	}

}





void UserController::moveBodyByKINECT(std::string msg)
{
	//自分自身の取得
	SimObj *my = this->getObj(this->myname());

	std::stringstream ss;
	ss << msg;
	std::string header;
	ss >> header;

	int i = 0;
	while(ss){
		i++;
		if (i == m_maxsize+1){
			break;
		}

		std::string jointInformation;
		ss >> jointInformation;
		std::replace(jointInformation.begin(),jointInformation.end(),':',' ');
		std::replace(jointInformation.begin(),jointInformation.end(),',',' ');
		
		std::stringstream jointss;
		jointss << jointInformation;

		std::string type;
		jointss >> type;

		if(type == "POSITION"){
			double x;
			double y;
			double z;
			jointss >> x >> y >> z;
		}
		else if(type == "WAIST"){
			double w;
			double x;
			double y;
			double z;
			jointss >> w >> x >> y >> z;
			
			m_qw = w;
			m_qx = x;
			m_qy = y;
			m_qz = z; 

			my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);
			continue;
		}
		else if(type == "END"){
			break;
		}

		double w;
		double x;
		double y;
		double z;
		jointss >> w >> x >> y >> z;
		double angle = acos(w)*2;
		double tmp = sin(angle/2);
		double vx = x/tmp;
		double vy = y/tmp;
		double vz = z/tmp;
		double len = sqrt(vx*vx+vy*vy+vz*vz);
		if(len < (1 - m_range) || (1 + m_range) < len) continue;
		if(type != "HEAD_JOINT1"){
			my->setJointQuaternion(type.c_str(),w,x,y,z);
		}
	}
}


bool UserController::slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion dest)
{
	double ht = qtn1[0] * qtn2[0] + qtn1[1] * qtn2[1] + qtn1[2] * qtn2[2] + qtn1[3] * qtn2[3];
	double hs = 1.0 - ht * ht;
	if (hs <= 0.0) {//when hs is over 1.0
		dest[0] = qtn1[0];
		dest[1] = qtn1[1];
		dest[2] = qtn1[2];
		dest[3] = qtn1[3];
		printf("false\n");
		return false;
	}
	else {//when hs is under 1.0
		hs = sqrt(hs);
		if (fabs(hs) < 0.0001) { //Absolute value of hs is near 0
			dest[0] = (qtn1[0] * 0.5 + qtn2[0] * 0.5);
			dest[1] = (qtn1[1] * 0.5 + qtn2[1] * 0.5);
			dest[2] = (qtn1[2] * 0.5 + qtn2[2] * 0.5);
			dest[3] = (qtn1[3] * 0.5 + qtn2[3] * 0.5);
			printf("near 0\n");
		}
		else { //Absolute value of hs is not near 0
			double ph = acos(ht);
			double pt = ph * time;
			double t0 = sin(ph - pt) / hs;
			double t1 = sin(pt) / hs;
			dest[0] = qtn1[0] * t0 + qtn2[0] * t1;
			dest[1] = qtn1[1] * t0 + qtn2[1] * t1;
			dest[2] = qtn1[2] * t0 + qtn2[2] * t1;
			dest[3] = qtn1[3] * t0 + qtn2[3] * t1;
			printf("w=%fx=%fy=%fz=%f\t\n",dest[0],dest[1],dest[2],dest[3]);
		}
	}
	return true;
}



extern "C" Controller * createController ()
{
  return new UserController;
}
