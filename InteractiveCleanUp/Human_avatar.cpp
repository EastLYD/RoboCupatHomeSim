#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:
	void moveBodyByKINECT(char* msg);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	bool slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion dest);
private:

	//移動速度
	double vel;
	ViewService *m_view;
	BaseService *m_kinect;
	BaseService *m_hmd;
	BaseService *m_wii;
    BaseService *m_orsDK2;
    BaseService *m_Sensor;

   bool take;
   bool put;


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

	bool chk_orsDK2;
    bool restart_on;




	// ロボットの名前
	std::string robotName;

	bool init_flag;
	dQuaternion bodypartsQ_pre[5], bodypartsQ_now[5], bodypartsQ_middle[5];
	bool Mission_complete;
};


void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";
    take = true;
    put = false;

	//m_kinect = connectToService("SIGKINECT");
	//m_hmd = connectToService("SIGHMD");
	m_kinect = NULL;
	m_hmd = NULL;
	m_wii = NULL;
	m_orsDK2 = NULL;
	m_Sensor = NULL;
	
	vel      = 10.0;
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

   restart_on = false;



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
	init_flag = true;
	Mission_complete = false;
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{

  // サービスが使用可能か定期的にチェックする
  bool av_kinect = checkService("SIGKINECT");


  bool av_Sensor = checkService("SIGSENSOR");

  SimObj *my = this->getObj(this->myname());
  // 使用可能
  if(av_kinect && m_kinect == NULL){
    // サービスに接続
    m_kinect = connectToService("SIGKINECT");

  }
  else if (!av_kinect && m_kinect != NULL){
    m_kinect = NULL;
  }



  if(av_Sensor && m_Sensor == NULL){
    // サービスに接続
    m_Sensor = connectToService("SIGSENSOR");

  }

  return 0.01;
}





void UserController::onRecvMsg(RecvMsgEvent &evt)
{


  std::string sender = evt.getSender();		
  //自分自身の取得
  SimObj *my = getObj(myname());
  std::string msg;
  msg= evt.getMsg();
  //メッセージ取得
  char *all_msg = (char*)evt.getMsg();
 // printf("all_msg=\n%s\n",all_msg);

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);
  //std::cout << "the status of Hesssss " <<  headss << std::endl;



//	 if( msg == "Task_start" && sender == "moderator_0" && Mission_complete == false ) {

    //std::cout << all_msg  << std::endl;

//		sendMsg("robot_000","Start_Cycle");
//		sendMsg("Kinect_000","Start_Cycle");
//	}
	 if(msg == "Show_me" && sender == "robot_000"  ) {
		std::cout << "Show_me"  << std::endl;
		sendMsg("moderator_0","Start_motion");
//        sendMsg("robot_000","Start_Task");
	}

	 if(msg == "Task_end" && sender == "moderator_0" && Mission_complete == false ) {
		
		sendMsg("moderator_0","init_time");
	//	broadcastMsg("Start_Cycle");
 
	}
	
	 if(msg == "Mission_complete" && sender == "moderator_0" && Mission_complete == false ) {
		
     Mission_complete = true;
	}	

if(headss == "KINECT_DATA") {
	  //KINECTデータによる頭部以外の体の動き反映
	  moveBodyByKINECT(all_msg);

  }

}









void UserController::moveBodyByKINECT(char* all_msg)
{
	//自分自身の取得
	SimObj *my = this->getObj(this->myname());
	char* msg = strtok(all_msg," ");



	if (strcmp(msg,"KINECT_DATA") == 0) {
		int i = 0;
		while (true) {
			i++;
			if (i == m_maxsize+1)
				break;
			char *type = strtok(NULL,":");
			if (strcmp(type,"POSITION") == 0) {
				//体の位置
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				//エージェント座標からグローバル座標への変換
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
        	//	printf("x=%f, y=%f, z=%f\n",x,y,z);
        	//	printf("m_yrot=%f, cos(m_yrot)=%f, sin(m_yrot)=%f\n",m_yrot,cos(m_yrot),sin(m_yrot));
        	//	printf("gx=%f, gz=%f\n",gx,gz);
        	//	printf("m_posx=%f, m_posy=%f, m_posz=%f\n",m_posx,m_posy,m_posz);
        		//printf("m_posx+gx=%f, m_posy+y=%f, m_posz+gz=%f\n",m_posx+gx,m_posy+y,m_posz+gz);
			//	my->setPosition(m_posx+gx,m_posy,m_posz+gz);
				//printf("the avatar postion is  X : %f  ----- Y: %f ------ Z :  %f ---- end \n",m_posx+gx,m_posy+y,m_posz+gz);
				continue;
			} 
			else if(strcmp(type,"WAIST") == 0) {
				static dQuaternion bodyQ_pre, bodyQ_now, bodyQ_middle;
				//体全体の回転 rotation of whole body
				double w = atof(strtok(NULL,","));
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				m_qw = w;
				m_qx = x;
				m_qy = y;
				m_qz = z; 
				// Spherical linear interpolation on quaternion

				my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);
				continue;
			}
			else if (strcmp(type,"END") == 0) {
				break;
			}
#if 0

#else
			//関節の回転
			double w = atof(strtok(NULL,","));
			double x = atof(strtok(NULL,","));
			double y = atof(strtok(NULL,","));
			double z = atof(strtok(NULL," "));
			double angle = acos(w)*2;
			double tmp = sin(angle/2);
			double vx = x/tmp;
			double vy = y/tmp;
			double vz = z/tmp;
			double len = sqrt(vx*vx+vy*vy+vz*vz);
			if(len < (1 - m_range) || (1 + m_range) < len) continue;
			// HEAD_JOINT1はHMDにより回転
			if(strcmp(type,"HEAD_JOINT1") != 0 ){
				my->setJointQuaternion(type,w,x,y,z);
			}
#endif

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
