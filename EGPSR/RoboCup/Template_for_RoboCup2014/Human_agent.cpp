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

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define MAX_CHARS_PER_LINE 512


class UserController : public Controller
{
public:


  void onRecvMsg(RecvMsgEvent &evt);
  void onInit(InitEvent &evt);
  double onAction(ActionEvent &evt);
  void parseFile(const std::string fileNam_my);
private:

  //移動速度
  double vel;
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
};

void UserController::onInit(InitEvent &evt)
{
  robotName = "robot_000";

  //m_kinect = connectToService("SIGKINECT");
  //m_hmd = connectToService("SIGHMD");
parseFile("command.txt");

//printf("Reslutat %s", rooms[2]);
rooms.clear();
objects.clear();
m_message = 10 ;

cycle = 0;

  vel      = 10.0;
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

 if(msg == "Task_start"   && sender == "moderator_0"  && cycle < 2) {
  sendMsg("moderator_0","init_time");
 
  std::string message,brodcast_msg;
  parseFile("command.txt");
  message = ":"+ rooms[cycle] +";"+ objects[cycle]+"."; 
  brodcast_msg = "Go to the "+ rooms[cycle] +", grasp the "+ objects[cycle]+" and come back here"; 
//broadcastMsgToSrv(brodcast_msg);
  broadcastMsg(brodcast_msg);
//LOG_MSG(("%s: %s",sender.c_str(), brodcast_msg.c_str()));
//sendMsg("robot_000",message);
//sendMsg("moderator_0",message);
  }

 if(msg == "Task_end" && sender == "moderator_0"  ) {
   
   cycle = cycle+1;
  sendMsg("moderator_0","init_time");

  }

    


}


void UserController::parseFile(const std::string fileNam_my){
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
     
   
      found2 = buf.find(",",found);
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

extern "C" Controller * createController ()
{
  return new UserController;
}
