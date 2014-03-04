#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <sstream>

using namespace std;

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

private:
  SimObj *m_my;
  bool check1;
  bool elevator;
  bool crowd;
  bool collision;
  bool check1_clear;
  bool elevator_clear;
  bool not_elevator;
  bool a;
  bool b;
  bool c;
  bool d;
  int total;
};  
  
void MyController::onInit(InitEvent &evt) {  
  check1=false;
  elevator=false;
  crowd=false;
  collision=false;
  check1_clear=false;
  elevator_clear=false;
  not_elevator = false;
  a=false;
  b=false;
  c=false;
  d=false;
  total = 0;
}  
  
double MyController::onAction(ActionEvent &evt) 
{  
  if(check1==true && check1_clear==true && a== false){
      total=total+300;
      stringstream ss;
      ss << total;
      string result = ss.str();
      sendMsg("SIGViewer",result);
      a=true;
  }

  if(elevator_clear == false && elevator==true){
      total = total - 100;
      stringstream ss4;
      ss4 << total;
      string result4 = ss4.str();
      sendMsg("SIGViewer",result4);
      elevator =false;
  }

  if(elevator==true && elevator_clear == true && b==false){
      total=total+300;
      stringstream ss2;
      ss2 << total;
      string result2 = ss2.str();
      sendMsg("SIGViewer",result2);
      b=true;
      elevator_clear=false; //誤メッセージ判定のため
      elevator=false;
  }
  if(collision==true && c == false){
      total=0;
      stringstream ss3;
      ss3 << total;
      string result3 = ss3.str();
      sendMsg("SIGViewer",result3);
      c=true;
  }
  if(crowd == true && d == false){
      total=total+400;
      stringstream ss5;
      ss5 << total;
      string result5 = ss5.str();
      sendMsg("SIGViewer",result5);
      d=true;
  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
    string msg = evt.getMsg();
    if(msg == "checkpoint1_clear"){
        check1_clear = true;
    }else if(msg == "elevator_clear"){
        elevator_clear = true;
    }else if(msg == "Collision"){
        collision = true;
    }else if(msg == "check1"){
        check1 = true;
    }else if(msg == "elevator"){
        elevator = true;
    }else if (msg == "crowd"){
        crowd = true;
    }
}  

void MyController::onCollision(CollisionEvent &evt) { 
}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

