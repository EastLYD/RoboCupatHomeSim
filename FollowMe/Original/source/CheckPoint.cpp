#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
  
class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

private:
  SimObj *m_my;
  std::vector<std::string> m_entities;
  
  double tboxSize_x, tboxSize_z;

  double tboxMin_y, tboxMax_y;

};  
  
void MyController::onInit(InitEvent &evt) {  
  m_my = getObj(myname());
  getAllEntities(m_entities);

  tboxSize_x  = 20.0;
  tboxSize_z  = 40.5; 
  tboxMin_y    = 40.0;
  tboxMax_y    = 1000.0;
}  
  
double MyController::onAction(ActionEvent &evt) 
{  
  // 自分の位置取得
  Vector3d myPos;
  m_my->getPosition(myPos);
  
  int entSize = m_entities.size();
  for(int i = 0; i < entSize; i++){
    if(m_entities[i] == "robot_004"){
      
        // エンティティ取得
        SimObj *ent = getObj(m_entities[i].c_str());

        // 位置取得
        Vector3d tpos;
        ent->getPosition(tpos);

        Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());

        if(abs(vec.x()) < tboxSize_x/2.0 &&
           abs(vec.z()) < tboxSize_z/2.0){
          LOG_MSG(("Check Point 1 Clear !"));
          std::string msg = "checkpoint1_clear";
          sendMsg("score", msg);
        }
    }
  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
}  

void MyController::onCollision(CollisionEvent &evt) { 
}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

