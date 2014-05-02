#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  void onCollision(CollisionEvent &evt);
  bool recognizeTrash(Vector3d &pos, std::string &name);
  std::string getPointedTrashName(std::string entName);
 double PathinFrontObject(std::string room, std::string object);
  // エージェントが指差している方向にあるオブジェクトの名前を取得します
  std::string getPointedObjectName(std::string entName);
  // 位置を指定しその方向に回転を開始し、回転終了時間を返します
  double rotateTowardObj(Vector3d pos, double vel, double now);
  double rotateTowardObj(Vector3d pos);
  // 位置を指定しその方向に進みます
  double goToObj(Vector3d pos, double vel, double range, double now);
  double goToObj(Vector3d pos, double range);

  void stopRobotMove(void);
  void recognizeObjectPosition(Vector3d &pos, std::string &name);
  double goGraspingObject(Vector3d &pos);
  void neutralizeArms(double evt_time);






private:
  // Takeitの対称から除外するオブジェクトの名前
  std::vector<std::string> exobj;
  std::vector<std::string> extrash;
  RobotObj *m_my;
  Vector3d m_tpos;
  std::string m_graspObjectName;
  //RobotObj *m_my;
  int m_robotState;
  int m_state;
  int m_frontState;
  double m_movingSpeed;      // actual velocity of the moving robot

   std::string room_msg;
   std::string object_msg;

  Vector3d m_defaultNormalVector;	// エージェントの正面の方向ベクトル通常はz軸の正の方向と考える

  std::string m_pointedObject;  // 取りにいくオブジェクト名

	// pointed trash
  std::string m_pointedtrash;

  // onActionの戻り値
  double m_onActionReturn;

  // 1stepあたりの移動距離
  double m_speedDelta;

  // ゴミの名前
  std::string m_tname;
  // ゴミ候補オブジェクト
  std::vector<std::string> m_trashes;
  // ゴミ箱オブジェクト
  std::vector<std::string> m_trashboxs;

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
 // double m_jointVelocity;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // 移動終了時間
  double m_time;
  double m_time_LA1;
  double m_time_LA4;
  double m_time_RA1;
  double m_time_RA4;
  double m_jointVelocity;    // rotation speed around the joint
  // 初期位置
  Vector3d m_inipos;

	Vector3d m_relayPoint0;
	Vector3d m_relayPoint1;
	Vector3d m_relayPoint2;
  Vector3d m_relayPoint3;
  Vector3d m_relayPoint4;
  Vector3d m_relayPoint5;



	Vector3d m_kitchenPoint;
	Vector3d m_livingPoint;
	Vector3d m_bedroomPoint;
	Vector3d m_lobbyPoint;
  Vector3d m_originPoint;
  Vector3d m_livingroomPoint;
  Vector3d m_lastPoint;

  // access points
  // kitchen
   std::vector<Vector3d> m_frontkitchen;
  Vector3d m_kitchenFront1;
  Vector3d m_kitchenFront2;
  Vector3d m_kitchenFront3;
  Vector3d m_kitchenFront4;
  Vector3d m_kitchenFront5;
  Vector3d m_kitchenFront6;
  Vector3d m_kitchenFront7;
  // bedroom
  std::vector<Vector3d> m_frontbedroom;
  Vector3d m_bedroomFront1;
  Vector3d m_bedroomFront2;
  Vector3d m_bedroomFront3;

 // lobby
  std::vector<Vector3d> m_frontlooby;
  Vector3d m_lobbyFront1;
  Vector3d m_lobbyFront2;
  Vector3d m_lobbyFront3;
  Vector3d m_lobbyFront4;
 // living room
  std::vector<Vector3d> m_frontlivingroom;
  Vector3d m_livingroomFront1;
  Vector3d m_livingroomFront2;
  Vector3d m_livingroomFront3;

  int stopfront;


  // grasp中かどうか
  bool m_grasp;

  // ゴミ認識サービス
  BaseService *m_recogSrv;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;

  std::string msg_ob;
  std::string msg_trash;

  // times wasted for moving, adjusting or driving
  //double m_time;
  double m_time1;
  double m_time4;

 Vector3d posR;

  double m_angularVelocity;  // rotation speed of the wheel


};

void RobotController::onInit(InitEvent &evt)
{
  m_robotState = 0;
  //エージェントの正面の定義はz軸の正の向きを向いていると仮定する
  m_defaultNormalVector = Vector3d(0,0,1);
  m_onActionReturn = 0.1;
  m_speedDelta = 2.0;

  m_my = getRobotObj(myname());

  // important to modify
 // m_my = getRobotObj(myname());
  // 初期位置取得
  //m_my->getPartsPosition(m_inipos,"RARM_LINK2");

  room_msg = "";
  object_msg ="";

  stopfront = 0;
  posR = Vector3d(0, 0, 0);


	m_relayPoint1 = Vector3d(20, 30, 0);
	m_relayPoint2 = Vector3d(20, 30, 350);
	m_relayPoint3 = Vector3d(200, 30, 320);
  m_relayPoint4 = Vector3d(-400, 30, 0);
  m_relayPoint5 = Vector3d(-400, 30, -130);




	m_originPoint = Vector3d(20, 30, -200);
	m_kitchenPoint = Vector3d(380, 30, 320);
	m_bedroomPoint = Vector3d(-400, 30, 200);
	m_lobbyPoint = Vector3d(-300, 30, -180);
  m_livingroomPoint = Vector3d(300, 30, -130);
  Vector3d m_lastPoint = Vector3d(0, 0, 0);




  
  m_kitchenFront1 = Vector3d(390, 30, 220);
  m_kitchenFront2 = Vector3d(390, 30, 120);
  m_kitchenFront3 = Vector3d(270, 30, 120);
  m_kitchenFront4 = Vector3d(150, 30, 120);
  m_kitchenFront5 = Vector3d(150, 30, 230);
  m_kitchenFront6 = Vector3d(150, 30, 300);
  m_kitchenFront7 = Vector3d(270, 30, 320);
 
  m_frontkitchen.push_back(m_kitchenFront1 );
  m_frontkitchen.push_back(m_kitchenFront2 );
  m_frontkitchen.push_back(m_kitchenFront3 );
  m_frontkitchen.push_back(m_kitchenFront4 );
  m_frontkitchen.push_back(m_kitchenFront5 );
  m_frontkitchen.push_back(m_kitchenFront6 );
  m_frontkitchen.push_back(m_kitchenFront7 );
  // bedroom
  m_bedroomFront1 = Vector3d(-400, 30, 340);
  m_bedroomFront2 = Vector3d(-150, 30, 340);
  m_bedroomFront3 = Vector3d(-400, 30, 340);
  m_frontbedroom.push_back(m_bedroomFront1 );
  m_frontbedroom.push_back(m_bedroomFront2 );
  m_frontbedroom.push_back(m_bedroomFront3 );
 

 // lobby
  m_lobbyFront1 = Vector3d(-130, 30, -320);
  m_lobbyFront2 = Vector3d(-220, 30, -220);
  m_lobbyFront3 = Vector3d(-190, 30, -160);
  m_lobbyFront4 = Vector3d(-350, 30, -180);
  m_frontlooby.push_back(m_lobbyFront1 );
  m_frontlooby.push_back(m_lobbyFront2 );
  m_frontlooby.push_back(m_lobbyFront3 );
  m_frontlooby.push_back(m_lobbyFront4 );


 // living room
 m_livingroomFront1 = Vector3d(300, 30, -135);
 m_livingroomFront2 = Vector3d(0, 30, -70);
 m_livingroomFront3 = Vector3d(110, 30, 0);
  m_frontlivingroom.push_back(m_livingroomFront1 );
  m_frontlivingroom.push_back(m_livingroomFront2 );
  m_frontlivingroom.push_back(m_livingroomFront3 );
 // 車輪半径
  m_radius  = 10.0;
  m_angularVelocity = 1;

 m_frontState = 0;

  m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)



  // 車輪間距離
  m_distance = 10.0;

 
  

  // 移動終了時間初期化
  m_time = 0.0;
  m_time_LA1 = 0.0;
  m_time_LA4 = 0.0;
  m_time_RA1 = 0.0;
  m_time_RA4 = 0.0;

  // 車輪の半径と車輪間距離設定
  m_my->setWheel(m_radius, m_distance);
  m_state = 0;

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 0.6;

  // 関節の回転速度
  m_jointVelocity = 1.0;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;

  // ここではゴミの名前が分かっているとします
  m_trashes.push_back("petbottle_1");
  m_trashes.push_back("can_0");
  m_trashes.push_back("apple");

  // ゴミ箱登録
  m_trashboxs.push_back("trashbox_0");
  m_trashboxs.push_back("trashbox_1");
  m_trashboxs.push_back("trashbox_2");

	//broadcastMsgToSrv("Waiting 'start' message\n");

}

double RobotController::onAction(ActionEvent &evt)
{
  
if (m_grasp == true)
{

      m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
      m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);

}
 // getObj("apple_0")->getPosition(posR);
 //  printf("Apple 0  :  X : %f    Y :  %f Z :  %f \n",posR.x() , posR.y(), posR.z() );

  // getObj("apple_1")->getPosition(posR);
  // printf("Apple 1  :  X : %f    Y :  %f Z :  %f \n",posR.x() , posR.y(), posR.z() );

 // getObj("apple_2")->getPosition(posR);
 // printf("Apple 2  :  X : %f    Y :  %f Z :  %f \n",posR.x() , posR.y(), posR.z() );

  //getObj("apple_3")->getPosition(posR);
  //printf("Apple 3  :  X : %f    Y :  %f Z :  %f \n",posR.x() , posR.y(), posR.z() );


 // printf("the stopfront is %d\n",stopfront);
 // printf("the State is %d\n",m_state);
   


 //   m_my->setJointVelocity("LARM_JOINT4", 10.0, 0.0);
	switch(m_state){
	





//case 0: {

    case 100: {  // direct to the trash
      if(evt.time() >= m_time ) {
        stopRobotMove();    // at first, stop robot maneuver
        sendMsg("moderator_0","Start grasping process");

        //broadcastMsgToSrv("Start grasing");
        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);  // get position of object
        double l_moveTime = rotateTowardObj(l_tpos);  // rotate toward the position and calculate the time to be elapsed.

        m_time = l_moveTime+evt.time();
        m_state = 101;
      }
      break;
    }
    case 101: {  // proceed toward trash
      if(evt.time() >= m_time ) {
        this->stopRobotMove();

        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);
        double l_moveTime = goToObj(l_tpos, 75.0);  // go toward the position and calculate the time to be elapsed.

        m_time = l_moveTime+evt.time();
        m_state = 102;
      }
      break;
    }
    case 102: {  // get back a bit after colliding with the table
      if(evt.time() >= m_time ) {
        this->stopRobotMove();    // at first, stop robot maneuver

        m_my->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
        m_time = 20./m_movingSpeed + evt.time();
        m_state = 103;
      }
      break;
    }
    
   case 103: {  // rotate toward the trash
      if(evt.time() >= m_time ) {
        this->stopRobotMove();

        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);
        double l_moveTime = rotateTowardObj(l_tpos);

        m_time = l_moveTime+evt.time();
        m_state = 104;
      }
      break;
    }
    case 104: {  // prepare the robot arm to grasping the trash
      if(evt.time() >= m_time ) {
        this->stopRobotMove();
        this->neutralizeArms(evt.time());

        m_state = 105;
      }
      break;
    }
    case 105: {  // fix robot direction for grasping
      if(evt.time() >= m_time1 ) m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
      if(evt.time() >= m_time4 ) m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
      if(evt.time() >= m_time1 && evt.time() >= m_time4 ) {
        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);
        double l_moveTime = rotateTowardObj(l_tpos);

        m_time = l_moveTime+evt.time();

        m_state = 106;
      }
      break;
    }
    case 106: {  // approach to the trash
      if(evt.time() >= m_time ) {
        this->stopRobotMove();

        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);
        double l_moveTime = goToObj(l_tpos, 30.0);
        m_time = l_moveTime+evt.time();

        m_state = 107;
      }
      break;
    }
    case 107: {  // try to grasp trash
      if(evt.time() >= m_time ) {
        this->stopRobotMove();
        Vector3d l_tpos;
        this->recognizeObjectPosition(l_tpos, m_pointedObject);
        double l_moveTime = goGraspingObject(l_tpos);
        m_time = l_moveTime+evt.time();

        m_state = 108;
      }
      
      break;
    }
    case 108: {  

      broadcastMsg("Object_grasped");
      LOG_MSG(("Object_grasped"));
     m_state = 110;

      break;
    }
    case 110: {
          
      if(evt.time() >= m_time ) {
        m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
        this->neutralizeArms(evt.time());
      if(evt.time() >= m_time1 )  m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
      if(evt.time() >= m_time4 )  m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
     // sendMsg("moderator_0","Object_grasped");
      
        m_state = m_frontState;
      }
      break;
    }


    
      case 109: {
        
if(room_msg=="kitchen")
    {
    m_state = 50;
    room_msg = "";
    object_msg = "";
    }
      else if(room_msg=="lobby")
       {
        m_state = 60;
        room_msg = "";
        object_msg = "";
       }
     else if(room_msg=="bed room")
       {
        m_state = 70;
        room_msg = "";
        object_msg = "";
       }
            else if(room_msg=="living room")
       {
         m_state = 80;
         room_msg = "";
         object_msg = "";
       }
       
    break;
  }


 case 200: {
    // ゴミ箱に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      // grasp中のパーツを取得します getting grasped tarts
      CParts *parts = m_my->getParts("RARM_LINK7");
      // releaseします
      parts->releaseObj();
      // ゴミが捨てられるまで少し待つ
      sleep(1);
      // 捨てたゴミをゴミ候補から削除 deleting grasped object from list
     //   std::vector<std::string>::iterator it;
      //  it = std::find(m_trashes.begin(), m_trashes.end(), m_pointedObject);
      //  m_trashes.erase(it);
      // grasp終了
      m_grasp = false;

     m_state++;
    }
    break;
  }
case 201: {

      broadcastMsg("Task_finished");
      LOG_MSG(("Task_finished"));
       sleep(1);
      // sendMsg("man_000","Task_finished");
      m_state = 1;
    break;
  }







		// move toward the kitchen

		case 10: { // set rotation for relay point
		//	broadcastMsgToSrv("Move toward the kitchen");
    sleep(1);
          broadcastMsg("Task_start");
         LOG_MSG(("Task_start"));
      sleep(1);
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 11: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
      case 12: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint2, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 13: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 14: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint3, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 15: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint3, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
		case 16: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_kitchenPoint, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 17: { // rotate toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_kitchenPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 18: { // move toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("I arrived at the kitchen");
				//broadcastMsgToSrv("Waiting a next message\n");
        
			m_state++;
        
			}
        
			break;
		}
    case 19: { 
      
      broadcastMsg("Room_reached");
      LOG_MSG(("Room_reached"));
         sleep(1);
       
        m_lastPoint= m_kitchenPoint;
        PathinFrontObject(room_msg,m_pointedObject);
      
      break;
    }
	


		// move toward the lobby
		case 20: { // set rotation for relay point
			//broadcastMsgToSrv("Move toward the lobby");
        broadcastMsg("Task_start");
        LOG_MSG(("Task_start"));
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 21: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 22: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 23: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 24: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint5, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 25: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint5, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 26: { // move toward lobby point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_lobbyPoint, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 27: { // rotate toward lobby point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_lobbyPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 28: { // move toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("I arrived at the lobby");
				//broadcastMsgToSrv("Waiting a next message\n");
       m_state++;
				
        
			}
			break;
		}

case 29: { 
      broadcastMsg("Room_reached");
      LOG_MSG(("Room_reached"));
        m_state = 0;
        m_lastPoint= m_kitchenPoint;
        PathinFrontObject(room_msg,m_pointedObject);
      
      break;
    }


		// move toward the bed room
		case 30: { // set rotation for relay point
			//broadcastMsgToSrv("Move toward the bed room");
        broadcastMsg("Task_start");
         LOG_MSG(("Task_start"));
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 31: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 32: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
				m_state++;
			}
			break;
		}
    case 33: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 34: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_bedroomPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }

		case 35: { // rotate toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_bedroomPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 36: { // move toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("I arrived at the bed room");
		
	     m_state++;

         
			}
     
			break;
		}
case 37: { // move toward bed room point
      
       broadcastMsg("Room_reached");
       LOG_MSG(("Room_reached"));

        m_lastPoint = m_bedroomPoint;
        PathinFrontObject(room_msg,m_pointedObject);
     
      
     
      break;
    }


// move toward the bed room
    case 40: { //set rotation for relay point
              broadcastMsg("Task_start");
              LOG_MSG(("Task_start"));
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_livingroomPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }

    case 41: { // rotate toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_livingroomPoint, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 42: { // move toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
       //  broadcastMsgToSrv("I arrived at the living room");
       // broadcastMsgToSrv("Task finished\n");
        m_state++;

      }
       
      break;
    }
case 43: { // move toward bed room point

       broadcastMsg("Room_reached");
       LOG_MSG(("Room_reached"));
        m_lastPoint = m_livingroomPoint;     
      break;
    }

// move back to the kitchen

    case 50: { // set rotation for relay point
     // broadcastMsgToSrv("Move toward the kitchen");

      m_time = rotateTowardObj(m_relayPoint3, m_vel, evt.time());
      m_state++;
      break;
    }
    case 51: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint3, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
      case 52: { // move toward relay point

      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint2, m_vel, evt.time());
        m_state++;
      }
      break;
    }


    case 53: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
        m_state=1000;
      }
      break;
    }
    case 1000: {
           sendMsg("moderator_0","End grasping process");
      m_state= 54;
                break;
    }
    case 54: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 55: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 56: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_originPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 57: { // rotate toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_originPoint, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 58: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
      //  broadcastMsgToSrv("came back from the kitchen");
      //  broadcastMsgToSrv("Waiting a next message\n");
        m_state = 200;
        m_lastPoint = m_originPoint;
      }
      break;
    }



// move back to the lobby
    case 60: { // set rotation for relay point
     // broadcastMsgToSrv("Move toward the lobby");
      m_time = rotateTowardObj(m_relayPoint5, m_vel, evt.time());
      m_state++;
      break;
    }
    case 61: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint5, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 62: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 63: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 64: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 65: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 66: { // move toward lobby point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_originPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 67: { // rotate toward lobby point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_originPoint, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 68: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
    //    broadcastMsgToSrv("Came back from the lobby");
     //   broadcastMsgToSrv("Waiting a next message\n");
        m_state = 200;
      }
      break;
    }



// move back to the bed room
    case 70: { // set rotation for relay point
   //   broadcastMsgToSrv("Move toward the bed room");
      m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
      m_state++;
      break;
    }
    case 71: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 72: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
        m_state++;
      }
      break;
    }
    case 73: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 74: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_originPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }

    case 75: { // rotate toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_originPoint, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 76: { // move toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
      //  broadcastMsgToSrv("Came back from the bed room");
      //  broadcastMsgToSrv("Task finished\n");
        m_state = 200;
      }
      break;
    }


// move back from the bed room
    case 80: { //set rotation for relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_originPoint, m_vel, evt.time());
        m_state++;
      }
      break;
    }

    case 81: { // rotate toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_originPoint, m_vel*4, 0.0, evt.time());
        m_state++;
      }
      break;
    }
    case 82: { // move toward bed room point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
     //   broadcastMsgToSrv("I arrived at the living room");
     //   broadcastMsgToSrv("Task finished\n");
        m_state = 200;
      }
      break;
    }


   


case 300: { // set rotation for relay point

      m_time = rotateTowardObj(m_kitchenFront1, m_vel, evt.time());

      m_state= m_state+10;
      break;
    }
    case 310: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront1, m_vel*4, 0.0, evt.time());
        if(stopfront == 1){
           stopfront = 0;
           m_frontState = m_state+10;
           
            if(evt.time() >= m_time){
          // 回転を止める
            m_my->setWheelVelocity(0.0, 0.0);
           }
            m_state = 100;
        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
      case 320: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront2, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 330: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront2, m_vel*4, 0.0, evt.time());
        if(stopfront == 2){
           stopfront = 0;
           m_frontState = m_state+10;   
           
            
            m_state = 100;
        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
    case 340: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront3, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 350: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront3, m_vel*4, 0.0, evt.time());
        if(stopfront == 3){
           stopfront = 0;
          m_frontState = m_state+10;
          
          
            m_state = 100;
        }
        else {
       m_state = m_state+10;
      }
      }
      break;
    }
    case 360: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront4, m_vel, evt.time());
        m_state= m_state+10;
      }
      break;
    }
    case 370: { // rotate toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront4, m_vel*4, 0.0, evt.time());
        if(stopfront == 4){
           stopfront = 0;
           m_frontState = m_state+10;
           
          m_state = 100;

        }
        else {
       m_state = m_state+10;
      }
      }
      break;
    }
    case 380: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront5, m_vel, evt.time());
        m_state= m_state+10;
      }
      break;
    }
    case 390: { // rotate toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront5, m_vel*4, 0.0, evt.time());
        if(stopfront == 5){
           stopfront = 0;
           m_frontState = m_state+1;
          
           m_state = 100;
        }
        else {
       m_state = m_state+1;
      }
      }
      break;
    }

case 391: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront6, m_vel, evt.time());
       m_state = m_state+1;
      }
      break;
    }
    case 392: { // rotate toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront6, m_vel*4, 0.0, evt.time());
        if(stopfront == 6){
           stopfront = 0;
           m_frontState = m_state+1;
          
          m_state = 100;

        }
        else {
       m_state = m_state+1;
      }
      }
      break;
    }
case 393: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_kitchenFront7, m_vel, evt.time());
        m_state= m_state+1;
      }
      break;
    }
    case 394: { // rotate toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_kitchenFront7, m_vel*4, 0.0, evt.time());
        if(stopfront == 7){
           stopfront = 0;
           m_frontState = m_state+1;
           if(evt.time() >= m_time){
          // 回転を止める
            m_my->setWheelVelocity(0.0, 0.0);
           }
           m_state = 100;
        }
        else {
       m_state = m_state+1;
      }
      }
      break;
    }


    case 395: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);

        m_state = 0;
        m_lastPoint= m_kitchenPoint;
        m_state = 109;
      }
      break;
    }








case 400: { // set rotation for relay point

      m_time = rotateTowardObj(m_lobbyFront1, m_vel, evt.time());

      m_state = m_state+10;
      break;
    }
    case 410: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_lobbyFront1, m_vel*4, 0.0, evt.time());
        if(stopfront == 1){
           stopfront = 0;
           m_frontState = m_state+10;
            m_state = 100;
        }
        else {
       m_state = m_state+10;
      }

      }
      break;
    }
      case 420: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_lobbyFront2, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 430: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_lobbyFront2, m_vel*4, 0.0, evt.time());
        if(stopfront == 2){
           stopfront = 0;
           m_frontState = m_state+10;
            m_state = 100;

        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
    case 440: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_lobbyFront3, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 450: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_lobbyFront3, m_vel*4, 0.0, evt.time());
        if(stopfront == 3){
           stopfront = 0;
           m_frontState = m_state+10;
            m_state = 100;

        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
  case 460: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_lobbyFront4, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 470: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_lobbyFront4, m_vel*4, 0.0, evt.time());
        if(stopfront == 4){
           stopfront = 0;
           m_frontState = m_state+10;
            m_state = 100;

        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
    case 480: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);

        m_state = 0;
        m_lastPoint= m_lobbyPoint;
        m_state = 109;
      }
      break;
    }



case 500: { // set rotation for relay point

      m_time = rotateTowardObj(m_bedroomFront1, m_vel, evt.time());

      m_state = m_state+10;
      break;
    }
    case 510: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_bedroomFront1, m_vel*4, 0.0, evt.time());
        if(stopfront == 1){
                 stopfront = 0;
                 m_frontState = m_state+10;
                  m_state = 100;

        }
        else {
        m_state++;
      }
      }
      break;
    }
      case 520: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_bedroomFront2, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 530: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_bedroomFront2, m_vel*4, 0.0, evt.time());
        if(stopfront == 2){

         stopfront = 0;
         m_frontState = m_state+10;
          m_state = 100;
        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
    case 540: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_bedroomFront3, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 550: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_bedroomFront3, m_vel*4, 0.0, evt.time());
        if(stopfront == 3){
           stopfront = 0;
           m_frontState = m_state+10;
            m_state = 100;

        }
        else {
       m_state = m_state+10;
      }
      }
      break;
    }
  
    case 560: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);

        
        m_lastPoint= m_bedroomPoint;
        m_state = 109;
        stopfront = 0;
      }
      break;
    }


case 600: { // set rotation for relay point

      m_time = rotateTowardObj(m_livingroomFront1, m_vel, evt.time());

     m_state = m_state+10;
      break;
    }
    case 610: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_livingroomFront1, m_vel*4, 0.0, evt.time());
        if(stopfront == 1){
           stopfront = 0;
           m_frontState = m_state+10;
           m_state = 100;
        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
      case 620: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_livingroomFront2, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 630: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_livingroomFront2, m_vel*4, 0.0, evt.time());
        if(stopfront == 2){
           stopfront = 0;
           m_frontState = m_state+10;
              m_state = 100;
        }
        else {
        m_state = m_state+10;
      }
      }
      break;
    }
    case 640: { // move toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = rotateTowardObj(m_livingroomFront3, m_vel, evt.time());
        m_state = m_state+10;
      }
      break;
    }
    case 650: { // rotate toward relay point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);
        m_time = goToObj(m_livingroomFront3, m_vel*4, 0.0, evt.time());
        if(stopfront == 3){
           stopfront = 0;
           m_frontState = m_state+10;
             m_state = 100;
        }
        else {
       m_state = m_state+10;
      }
      }
      break;
    }
  
    case 660: { // move toward kitchen point
      if(evt.time() >= m_time){
        m_my->setWheelVelocity(0.0, 0.0);

        m_state = 0;
        m_lastPoint= m_livingPoint;
      //  m_state = 100;
        stopfront = 0;
      }
      break;
    }





  }
  return m_onActionReturn;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
 std::string msg    = evt.getMsg();
	// 送信者がゴミ認識サービスの場合
//	char *all_msg = (char*)evt.getMsg();

   std::size_t found=0;
   std::size_t found2=0;
   std::size_t found3=0;
   std::string task;
   LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));
   //std::ifstream fin;
 // fin.open(fileNam_my.c_str());
   bool dest = false;
   if(sender == "man_000")
   {
   // std::cout << "the message Robot is " + msg  << std::endl ;
     found = msg.find("Go to the ",0);
    if (found != std::string::npos){
      task = msg.substr(found+10);
     // rooms.push_back(room);
     // printf("task %s \n",task);
     //  broadcastMsgToSrv("Task_start");
   // std::cout << "Task : "+ task  << std::endl;
    }

    found2 = task.find(" grasp the ",0);
    if (found3 != std::string::npos){
       room_msg = task.substr(0,found2);
     // rooms.push_back(room);
   // printf("room %s \n",room_msg);
    //  std::cout << "room : "+room_msg  << std::endl;
    }
     
      found3 = task.find(" and come back here",found);
      if (found3 != std::string::npos){
         object_msg = task.substr(found2+11,found3-found2-11);
      //  buf = buf.substr(found+1);
      //  objects.push_back(object);
      //  cout << "object"+object_msg  ;
      //  std::cout << "object : "+object_msg  << std::endl;
      //  printf("object %s \n",object_msg);
       
 }


// kitchen message

 if(room_msg == "living room" ){

    m_state = 40; // move to the kitchen
    
    if(object_msg == "apple" )
  {
    m_pointedObject = "apple_0"; 
  }
    if(object_msg == "can" )
  {
    m_pointedObject = "can_0"; 
  }
  if(object_msg == "mug" )
  {
    m_pointedObject = "mug_0"; 
  }
  if(object_msg == "pet" )
  {
    m_pointedObject = "pet_0"; 
  }

  }

// lobby message
 if(room_msg == "bed room" ){

    m_state = 30; // move to the kitchen
    
    if(object_msg == "apple" )
  {
    m_pointedObject = "apple_3"; 
  }
    if(object_msg == "can" )
  {
    m_pointedObject = "can_3"; 
  }
  if(object_msg == "mug" )
  {
    m_pointedObject = "mug_3"; 
  }

  if(object_msg == "pet" )
  {
    m_pointedObject = "pet_3"; 
  }

  }

 if(room_msg == "lobby" ){

    m_state = 20; // move to the kitchen

    if(object_msg == "apple" )
  {
    m_pointedObject = "apple_1"; 
  }
    if(object_msg == "can" )
  {
    m_pointedObject = "can_1"; 
  }
  if(object_msg == "mug" )
  {
    m_pointedObject = "mug_1"; 
  }
  if(object_msg == "mug" )
  {
    m_pointedObject = "mug_1"; 
  }
  if(object_msg == "pet" )
  {
    m_pointedObject = "pet_1"; 
  }

  }

  if(room_msg == "kitchen" ){

    m_state = 10; // move to the kitchen
    
    if(object_msg == "apple" )
  {
    m_pointedObject = "apple_2"; 
  }
    if(object_msg == "can" )
  {
    m_pointedObject = "can_2"; 
  }
  if(object_msg == "mug" )
  {
    m_pointedObject = "mug_2"; 
  }
  
  if(object_msg == "pet" )
  {
    m_pointedObject = "pet_2"; 
  }

  }


}
  
/*	if(msg == "start" && m_state ==0)
	{
		m_state = 1; // initial settings
	}
	
	else
	{
	//	broadcastMsgToSrv("Task finished\n");
	}*/
}


std::string RobotController::getPointedObjectName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashSize = m_trashes.size();
  for(int i = 0; i < trashSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashes[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  if(cos < 0)
    continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}


void RobotController::onCollision(CollisionEvent &evt)
{
  if (m_grasp == false){
    typedef CollisionEvent::WithC C;
    //触れたエンティティの名前を得ます
    const std::vector<std::string> & with = evt.getWith();
    // 衝突した自分のパーツを得ます
    const std::vector<std::string> & mparts = evt.getMyParts();
    //　衝突したエンティティでループします
    for(int i = 0; i < with.size(); i++){
      //右手に衝突した場合
      if(mparts[i] == "RARM_LINK7"){
        //自分を取得
        SimObj *my = getObj(myname());
        //自分の手のパーツを得ます
        CParts * parts = my->getParts("RARM_LINK7");
        if(parts->graspObj(with[i])){
          m_grasp = true;
        }
      }
    }
  }
}
double RobotController::rotateTowardObj(Vector3d pos)
{
  // "pos" means target position
  // get own position
  Vector3d ownPosition;
  m_my->getPartsPosition(ownPosition,"RARM_LINK2");

  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= ownPosition;

  // ignore variation on y-axis
  l_pos.y(0);

  // get own rotation matrix
  Rotation ownRotation;
  m_my->getRotation(ownRotation);

  // get angles arround y-axis
  double qw = ownRotation.qw();
  double qy = ownRotation.qy();
  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0) theta = -1.0*theta;

  // rotation angle from z-axis to x-axis
  double tmp = l_pos.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);

  // direction
  if(l_pos.x() > 0) targetAngle = -1.0*targetAngle;
  targetAngle += theta;

  double angVelFac = 3.0;
  double l_angvel = m_angularVelocity/angVelFac;

  if(targetAngle == 0.0) {
    return 0.0;
  }
  else {
    // circumferential distance for the rotation
    double l_distance = m_distance*M_PI*fabs(targetAngle)/(2.0*M_PI);

    // Duration of rotation motion (micro second)
    double l_time = l_distance / (m_movingSpeed/angVelFac);

    // Start the rotation
    if(targetAngle > 0.0) {
      m_my->setWheelVelocity(l_angvel, -l_angvel);
    }
    else{
      m_my->setWheelVelocity(-l_angvel, l_angvel);
    }

    return l_time;
  }
}
double RobotController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
  // 自分の位置の取得
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  tmpp -= myPos;

  // y方向は考えない
  tmpp.y(0);

  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);

  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();

  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // z方向からの角度
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);

  // 方向
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;
  targetAngle += theta;

  if(targetAngle == 0.0){
    return 0.0;
  }
  else {
    // 回転すべき円周距離
    double distance = m_distance*M_PI*fabs(targetAngle)/(2*M_PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius*velocity;

    // 回転時間(u秒)
    double time = distance / vel;

    // 車輪回転開始
    if(targetAngle > 0.0){
      m_my->setWheelVelocity(velocity, -velocity);
    }
    else{
      m_my->setWheelVelocity(-velocity, velocity);
    }

    return now + time;
  }
}

// object まで移動
double RobotController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  pos -= myPos;

  // y方向は考えない
  pos.y(0);

  // 距離計算
  double distance = pos.length() - range;

  // 車輪の半径から移動速度を得る
  double vel = m_radius*velocity;

  // 移動開始
  m_my->setWheelVelocity(velocity, velocity);

  // 到着時間取得
  double time = distance / vel;

  return now + time;
}


double RobotController::goToObj(Vector3d pos, double range) {
  // get own position
  Vector3d robotCurrentPosition;
  //m_robotObject->getPosition(robotCurrentPosition);
  m_my->getPartsPosition(robotCurrentPosition,"RARM_LINK2");

  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= robotCurrentPosition;

  // ignore y-direction
  l_pos.y(0);

  // measure actual distance
  double distance = l_pos.length() - range;

  // start moving
  m_my->setWheelVelocity(m_angularVelocity, m_angularVelocity);

  // time to be elapsed
  double l_time = distance / m_movingSpeed;

  return l_time;
}




void RobotController::stopRobotMove(void) {
  m_my->setWheelVelocity(0.0, 0.0);
}
void RobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
  // get object of trash selected
  SimObj *trash = getObj(name.c_str());

  // get trash's position
  trash->getPosition(pos);
}

double RobotController::goGraspingObject(Vector3d &pos)
{
  double l_time;
  double thetaJoint4 = 20.0;

  m_my->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

  l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

  return l_time;
}


void RobotController::neutralizeArms(double evt_time)
{
  double angleJoint1 = m_my->getJointAngle("RARM_JOINT1")*180.0/(M_PI);
  double angleJoint4 = m_my->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
  double thetaJoint1 = -15 - angleJoint1;
  double thetaJoint4 = -110 - angleJoint4;

  if(thetaJoint4<0) m_my->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
  else m_my->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

  if(thetaJoint1<0) m_my->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
  else m_my->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

  m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;
  m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;
}


double RobotController::PathinFrontObject(std::string room, std::string object)
{

Vector3d obj_pos;
Vector3d dist;
double distance;
distance = 0;
double distance2;
distance2 = 10000;
obj_pos =  Vector3d(20, 30, -200);
SimObj *obj = getObj(object.c_str());

  // get trash's position
  obj->getPosition(obj_pos);



if(room == "kitchen" ){

m_state = 300;


 for(int i=0;i<m_frontkitchen.size();i++)
{

//dist = m_frontkitchen[i]-obj_pos;
dist = obj_pos;
dist  -= m_frontkitchen[i];

distance = dist.length();
if(distance < distance2)
{
 distance2 = distance;
 stopfront = i+1;
}

}
distance2 = 1000;
distance = 0;
//printf("the value of distance is %f and distance 2 is : %f\n",distance,distance2);
}



else if (room == "lobby" ){

m_state = 400;
for(int j=0;j<m_frontlooby.size();j++)
{
//dist = m_frontkitchen[j]-obj_pos;
dist = obj_pos;
dist  -= m_frontlooby[j];

distance = dist.length();
if(fabs(distance)< fabs(distance2))
{
 distance2 = distance;
 stopfront = j+1;
}


}
distance2 = 1000;
distance = 0;


}
else if (room == "bed room" ){
m_state = 500;
for(int k=0;k<m_frontbedroom.size();k++)
{
dist = obj_pos;
dist  -= m_frontbedroom[k];
distance = dist.length();
printf("the value of distance is %f and distance 2 is : %f\n",distance,distance2);
if(fabs(distance)< fabs(distance2))
{
 distance2 = distance;
 stopfront = k+1;
}

}
distance2 = 1000;
distance = 0;

}


else if (room == "living room" ){
m_state = 600;
for(int m=0;m<m_frontlivingroom.size();m++)
{
dist = obj_pos;
dist  -= m_frontlivingroom[m];
distance = dist.length();
if(fabs(distance)< fabs(distance2))
{
 distance2 = distance;
 stopfront = m+1;
}


}
distance2 = 1000;
distance = 0;
}

}


//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}
