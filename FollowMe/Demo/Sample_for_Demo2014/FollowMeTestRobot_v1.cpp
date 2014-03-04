#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

struct coordinate{
    double x[255];
    double y[255];
    double z[255];
    double w[255];
};


bool first;
bool start;
bool elevator;
bool crowd;
bool end;
bool stop;


double dx,dy,dz;
double x,y,z,w;
int i; 
coordinate temp; 



class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

  /* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
   * @param  pos 回転したい方向の位置
   * @param  vel 回転速度
   * @param  now 現在時間
   * @return 回転終了時間
   */
  double rotateTowardObj(Vector3d pos);

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);

private:
  RobotObj *m_my;

  ViewService *m_view;

  // ゴミの場所
  Vector3d m_tpos;  

  // ゴミの名前
  std::string m_tname;  

  /* ロボットの状態
   * 0 初期状態
   */
  int m_state; 

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
  double m_jvel;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // ゴミ候補オブジェクト
  std::vector<std::string> m_trashes;

  // 移動終了時間
  double m_time;

  // 初期位置
  Vector3d m_inipos;

  // grasp中かどうか
  bool m_grasp;

  //collision 
  bool collision;

};  

void MyController::onInit(InitEvent &evt) 
{



  start = false;
  elevator = false;
  crowd = false;
  end = false;
  stop = false;

  first = false;
  i=0;

  m_my = getRobotObj(myname());

  // 初期位置取得
  m_my->getPosition(m_inipos);

  // 車輪の半径と車輪間隔距離
  m_radius = 10.0;
  m_distance = 10.0;

  m_time = 0.0;

  // 車輪の半径と車輪間距離設定
  m_my->setWheel(m_radius, m_distance);
  m_state = 0;

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 0.3;

  // 関節の回転速度
  m_jvel = 0.6;

}  
  
double MyController::onAction(ActionEvent &evt)
{  
  if(first == false){
    std::string msg = "start";  
    broadcastMsg(msg);
    first = true; 
  }

    int count=0;
    double r=0.0; //2点間の直線距離
    double angle;

    if(end==false){
        if(start==true){

            Vector3d pos;
            Vector3d npos;

            if(first==false){
                FILE* fp;           
                x=0;
                y=0;
                z=0;
                w=0; //チェックポイント
      
                dx=0;
                dy=0;
                dz=0;

                if((fp = fopen("node.txt", "r")) == NULL) {
	             printf("File do not exist.\n");
	             exit(0);
                }
                while(fscanf(fp, "%lf,%lf,%lf,%lf", &x, &y, &z,&w) != EOF) {
                    temp.x[i]=x;
                    temp.y[i]=y;
                    temp.z[i]=z;
                    temp.w[i]=w;
                    i++;
                }
                fclose(fp);
                first = true;
                i=0;
            }

            if(stop==false){
                my->getPosition(pos);

                npos.x(temp.x[i]); 
                npos.y(temp.y[i]); 
                npos.z(temp.z[i]); 

                angle = rotateTowardObj(npos);   

                if(angle < 0.0){
                    angle = -1.0 * angle;
                }
 		/*ここに相当する部分を書く*/
                //my->setAxisAndAngle(0, 1.0, 0, -angle);
   	        // 回転すべき円周距離
                double distance = m_distance*PI*fabs(targetAngle)/(2*PI);
                if(targetAngle > 0.0){
                    m_my->setWheelVelocity(velocity, -velocity);
                }
                else{
                    m_my->setWheelVelocity(-velocity, velocity);
                }
                // 車輪の半径から移動速度を得る
                double vel = m_radius*velocity;
    
                // 回転時間(u秒)
                double time = (distance / vel) + evt.time();
    
                if(evt.time>=time){
		    m_my->setWheelVelocity(0, 0);
                }

                dx=(temp.x[i]-pos.x());
                dy=(temp.y[i]-pos.y());
                dz=(temp.z[i]-pos.z());

                r=sqrt(pow(dx,2)+pow(dz,2));
                //ここまでが現在地から次の座標までの距離と角度の計算

                double vel = m_radius*velocity;

                // 移動開始
                m_my->setWheelVelocity(velocity, velocity);

                // 到着時間取得
                double time2 = r / vel;

                count = 0;
                count2= 0;
                i++;
            }


            if(temp.w[i-1] == 1.0){
                std::string msg = "point1";  
                //"robot_000"にメッセージを送信します  
                sendMsg("man_001", msg); 

            }else if(temp.w[i-1] == 2.0){

                stop=true;
                broadcastMsgToSrv("elevator");

                if(elevator==true){
                   stop=false;
                }
            }
        }
    }

/***************************************************************************/
  
/*
      if(strstr(myname() , "robot_004")!= NULL){    
        m_my->setWheelVelocity(m_vel*2, m_vel*2);
      }
*/
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
}  

void MyController::onCollision(CollisionEvent &evt) 
{
  std::string msg = "Collision";  
  
  broadcastMsg(msg); 
}
  

double MyController::rotateTowardObj(Vector3d pos)
{
  // 自分の位置の取得
  Vector3d myPos;
  my->getPosition(myPos);

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  tmpp -= myPos;
  
  // y方向は考えない
  tmpp.y(0);
  
  // 自分の回転を得る
  Rotation myRot;
  my->getRotation(myRot);

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

  return targetAngle;
}


// object まで移動
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);
 // m_my->getPartsPosition(myPos,"RARM_LINK7");
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

extern "C" Controller * createController() {  
  return new MyController;  
}  

