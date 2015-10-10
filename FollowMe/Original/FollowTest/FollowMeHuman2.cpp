#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )   

bool start;
bool sw;
using namespace std;

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
  double rotateTowardObj1(Vector3d pos, double vel, double now);
  double rotateTowardObj2(Vector3d pos, double vel, double now);
  //avoid
  double rotateTowardObj3(Vector3d pos, double vel, double now);

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);

private:
 SimObj *my;
// ViewService *m_view;

};  

void MyController::onInit(InitEvent &evt) 
{
  my = getObj(myname()); 


  start =false;
  sw=false;
      // 左手を下に下げます  
      my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));  
  
      // 右手をsageます  
      my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

/*
      my->setJointAngle("LLEG_JOINT2", DEG2RAD(-90));  
      my->setJointAngle("RLEG_JOINT2", DEG2RAD(-90));


      my->setJointAngle("LLEG_JOINT4", DEG2RAD(90));  
      my->setJointAngle("RLEG_JOINT4", DEG2RAD(90));
*/





//  m_view = (ViewService*)connectToService("SIGViewer");  
}  
  
double MyController::onAction(ActionEvent &evt)
{  
/*
// カメラの垂直方向(radian)の視野角を取得します  
  double fovy = my->getCamFOV(5) * PI / 180.0;  
  
  // アスペクト比を取得します  
  double ar = my->getCamAS(5);  
    
  // カメラの水平方向の視野角(degree)を計算します  
  double fovx = 2 * atan(tan(fovy*0.5)*ar) * 180.0 / PI;  
  
  unsigned char distance = 255;
  unsigned char least_distance = 255;  
// 視線方向からの角度  
    double theta = 0.0;  

    // 視線方向からの水平面の角度  
    double phi = 0.0;

    double phi2 = 0.0;  
  
  if(m_view != NULL) {    
  
    // 視線方向水平面の距離データを取得します    
    ViewImage *img = m_view->distanceSensor1D(0,255,5);    
    char *buf = img->getBuffer();  

    // 距離データ画像の幅取得  
    int width = img->getWidth();  
  
    // 距離データ画像の高さ取得  
    int height = img->getHeight();   
    
    // データの長さを取得します     
 //   int length = img->getBufferLength();    
  
    
    // 距離データの最小値を求めます    
    for(int i = 0; i < width; i++){    
      for(int j = 0; j < height; j++){  
 	   int index = j *width + i;  
 	   unsigned char tmp_distance = (unsigned char)buf[index];  
  	  if(tmp_distance < least_distance){  
  
    	  // 水平方向における視線方向からの角度(rad)を計算します  
    	  phi   = fovx*i/(width-1.0) - fovx/2.0;  

  	  least_distance = buf[i]; 
	  }   
      }    
    } 
    LOG_MSG(("phi = %.1f,distance = %d",phi,least_distance));    
    delete img;   
   }

*/

/*
    //自分の位置を得ます。  
    Vector3d pos;  
    my->getPosition(pos);  
  
    //y軸周りの自分の回転を得ます。（クオータニオン）  
    Rotation rot;  
    my->getRotation(rot);  
  
    //クオータニオンから回転角を導出します。  
    double theta = 2*asin(rot.qy());  
  
    double dx = 0;  
    double dz = 0;  
  
    //移動する方向を決定します。  
    dx = sin(theta) * vel;  
    dz = cos(theta) * vel;  
*/  


/* 手をフル
    my->setJointAngle("RARM_JOINT2", DEG2RAD(-60)); 
    usleep(500000);
    my->setJointAngle("RARM_JOINT2", DEG2RAD(-40));
*/




    int count=0;
    Vector3d pos;


    if(start==true){
                while(count<20){
                    if(sw == false){
                        my->getPosition(pos);
                        my->setPosition(pos.x(),pos.y(),pos.z()-10);
                        my->setJointAngle("LARM_JOINT1", DEG2RAD(-30)); 
                        my->setJointAngle("RLEG_JOINT2", DEG2RAD(-20));
                        my->setJointAngle("RLEG_JOINT4", DEG2RAD(10));   
                        usleep(100000);
                        my->setJointAngle("LARM_JOINT1", DEG2RAD(0));  
                        my->setJointAngle("RLEG_JOINT2", DEG2RAD(0)); 
                        my->setJointAngle("RLEG_JOINT4", DEG2RAD(0)); 
                        sw = true;
                    }else{
                        my->getPosition(pos);
                        my->setPosition(pos.x(),pos.y(),pos.z()-10);
                        my->setJointAngle("RARM_JOINT1", DEG2RAD(-30)); 
                        my->setJointAngle("LLEG_JOINT2", DEG2RAD(-20));  
                        my->setJointAngle("LLEG_JOINT4", DEG2RAD(10));  
                        usleep(100000);
                        my->setJointAngle("RARM_JOINT1", DEG2RAD(0));  
                        my->setJointAngle("LLEG_JOINT2", DEG2RAD(0)); 
                        my->setJointAngle("LLEG_JOINT4", DEG2RAD(0));  
                        sw = false;
                    }
                    count++;
                    start=false;
               }
    }

    return 0.1;      
}   
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
    string msg = evt.getMsg();
    if(msg == "point1"){
        start = true;
    }
}  

void MyController::onCollision(CollisionEvent &evt) 
{

}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

