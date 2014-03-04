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
    double x[256];
    double y[256];
    double z[256];
    double w[256];
};


bool first;
bool start;
bool check1;
bool elevator;
bool check3;
bool crowd;
bool end;
bool stop;
bool flg;

double x,y,z,w;
int i; 
coordinate temp; 

using namespace std;


class MyController : public Controller {  
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 


	/* @brief  ゴミを認識しゴミの位置と名前を返す
	* @return  pos ゴミの位置
	* @return  ゴミの名前
	* @return  ゴミの認識に成功した場合はtrue
	*/
	bool recognizeTrash(Vector3d &pos, std::string &name); 

	/* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	* @param  pos 回転したい方向の位置
	* @param  vel 回転速度
	* @param  now 現在時間
	* @return 回転終了時間
	*/
	double rotateTowardObj(Vector3d pos, double vel, double now);
  

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

	// ゴミの場所
	Vector3d m_tpos;  

	// ゴミの名前
	std::string m_tname;  

	/* ロボットの状態
	* 0 初期状態
	* 1 ゴミがある方向に回転している状態
	* 2 関節を曲げてゴミを取りに行っている状態
	* 3 ゴミを持ってゴミ箱の方向に回転している状態
	* 4 ゴミを持ってゴミ箱に向かっている状態
	* 5 ゴミを捨てて関節角度を元に戻している状態
	* 6 元に場所に戻る方向に回転している状態
	* 7 元の場所に向かっている状態
	* 8 元の向きに回転している状態
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

	Vector3d npos;

	// 初期位置
	Vector3d m_inipos;
};  

void MyController::onInit(InitEvent &evt) 
{  
	m_my = getRobotObj(myname());

	// 初期位置取得
	m_my->getPosition(m_inipos);

	start = false;
	check1 = false;
	elevator = false;
	check3 = false;
	crowd = false;
	end = false;
	stop = false;
	flg=false;

	first = false;
	i=0;

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
	}

	if(first==false){
		FILE* fp;
		x=0;
		y=0;
		z=0;
		w=0; //チェックポイント

		if((fp = fopen("node2.txt", "r")) == NULL) {
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

	switch(m_state){

	// 初期状態
	case 0: {
		npos.x(temp.x[i]); 
		npos.y(temp.y[i]); 
		npos.z(temp.z[i]);

		m_time= rotateTowardObj(npos,m_vel,evt.time());
		m_state = 1;

		break;
	}
	// 回転中
	case 1: {

		// 回転終了
		if(evt.time() >= m_time){

			// 回転を止める
			m_my->setWheelVelocity(0.0, 0.0);
			m_time = goToObj(npos, m_vel*20, 1.0, evt.time());
			m_state = 2;

		}
		break;
	}
	case 2: {
 
		if(evt.time() >= m_time){
			m_my->setWheelVelocity(0.0, 0.0);

			if(temp.w[i] == 2.0){
				std::string msg = "elevator";  
				//"man_000"にメッセージを送信します
				//broadcastMsgToSrv("Elevator");  
				sendMsg("man_000", msg);
				m_state = 0;
				i++;
			}else if(temp.w[i] == 3.0){
				std::string msg = "ok";  
				//"man_000"にメッセージを送信します
				//broadcastMsgToSrv("Elevator");  
				sendMsg("man_000", msg);
				sleep(10);
				m_state = 0;
				i++;
			}else{
			std::string msg = "elevator";
			//"man_000"にメッセージを送信します
			//broadcastMsgToSrv("Elevator");  
			sendMsg("man_000", msg);
			m_state = 0;
			i++;
			}
		}
		break;
	}
	case 99: {
		if(flg==false){
			m_my->setWheelVelocity(0.0, 0.0);
			std::string msg = "Collision";  
			broadcastMsg(msg);
			flg=true;
		}
	}
	}
	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	if(msg == "check1"){
		check1 = true;
	}else if(msg == "elevator"){
		elevator = true;
	}else if(msg == "check3"){
		check3 = true;
	}
}

void MyController::onCollision(CollisionEvent &evt) 
{
	//m_state =99;
}

bool MyController::recognizeTrash(Vector3d &pos, std::string &name)
{
	/////////////////////////////////////////////
	///////////ここでゴミを認識します////////////
	/////////////////////////////////////////////

	// 候補のゴミが無い場合
	if(m_trashes.empty()){
	return false;
	}

	// ここでは乱数を使ってゴミを決定します
	int trashNum = rand() % m_trashes.size();

	// ゴミの名前と位置を取得します
	name = m_trashes[trashNum];
	SimObj *trash = getObj(name.c_str());

	// ゴミの位置取得
	trash->getPosition(pos);
	return true;
}

double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
	// 自分の位置の取得
	Vector3d myPos;
	m_my->getPosition(myPos);

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
	double distance = m_distance*PI*fabs(targetAngle)/(2*PI);

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
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{
	// 自分の位置の取得
	Vector3d myPos;
	m_my->getPosition(myPos);

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

