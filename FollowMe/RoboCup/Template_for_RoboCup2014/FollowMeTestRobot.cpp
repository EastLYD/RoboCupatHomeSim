#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

using namespace std;


class MyController : public Controller
{
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

	/**
	 * @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	 * @param  pos 回転したい方向の位置
	 * @param  vel 回転速度
	 * @param  now 現在時間
	 * @return 回転終了時間
	 */
	double rotateTowardObj(Vector3d pos, double vel, double now);


	/**
	 * @brief  位置を指定しその方向に進みます
	 * @param  pos   行きたい場所
	 * @param  vel   移動速度
	 * @param  range 半径range以内まで移動
	 * @param  now   現在時間
	 * @return 到着時間
	 */
	double goToObj(Vector3d pos, double vel, double range, double now);

private:
	RobotObj *m_my;

	/**
	 * State of robot
	 *  0  wait
	 * 10+ 1st section
	 * 20+ 2nd section
	 * 30+ 3rd section
	 * 90+ finish line
	 */
	int m_state;

	double m_vel;      // 車輪の角速度
	double m_radius;   // 車輪半径
	double m_distance; // 車輪間距離

	double m_time; // 移動終了時間
};

void MyController::onInit(InitEvent &evt) 
{
	m_my = getRobotObj(myname());

	// 車輪の半径と車輪間隔距離
	m_radius = 10.0;
	m_distance = 10.0;

	// 移動終了時間
	m_time = 0.0;

	// 車輪の半径と車輪間距離設定
	m_my->setWheel(m_radius, m_distance);

	// 車輪の回転速度
	m_vel = 0.3;

	m_state = 0;
}

double MyController::onAction(ActionEvent &evt)
{
	switch(m_state){
		case 0:{
			break;
		}
		// 1st section
		case 10: {
			m_time = rotateTowardObj(Vector3d(-50,60,500),m_vel,evt.time());
			m_state = 11;
			break;
		}
		case 11: {
			if(evt.time() >= m_time){
				// 回転を止める
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-50,60,500), m_vel*20, 1.0, evt.time());
				m_state = 12;
			}
			break;
		}
		case 12: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-50,60,575), m_vel*20, 1.0, evt.time());
				m_state = 13;
			}
			break;
		}
		case 13: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-450,60,575),m_vel,evt.time());
				m_state = 14;
				break;
			}
		}
		case 14: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-450,60,575), m_vel*20, 1.0, evt.time());
				m_state = 15;
			}
			break;
		}
		case 15: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				usleep(3200000);
				m_state = 16;
			}
			break;
		}
		case 16: {
			m_time = rotateTowardObj(Vector3d(-525,60,575),m_vel,evt.time());
			m_state = 17;
			break;
		}
		case 17: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-525,60,575), m_vel*20, 1.0, evt.time());
				m_state = 20;
			}
			break;
		}
		// 2nd section
		case 20: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,575),m_vel,evt.time());
				m_state = 21;
			}
			break;
		}
		case 21: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,575), m_vel*20, 1.0, evt.time());
				m_state = 22;
			}
			break;
		}
		case 22: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,275),m_vel,evt.time());
				m_state = 23;
			}
			break;
		}
		case 23: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,275), m_vel*20, 1.0, evt.time());
				m_state = 24;
			}
			break;
		}
		case 24: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-1100,60,275),m_vel,evt.time());
				m_state = 25;
			}
			break;
		}
		case 25: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-1100,60,275), m_vel*20, 1.0, evt.time());
				m_state = 26;
			}
			break;
		}
		case 26: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//std::string msg = "elevator";
				//broadcastMsg(msg);
				broadcastMsg("entered");
				//LOG_MSG(("elevator"));
				//elevator = true;
				sleep(5);
				m_state = 27;
			}
			break;
		}
		case 27: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,275),m_vel,evt.time());
				m_state = 28;
			}
			break;
		}
		case 28: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,275), m_vel*20, 1.0, evt.time());
				m_state = 29;
			}
			break;
		}
		case 29: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				std::string msg = "ok";
				broadcastMsg(msg);
				sleep(10);
				m_state = 30;
			}
			break;
		}
		// 3rd section
		case 30: {
			m_time = rotateTowardObj(Vector3d(-800,60,-200),m_vel,evt.time());
			m_state = 31;
			break;
		}
		case 31: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-200), m_vel*20, 1.0, evt.time());
				m_state = 32;
			}
			break;
		}
		case 32: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-970,60,-200),m_vel,evt.time());
				m_state = 33;
			}
			break;
		}
		case 33: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-970,60,-200), m_vel*20, 1.0, evt.time());
				m_state = 34;
			}
			break;
		}
		case 34: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-970,60,-400),m_vel,evt.time());
				m_state = 35;
			}
			break;
		}
		case 35: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-970,60,-400), m_vel*20, 1.0, evt.time());
				m_state = 36;
			}
			break;
		}
		case 36: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,-550),m_vel,evt.time());
				m_state = 37;
			}
			break;
		}
		case 37: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-550), m_vel*20, 1.0, evt.time());
				m_state = 90;
			}
			break;
		}
		// finish line
		case 90: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,-750),m_vel,evt.time());
				m_state = 91;
			}
			break;
		}
		case 91: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-750), m_vel*20, 1.0, evt.time());
				m_state = 99;
			}
			break;
		}
		case 99: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsg("end");
				m_state = 0;
			}
			break;
		}
	}
	return 0.05;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	if(msg == "start"){
		m_state = 10;
	}
	else if(msg == "end"){
		m_my->setWheelVelocity(0.0, 0.0);
	}
}

void MyController::onCollision(CollisionEvent &evt) 
{
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

	if(qw*qy < 0){
		theta = -theta;
	}

	// z方向からの角度
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(tmpp.x() > 0){
		targetAngle = -1*targetAngle;
	}
	targetAngle += theta;

	if(targetAngle<-M_PI){
		 targetAngle += 2*M_PI;
	}
	else if(targetAngle>M_PI){
		targetAngle -= 2*M_PI;
	}

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


