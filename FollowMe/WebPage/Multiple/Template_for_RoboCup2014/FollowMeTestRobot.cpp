#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>

// Convert degree to radian
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
	 * @brief  Start rotating toword target position and return rotation time.
	 * @param  pos target position
	 * @param  vel velocity
	 * @param  now current time
	 * @return end time of rotation
	 */
	double rotateTowardObj(Vector3d pos, double vel, double now);

	/**
	 * @brief  Move to target position
	 * @param  pos   target position
	 * @param  vel   velocity
	 * @param  range allowable error range
	 * @param  now   current time
	 * @return end time of rotation
	 */
	double goToObj(Vector3d pos, double vel, double range, double now);

private:
	RobotObj *m_my;

	/**
	 * State of robot
	 *   0  wait
	 * 100+ 1st section
	 * 200+ 2nd section
	 * 300+ 3rd section
	 * 900+ finish line
	 */
	int m_state;

	double m_vel;      // angular velocity of wheel
	double m_radius;   // radius of wheel
	double m_distance; // distance of wheel

	double m_time; // time to stop moving
};

void MyController::onInit(InitEvent &evt) 
{
	m_my = getRobotObj(myname());

	// radius and distance of wheel
	m_radius = 10.0;
	m_distance = 10.0;

	// time to stop moving
	m_time = 0.0;

	// set radius and distance of wheel
	m_my->setWheel(m_radius, m_distance);

	// angular velocity of wheel
	m_vel = 0.3;

	// state of robot
	m_state = 0;
}

double MyController::onAction(ActionEvent &evt)
{
	switch(m_state){
		// wait
		case 0:{
			break;
		}
		// 1st section
		case 100: {
			m_time = rotateTowardObj(Vector3d(-50,60,500),m_vel,evt.time());
			m_state = 101;
			break;
		}
		case 101: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-50,60,500), m_vel*20, 1.0, evt.time());
				m_state = 102;
			}
			break;
		}
		case 102: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-50,60,575), m_vel*20, 1.0, evt.time());
				m_state = 103;
			}
			break;
		}
		case 103: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-450,60,575),m_vel,evt.time());
				m_state = 104;
				break;
			}
		}
		case 104: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-450,60,575), m_vel*20, 1.0, evt.time());
				m_state = 105;
			}
			break;
		}
		case 105: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				// wait for a walking person
				usleep(3200000);
				m_state = 106;
			}
			break;
		}
		case 106: {
			m_time = rotateTowardObj(Vector3d(-525,60,575),m_vel,evt.time());
			m_state = 107;
			break;
		}
		case 107: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-525,60,575), m_vel*20, 1.0, evt.time());
				m_state = 200;
			}
			break;
		}
		// 2nd section
		case 200: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,575),m_vel,evt.time());
				m_state = 201;
			}
			break;
		}
		case 201: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,575), m_vel*20, 1.0, evt.time());
				m_state = 202;
			}
			break;
		}
		case 202: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,275),m_vel,evt.time());
				m_state = 203;
			}
			break;
		}
		case 203: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,275), m_vel*20, 1.0, evt.time());
				m_state = 204;
			}
			break;
		}
		case 204: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-1100,60,275),m_vel,evt.time());
				m_state = 205;
			}
			break;
		}
		case 205: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-1100,60,275), m_vel*20, 1.0, evt.time());
				m_state = 206;
			}
			break;
		}
		case 206: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);

				// tell robot entered in an elevator
				broadcastMsg("entered");

				//  wait for a while until a door opened
				//sleep(5);
				m_state = 207;
			}
			break;
		}
		case 207: {
			break;
		}
		case 208: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,275),m_vel,evt.time());
				m_state = 209;
			}
			break;
		}
		case 209: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,275), m_vel*20, 1.0, evt.time());
				m_state = 210;
			}
			break;
		}
		case 210: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);

				// 
				std::string msg = "ok";
				broadcastMsg(msg);

				// 
				sleep(10);
				m_state = 300;
			}
			break;
		}
		// 3rd section
		case 300: {
			m_time = rotateTowardObj(Vector3d(-800,60,-200),m_vel,evt.time());
			m_state = 301;
			break;
		}
		case 301: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-200), m_vel*20, 1.0, evt.time());
				m_state = 302;
			}
			break;
		}
		case 302: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-970,60,-200),m_vel,evt.time());
				m_state = 303;
			}
			break;
		}
		case 303: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-970,60,-200), m_vel*20, 1.0, evt.time());
				m_state = 304;
			}
			break;
		}
		case 304: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-970,60,-400),m_vel,evt.time());
				m_state = 305;
			}
			break;
		}
		case 305: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-970,60,-400), m_vel*20, 1.0, evt.time());
				m_state = 306;
			}
			break;
		}
		case 306: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,-550),m_vel,evt.time());
				m_state = 307;
			}
			break;
		}
		case 307: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-550), m_vel*20, 1.0, evt.time());
				m_state = 900;
			}
			break;
		}
		// finish line
		case 900: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(Vector3d(-800,60,-750),m_vel,evt.time());
				m_state = 901;
			}
			break;
		}
		case 901: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(-800,60,-750), m_vel*20, 1.0, evt.time());
				m_state = 999;
			}
			break;
		}
		case 999: {
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsg("Task_finished");
				m_state = 0;
			}
			break;
		}
	}
	return 0.05;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string msg = evt.getMsg();
	if(msg == "Task_start"){
		m_state = 100;
		broadcastMsg("get message: Task_start");
	}
	else if(msg == "Task_end"){
		m_my->setWheelVelocity(0.0, 0.0);
		m_state = 0;
		broadcastMsg("get message: Task_end");
	}
	else if(m_state == 207 && msg == "leave the elevator"){
		broadcastMsg("get message: leave the elevator");
		m_state = 208;
	}
}

void MyController::onCollision(CollisionEvent &evt) 
{
}

double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
	// get own position
	Vector3d myPos;
	m_my->getPosition(myPos);

	// vector from own position to a target position
	Vector3d tmpp = pos;
	tmpp -= myPos;

	// rotation about y-axis is always 0
	tmpp.y(0);

	// get own rotation
	Rotation myRot;
	m_my->getRotation(myRot);

	// initial direction
	Vector3d iniVec(0.0, 0.0, 1.0);

	// get rotation angle about y-axis
	double qw = myRot.qw();
	double qy = myRot.qy();
	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0){
		theta = -theta;
	}

	// angle from z-axis
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// calcurate target angle
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
		// circumference length for rotation
		double distance = m_distance*M_PI*fabs(targetAngle)/(2*M_PI);

		// calcurate velocity from radius of wheels
		double vel = m_radius*velocity;

		// rotation time (micro second)
		double time = distance / vel;

		// start rotating
		if(targetAngle > 0.0){
			m_my->setWheelVelocity(velocity, -velocity);
		}
		else{
			m_my->setWheelVelocity(-velocity, velocity);
		}

		return now + time;
	}
}

// move to a target point
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{
	// get own position
	Vector3d myPos;
	m_my->getPosition(myPos);

	// vector from own position to a target position
	pos -= myPos;

	pos.y(0);

	// distance to a target position
	double distance = pos.length() - range;

	// calcurate veloocity from radius of wheels
	double vel = m_radius*velocity;

	// start moving
	m_my->setWheelVelocity(velocity, velocity);

	// calcurate time of arrival
	double time = distance / vel;

	return now + time;
}

extern "C" Controller * createController() {
	return new MyController;  
}


