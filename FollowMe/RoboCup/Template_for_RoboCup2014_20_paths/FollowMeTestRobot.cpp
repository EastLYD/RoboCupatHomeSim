#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
// #include <string> 
#include <sstream>
#include <iostream>
#include <math.h>
#include <unistd.h>

// Convert degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define ROBOTINFO(VAR, VAL) (std::cout << "[ROBOTINFO] " << VAR << " : " << VAL << std::endl)

int RAD2DEG (double RAD) {
	return (M_PI) * 180.0 / (RAD);
}

using namespace std;

struct coordinate{
	double x[255];
	double y[255];
	double z[255];
	double flag[255];
};


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

	int m_count;
	bool m_started;
	bool m_rotation;
	double m_d;

	coordinate node;

	int m_taskNum;

	void initCondition();
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

	m_taskNum = 0;

}

void MyController::initCondition()
{

	m_count = 0;
	FILE* fp;
	double x, y, z, flag;
	std::stringstream nodePath;
	nodePath << "nodes/node_" << m_taskNum++ << ".txt";

	if((fp = fopen(nodePath.str().c_str(), "r")) == NULL) {
		LOG_MSG(("File does not exist."));
		exit(0);
	}
	while(fscanf(fp, "%lf,%lf,%lf,%lf", &x,&y,&z,&flag) != EOF) {
			node.x[m_count]=x;
			node.y[m_count]=y;
			node.z[m_count]=z;
			node.flag[m_count++]=flag;
	}
	fclose(fp);
	m_my->setPosition(node.x[0], node.y[0], node.z[0] - 100);
	m_count = 1;
	m_started = false;
	m_rotation = false;
	m_d = -1;
}

double MyController::onAction(ActionEvent &evt)
{
	// check if the task is started or not
	if (m_started) {
		SimObj* me = getRobotObj(myname());
		Vector3d pos;
		me->getPosition(pos);
		int dx=(node.x[m_count]-pos.x());
		int dz=(node.z[m_count]-pos.z());
		double angle = atan2(dx,dz);
		// get distance between next step and robot position
		double dist = pow(dx,2) + pow(dz,2);

		// offset not to collide wth the human agent.

		if (evt.time() >= m_time) {

			if (node.flag[m_count] == 3.0) {
				node.x[m_count] = -1100;
			}
			else if (node.flag[m_count] == 3.5) {					
				// notify and wait while in the elevator 
				broadcastMsg("entered");
				node.x[m_count] = -800;
				m_my->setWheelVelocity(0.0, 0.0);
				node.flag[m_count] = 3.6;
				sleep(3);
			}
			// wait for the human to exit from the elevator.
			else if (node.flag[m_count] == 4.0) {
				broadcastMsg("ok");
				node.flag[m_count] = 4.1;
				sleep(3);
			}
			// avoid to collide with robot group
			else if (node.flag[m_count] == 5.0) {
				
			}
			

			// if the robot has reached the step or if it is rolling away from it, go to the next step
			if (((dx == 0 && dz == 0) 
				|| (dist > m_d && m_d > 0))
				&& !m_rotation) {
				++m_count;
				m_rotation = true;
			}

			// rotate toward the next step
			if (m_rotation) {
				
				m_time = rotateTowardObj(Vector3d(node.x[m_count],
													node.y[m_count],
													node.z[m_count]),
													m_vel, evt.time());
				if (m_time == 0.0) {
					m_rotation = false;
					m_d = -1;
					m_time = evt.time();
				}
			}
			// move to the current step
			else {

				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(Vector3d(node.x[m_count],node.y[m_count],node.z[m_count]), m_vel*20, 1.0, evt.time());
				m_d = pow(dx,2) + pow(dz,2);
			}



		}
	}

	return 0.05;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string msg = evt.getMsg();
	if(msg == "start"){
		m_state = 100;
		initCondition();
		broadcastMsg("get message: start");
		m_started = true;
	}
	else if(msg == "end"){
		//m_my->setWheelVelocity(0.0, 0.0);
		m_state = 999;
		broadcastMsg("get message: end");
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

	if(fabs(targetAngle) <= 0.025){
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


