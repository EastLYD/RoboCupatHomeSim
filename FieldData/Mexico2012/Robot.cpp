#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class DemoRobotController : public Controller {
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 

private:

	double refreshRateOnAction;

};


void DemoRobotController::onInit(InitEvent &evt) {

	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

}


double DemoRobotController::onAction(ActionEvent &evt) {
	return refreshRateOnAction;
}


void DemoRobotController::onRecvMsg(RecvMsgEvent &evt) {
}

//********************************************************************
extern "C" Controller * createController() {  
  return new DemoRobotController;  
}  
