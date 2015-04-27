#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <iomanip>

#define NUMBER_OF_REPETITION   10 /// 10

enum Reachable{
  UP,
  DOWN,
  LEFT,
  RIGHT
};

typedef struct target{
	std::string name;
	float x;
	float y;
	float z;
	float height; //bb on y-axis
	float length; //bb on x-axis
	float width;  //bb on z-axis
} Target;

typedef struct table{
	std::string name;
	float x;
	float y;
	float z;
	float length; //bb on x-axis
	float height; //bb on y-axis
	float width;  //bb on z-axis
	int reachable[4];
} Table;


class MyController : public Controller {
public:
	void   onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void   onCheckCollision();
	void   onCheckRoom();
	void   onCheckObject();
	void   onRecvMsg(RecvMsgEvent &evt);
	void   onCheckPositionfrontHuman();
	bool   checkAvailablePos(float posObj, Target obj, int indPosOnTable, std::vector<Target> vec);
	int    contains(std::vector<Table> vec, std::string key);
	int    contains(std::vector<Target> vec, std::string key);
	template<typename Type>
	int    contains(std::vector<Type> vec, Type key);
	int    getNextPosOnTable(int indPosOnTable, Table table);
	void   getNextTable(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable);
	void   performChange(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable);
	float  mapRange(float s, float a1, float a2, float b1, float b2);
	bool   checkRobotFinished();
	void   initRoomsObjects();
	void   reposObjects();
	void   onCheckFinalPosition();
	void   breakTask();
	// void takeAwayObjects();

private:
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification
	bool   colState;      // Collision state
	bool  pcolState;      // Collision state
	std::string roboName; // Robot's name
	std::string mdName;   // Moderator's name
	std::string humanName;   // Human's name
	const static unsigned int jtnum=7;
	std::vector<std::string> jointName;
	double prv1JAng_r[jtnum];
	double crrJAng_r[jtnum];

	std::vector<std::string> m_entities;
	std::vector<std::string> m_entNames;
	int entNum;

	std::vector<std::string> m_rooms;
	int m_roomState;

	// position vectors and rotation matrices required to modify robot's behavior
	Vector3d crrPos;
	Vector3d prv1Pos;
	Rotation crrRot;
	Rotation prv1Rot;

	std::string room_msg;
	std::string object_msg ;

	double take_time  ;
	bool init ;

	bool time_display ;

	Vector3d Obj_pos;
	Vector3d PrObj_pos;

	int trialCount;

	bool   isCleaningUp;
	double startTime;
	double endTime;

	bool Task_st;

	std::string m_pointedObject;

	double rsLen;
	bool grasping;

	bool unable_collision;

	std::map< std::string, std::vector<Target> > m_targets; //key is the name of the room
	std::map< std::string, std::vector<Table> > m_tables; //key is the name of the room
	Vector3d robotInitialPos;
	Vector3d humanPos;
	float radius;
};


void MyController::onInit(InitEvent &evt)
{
	m_rooms.push_back("livingroom"); //0
	m_rooms.push_back("kitchen");    //1
	m_rooms.push_back("lobby");      //2
	m_rooms.push_back("bedroom");    //3
	m_roomState = 6;

	m_pointedObject = "";
	initRoomsObjects();

	int i, cnt;
	Task_st = true;
	time_display = true;
	retValue = 0.01;
	colState = false;
	pcolState = false;
	roboName = "robot_000";
	mdName   = "moderator_0";
	humanName = "man_000";

	crrPos  = Vector3d(0,0,0);
	prv1Pos = Vector3d(0,0,0);

	Obj_pos  = Vector3d(0,0,0);
	PrObj_pos  = Vector3d(0,0,0);

	room_msg = "";
	object_msg = "" ;
	grasping = false;

	unable_collision = false;

	getObj(roboName.c_str())->getPosition(robotInitialPos);

	getObj(humanName.c_str())->getPosition(humanPos);
	radius = 200;

	getAllEntities(m_entities);
/*
  // select objects to be observed
  cnt = m_entities.size();
  for(i=0;i<cnt;i++){
    if((m_entities[i] != mdName) &&
       (m_entities[i] != roboName) &&
       (m_entities[i] != "trashbox_0") &&
       (m_entities[i] != "trashbox_1") &&
       (m_entities[i] != "trashbox_2") &&
       (m_entities[i] != "petbottle_0") &&
       (m_entities[i] != "petbottle_1") &&
       (m_entities[i] != "petbottle_2") &&
       (m_entities[i] != "petbottle_3") &&
       (m_entities[i] != "petbottle_4") &&
       (m_entities[i] != "banana") &&
       (m_entities[i] != "chigarette") &&
       (m_entities[i] != "chocolate") &&
       (m_entities[i] != "mayonaise_0") &&
       (m_entities[i] != "mayonaise_1") &&
       (m_entities[i] != "mugcup") &&
       (m_entities[i] != "can_0") &&
       (m_entities[i] != "can_1") &&
       (m_entities[i] != "can_2") &&
       (m_entities[i] != "can_3") &&
       (m_entities[i] != "apple") &&
       (m_entities[i] != "clock") &&
       (m_entities[i] != "kettle") ){
      m_entNames.push_back(m_entities[i]);
    }
  }
  entNum = m_entNames.size();
*/

	cnt = m_entities.size();

	for(i=0;i<cnt;i++) {
		int found;
		SimObj* entity = getObj(m_entities[i].c_str());
		Vector3d pos;
		entity->getPosition(pos);

		if( ( found = contains( m_tables["livingroom"], m_entities[i] ) ) != -1  ){
			if (m_tables["livingroom"][found].name != "Buffet1" )
			{
			m_tables["livingroom"][found].x = pos.x();
			m_tables["livingroom"][found].y = pos.y();
			m_tables["livingroom"][found].z = pos.z();
			}
		}

		else if( ( found = contains( m_tables["bedroom"], m_entities[i] ) ) != -1 ){
		if (m_tables["bedroom"][found].name != "Side board2" && m_tables["bedroom"][found].name != "Side table1")
			{
			m_tables["bedroom"][found].x = pos.x();
			m_tables["bedroom"][found].y = pos.y();
			m_tables["bedroom"][found].z = pos.z();
			}
		}

		else if( ( found = contains( m_tables["lobby"], m_entities[i] ) ) != -1 ){
		if (m_tables["lobby"][found].name != "Buffet2" && m_tables["lobby"][found].name != "Side board1")
			{
			m_tables["lobby"][found].x = pos.x();
			m_tables["lobby"][found].y = pos.y();
			m_tables["lobby"][found].z = pos.z();
			}
		}

		else if( ( found = contains( m_tables["kitchen"], m_entities[i] ) ) != -1 ){
			m_tables["kitchen"][found].x = pos.x();
			m_tables["kitchen"][found].y = pos.y();
			m_tables["kitchen"][found].z = pos.z();
		}

		if((m_entities[i] != mdName) &&
		   (m_entities[i] != roboName) &&
		   (m_entities[i] != "apple_0") &&
		   (m_entities[i] != "apple_1") &&
		   (m_entities[i] != "apple_2") &&
		   (m_entities[i] != "apple_3") &&
		   (m_entities[i] != "petbottle_0") &&
		   (m_entities[i] != "petbottle_1") &&
		   (m_entities[i] != "petbottle_2") &&
		   (m_entities[i] != "petbottle_3") &&
		   (m_entities[i] != "mugcup_0") &&
		   (m_entities[i] != "mugcup_1") &&
		   (m_entities[i] != "mugcup_2") &&
		   (m_entities[i] != "mugcup_3") &&
		   (m_entities[i] != "can_0") &&
		   (m_entities[i] != "can_1") &&
		   (m_entities[i] != "can_2") &&
		   (m_entities[i] != "can_3") ){
			m_entNames.push_back(m_entities[i]);
			//std::cout << "entitie: " << m_entities[i] << " pushed!" << std::endl;
		}

	}
	entNum = m_entNames.size();
	srand (2);

	reposObjects();
	trialCount = 0;
	//srand(time(NULL));

	/*for(int i=0; i<10; i++){
	  reposObjects();
	  //usleep(5000000);
	  }*/

	// std::cout << "robot is in the circle? " << checkRobotFinished() << std::endl;



	isCleaningUp = false;

	startTime =  0.0;
	endTime   = 600.0; // [sec]

	take_time = 0.0;
	init = false;
}


double MyController::onAction(ActionEvent &evt)
{
	// check whether Referee service is available or not
	bool available = checkService("RoboCupReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("RoboCupReferee");
	}

	// get information about the robot and renew it
	//----------------------------------
	SimObj *r_my = getObj(roboName.c_str());

	// for body configuration
	r_my->getPosition(crrPos);
	r_my->getRotation(crrRot);

	Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
	rsLen = rsLenVec.length();

	if(Task_st == true && trialCount < NUMBER_OF_REPETITION)
		{
			broadcastMsg("Task_start");
			// printf("tast_start moderator \n");
			Task_st = false;
			std::stringstream trial_ss;
			trial_ss << "RoboCupReferee/trial/";
			trial_ss << trialCount + 1 << "/";
			trial_ss << NUMBER_OF_REPETITION;
			if (m_ref != NULL){
				m_ref->sendMsgToSrv(trial_ss.str().c_str());
			}
			else{
				LOG_MSG((trial_ss.str().c_str()));
			}
		}
if( Task_st == true && trialCount >= NUMBER_OF_REPETITION)
	{
		// resetRobotCondition();
		LOG_MSG(("Mission_complete"));
		broadcastMsg("Mission_complete");
		if(m_ref != NULL){
            m_ref->sendMsgToSrv("RoboCupReferee/time/- END -");
		}
		time_display = false;
		Task_st = false;
	}


	onCheckCollision();

	// onCheckObject();
	for(int k=0;k<entNum;k++){
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();

		if ( unable_collision == false)
			{
				if(state){
					colState=true;       // collided with main body of robot

					if(rsLen == 0.0) {
						//        pcolState=true;
						r_my->setRotation(prv1Rot);
					} else {
						//        pcolState=false;
						r_my->setPosition(prv1Pos);
					}

					std::string msg = "RoboCupReferee/Collision with [" + m_entNames[k] + "]" "/-100";
					if(m_ref != NULL){
						m_ref->sendMsgToSrv(msg.c_str());
					}
					else{
						LOG_MSG((msg.c_str()));
					}
					break;
				}
			}
	}

	//  if(!colState && !pcolState){
	if(!colState){
		prv1Pos=crrPos;
		prv1Rot=crrRot;
		//colState=false;     // reset collision condition
	} else if(pcolState){
		for(int i=0;i<jtnum;i++) prv1JAng_r[i]=crrJAng_r[i];
		pcolState = false;
	} else{
		//Do nothing on "collided" condition
		//    LOG_MSG((colPtName.c_str()));
		colState = false;
	}

	std::stringstream time_ss;

	if(init == true)
		{
			take_time = evt.time();
			init = false;
		}

	double elapsedTime = evt.time() - startTime - take_time;

	if( time_display == true)
		{
			//if(evt.time() - startTime > endTime){
			if(elapsedTime > endTime){
				LOG_MSG(("Time_over"));
				broadcastMsg("Time_over");
				breakTask();
				//time_ss << "RoboCupReferee/time/00:00:00";
			}
			else{
				double remainedTime = endTime - elapsedTime;
				int min, sec, msec;
				sec = (int)remainedTime;
				min = sec / 60;
				sec %= 60;
				msec = (int)((remainedTime - sec) * 100);
				time_ss.str("");
				time_ss <<  "RoboCupReferee/time/";
				time_ss << std::setw(2) << std::setfill('0') << min << ":";
				time_ss << std::setw(2) << std::setfill('0') << sec;// << ":";
               //  time_ss << "youuu";
				//time_ss << std::setw(2) << std::setfill('0') << msec;
			}
			if(m_ref != NULL){
				std::string mess;
				mess =time_ss.str();
				m_ref->sendMsgToSrv(mess.c_str());
				 //std::cout << "Message to referee : " << time_ss.str() << std::endl;
			}
			else{
				LOG_MSG((time_ss.str().c_str()));
			}
		}

	return retValue;
}



void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg    = evt.getMsg();

	LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));

	// 送信者がゴミ認識サービスの場合
	char *all_msg = (char*)evt.getMsg();
	//std::string msg;
	msg= evt.getMsg();
	std::size_t found=0;
	std::size_t found2=0;
	std::size_t found3=0;
	std::string task;

	//std::ifstream fin;
	// fin.open(fileNam_my.c_str());
	bool dest = false;

	if(sender == "man_000")
		{
            //broadcastMsgToSrv(msg.c_str());
			// std::cout << "the message Moderator is "  + msg  << std::endl ;
			found = msg.find("Go to the ",0);
			if (found != std::string::npos){
				task = msg.substr(found+10);
				// rooms.push_back(room);
				// printf("task %s \n",task);
				// std::cout << "Task : "+ task  << std::endl;
			}

			found2 = task.find("  grasp the ",0);
			if (found3 != std::string::npos){
				room_msg = task.substr(0,found2);
				// rooms.push_back(room);
				// printf("room %s \n",room_msg);
				//  std::cout << "room : "+room_msg  << std::endl;
			}


			found3 = task.find(" and come back here",found);
			if (found3 != std::string::npos){
				object_msg = task.substr(found2+12,found3-found2-12);
				//buf = buf.substr(found+1);
				//  objects.push_back(object);
				// cout << "object"+object_msg  ;
				// std::cout << "object : "+object_msg  << std::endl;
				// printf("object %s \n",object_msg);
			}


			//  std::cout << "The moderator : " + room_msg  + "  and " +  object_msg + "|" << std::endl;
			if(room_msg == "living room" ){

				m_roomState = 3;
				if(object_msg == "apple" )
					{
						m_pointedObject = "apple_0";
					}
				if(object_msg == "can" )
					{
						m_pointedObject = "can_0";
					}
				if(object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_0";
					}

				if(object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_0";
					}
			}


			// lobby message
			if(room_msg == "bed room" ){

				m_roomState = 2;
				if(object_msg == "apple" )
					{
						m_pointedObject = "apple_3";
					}
				if(object_msg == "can" )
					{
						m_pointedObject = "can_3";
					}
				if(object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_3";
					}

				if(object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_3";
					}
			}

			if(room_msg == "lobby" ){
				if(object_msg == "apple" )
					{
						m_pointedObject = "apple_1";
					}
				if(object_msg == "can" )
					{
						m_pointedObject = "can_1";
					}
				if(object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_1";
					}

				if(object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_1";
					}

				m_roomState = 1;
			}

			if(room_msg == "kitchen" ){

				m_roomState = 0;

				if(object_msg == "apple" )
					{
						m_pointedObject = "apple_2";
					}
				if(object_msg == "can" )
					{
						m_pointedObject = "can_2";
					}
				if(object_msg == "mugcup" )
					{
						m_pointedObject = "mugcup_2";
					}

				if(object_msg == "petbottle" )
					{
						m_pointedObject = "petbottle_2";
					}
			}
		}




	if (((msg=="Room_reached") || (msg=="room_reached")) && sender == "robot_000")
		{
			onCheckRoom();
		}

	if (((msg=="Object_grasped") || (msg=="object_grasped")) && sender == "robot_000")
		{
			onCheckObject();
			grasping = true;
		}
	if (((msg=="Object_released") || (msg=="object_released")) && sender == "robot_000")
		{
			grasping = false;
		}
	if(msg == "Start grasping process")
		{
			std:: string name;
			//name = m_pointedObject;
			SimObj *trash = getObj(m_pointedObject.c_str());
			// get trash's position
			trash->getPosition(Obj_pos);
			PrObj_pos = Obj_pos;
			unable_collision = true;
		}

    if(msg == "End grasping process")
		{
			unable_collision = false;
		}



		if (sender == "robot_000" && msg == "Task_finished") {
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		LOG_MSG(("before onCheckPositionfronHuman"));
		onCheckPositionfrontHuman();
		LOG_MSG(("after onCheckPositionfronHuman"));
		sleep(1);
		LOG_MSG(("before reposObjects"));
		//reposObjects();
		LOG_MSG(("after reposObjects"));
		startTime = 0.0;
        Task_st = true;
		breakTask();
	}

	if( (sender == "robot_000" || sender == "RoboCupReferee")  && msg == "Give_up")
	{
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		LOG_MSG(("before reposObjects"));
		//reposObjects();
		LOG_MSG(("after reposObjects"));
		startTime = 0.0;
		breakTask();
	}
	
	if(msg == "init_time")
		{
			init = true;
		}
}


void MyController::onCheckCollision(){
}

void MyController::onCheckPositionfrontHuman()
{
	Vector3d rob_pos;
	Vector3d hum_pos;
	Vector3d dist;
	double distance;
	distance = 0;
	std::string avat;
	std::string rob;
	std::string final;
	final = "Front Human";
	avat = "man_000";
	rob = "robot_000";
	rob_pos =  Vector3d(0, 0, 0);
	hum_pos =  Vector3d(0, 0, 0);
	dist =  Vector3d(0, 0, 0);
	SimObj *avatar = getObj(avat.c_str());
	SimObj *robot = getObj(rob.c_str());

	// get trash's position
	robot->getPosition(rob_pos);
	avatar ->getPosition(hum_pos);
	dist = rob_pos;
	dist -= hum_pos;
	distance = dist.length();
	printf("The distance to human is %f\n", distance);
	if(distance <  radius )
		{
			std::string msg = "RoboCupReferee/Robot is in [" + final + "]" "/+400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}

		}
}


void MyController::onCheckFinalPosition()
{
}


void MyController::onCheckObject()
{
	std:: string name;
	name = m_pointedObject;
	SimObj *trash = getObj(name.c_str());
	// get trash's position
	trash->getPosition(Obj_pos);

	if(Obj_pos.x()== PrObj_pos.x() && Obj_pos.y()== PrObj_pos.y() && Obj_pos.z()== PrObj_pos.z())
		{

		}
	else
		{
			std::string msg = "RoboCupReferee/Robot is in [" + m_pointedObject + "]" "/+400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}

}

void MyController::onCheckRoom()
{
	int x, z;
	int num = 4;
	x =crrPos.x();
	z =crrPos.z();

	LOG_MSG(("Check robot position, select from 4 rooms"));
	if(x>-100&&x<500&&z>-425&&z<75)	// living room
		{ // bed room
			num=0;
			if(m_roomState==3){
				std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]" "/+400";
				m_roomState=0;
				if(m_ref != NULL){
					m_ref->sendMsgToSrv(msg.c_str());
				}
				else{
					LOG_MSG((msg.c_str()));
				}
			}
		}
	else if(x>100&&x<500&&z>75&&z<425){ // kitchen
		num=1;
		if(m_roomState==0){
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]" "/+400";
			m_roomState=1;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else if(x>-500&&x<-100&&z>-425&&z<-75){ // lobby
		num=2;
		if(m_roomState==1){
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]" "/+400";
			m_roomState=2;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else if(x>-500&&x<-100&&z>75&&z<425){ // bed room
		num=3;
		if(m_roomState==2){
			std::string msg = "RoboCupReferee/Robot is in [" + m_rooms[num] + "]" "/+400";
			m_roomState=3;
			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	}
	else {
		LOG_MSG(("Robot is not staying at any room"));
	}

}


/*
  check if the position, posObj, where we want to put the object, obj, is
  available compared to the already placed objects contained in the vector, vec
*/
bool MyController::checkAvailablePos(float posObj, Target obj, int indPosOnTable, std::vector<Target> vec)
{
	bool available = true;

	if(vec.size() > 0){
		if( indPosOnTable == DOWN || indPosOnTable == UP ){
			for(std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it){
				float xInf = it->x - 0.5 * it->length - 20;
				float xSup = it->x + 0.5 * it->length + 20;
				float xInfObj = posObj - 0.5 * obj.length;
				float xSupObj = posObj + 0.5 * obj.length;

				available = (xInfObj < xInf && xSupObj < xInf ||
														 xInfObj > xSup && xSupObj > xSup);
			}
		}
		else{
			for(std::vector<Target>::iterator it = vec.begin(); it != vec.end() && available; ++it){
				float zInf = it->z - 0.5 * it->width - 20;
				float zSup = it->z + 0.5 * it->width + 20;
				float zInfObj = posObj - 0.5 * obj.width;
				float zSupObj = posObj + 0.5 * obj.width;

				available = (zInfObj < zInf && zSupObj < zInf ||
														 zInfObj > zSup && zSupObj > zSup);
			}
		}
	}

	return available;
}



int MyController::getNextPosOnTable(int indPosOnTable, Table table){
	int j;
	for(j=(indPosOnTable+1) % 4; j != indPosOnTable && table.reachable[j] == -1; j = (j + 1) % 4);

	j--;

	if(j != indPosOnTable){
		if(j == 0)
			j = 3;
		else
			j--;
	}

	else
		j = -1;

	return j;
}


void MyController::getNextTable(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable){
	*indTab = (*indTab + 1) % vecTable.size();
	*table = vecTable[*indTab];

	bool posAvailable = false;

	while( !posAvailable ) {
		*indPosOnTable = rand() % 4;
		posAvailable = table->reachable[*indPosOnTable] != -1;
	}
}

void MyController::performChange(int* indTab, int* indPosOnTable, Table* table, std::vector<Table> vecTable){
	int randChange = rand() % 2; // rand to determine if we look for another position or another table

	if(randChange == 1){
		int j = getNextPosOnTable(*indPosOnTable, *table);

		if(j == -1){
			getNextTable(indTab, indPosOnTable, table, vecTable);
		}

		else{
			*indPosOnTable = j;
		}
	}

	else{
		getNextTable(indTab, indPosOnTable, table, vecTable);
	}
}


void MyController::reposObjects()
{
	for(std::map< std::string, std::vector<Target> >::iterator it = m_targets.begin(); it != m_targets.end(); ++it){
		std::vector<Table> vecTable = m_tables[it->first];
		std::vector<Target> placedObjects;
		int nbTables = vecTable.size();
		float yObj;
		float xObj;
		float zObj;

		for(std::vector<Target>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2){
			int indTab = rand() % nbTables;
			Table table = vecTable[indTab];
			bool posAvailable = false;
			int indPosOnTable;
			int nbTries;

			/*
			  we took a random table among the available ones and now we take a random
			  position on this table (UP, DOWN, LEFT, RIGHT) if it's available
			*/
			while( !posAvailable ) {
				indPosOnTable = rand() % 4;
				posAvailable = table.reachable[indPosOnTable] != -1;
			}

			/*
			  This loop is made mainly to avoid placing different objects at the same
			  spot. If we try to place the object 10 times and it doesn't succeed in
			  finding an available spot then we try to find another position on the
			  current table or change the table directly
			*/

			do{
				nbTries = 0;

				if( indPosOnTable == DOWN || indPosOnTable == UP ){
					yObj = table.y + 0.5 * table.height + 0.5 * it2->height + 1.75;
					float xOffset = 10;
					float xInf = table.x - 0.5 * table.length + xOffset;
					float xSup = table.x + 0.5 * table.length - xOffset;
					float zOffset = it2->width;

					do{
						xObj = mapRange(rand(), 0,  RAND_MAX, xInf, xSup);
						nbTries++;
					} while(!checkAvailablePos(xObj, *it2, indPosOnTable, placedObjects) && nbTries < NUMBER_OF_REPETITION);

					if(nbTries >= 9){
						performChange(&indTab, &indPosOnTable, &table, vecTable);
					}

					else{

						if(indPosOnTable == DOWN){
							zObj = table.z + 0.5 * table.width - zOffset;
						}

						else{
							zObj = table.z - 0.5 * table.width + zOffset;
						}
					}
				}

				else{
					yObj = table.y + 0.5 * table.height + 0.5 * it2->height + 1.75;
					float zOffset = 10;
					float zSup = table.z - 0.5 * table.width + zOffset;
					float zInf = table.z + 0.5 * table.width - zOffset;
					float xOffset = it2->length;

					do{
						zObj = mapRange(rand(), 0,  RAND_MAX, zInf, zSup);
						nbTries++;
					} while(!checkAvailablePos(zObj, *it2, indPosOnTable, placedObjects) && nbTries < NUMBER_OF_REPETITION);

					if(nbTries >= 9){
						performChange(&indTab, &indPosOnTable, &table, vecTable);
					}

					else{
						if(indPosOnTable == RIGHT){
							xObj = table.x + 0.5 * table.length - xOffset;
						}

						else{
							xObj = table.x - 0.5 * table.length + xOffset;
						}
					}
				}
			}while(nbTries >= 9);

			SimObj* target = getObj(it2->name.c_str());
			target->setPosition(Vector3d(xObj, yObj, zObj));
			Rotation rot;
			rot.setQuaternion(1, 0, 0, 0);
            target->setRotation(rot);
			it2->x = xObj;
			it2->y = yObj;
			it2->z = zObj;

			placedObjects.push_back(*it2);
		}

		placedObjects.clear();
	}

	//reset robot position
	RobotObj* robot = getRobotObj(roboName.c_str());

     if(grasping == true)
		 {
            CParts * parts = robot->getParts("LARM_LINK7");
            parts->releaseObj();
            grasping = false;

		 }
		 robot->setWheelVelocity(0.0,0.0);
		 robot->setPosition(robotInitialPos);
		 Rotation rot;
		 rot.setQuaternion(1, 0, 0, 0);
         robot->setRotation(rot);
         robot->setJointVelocity("LARM_JOINT0", 0.0,0.0);
         robot->setJointAngle("LARM_JOINT0", 0.0);
		 robot->setJointVelocity("LARM_JOINT1", 0.0,0.0);
		 robot->setJointAngle("LARM_JOINT1", 0.0);
		 robot->setJointVelocity("LARM_JOINT3", 0.0,0.0);
		 robot->setJointAngle("LARM_JOINT3", 0.0);
		 robot->setJointVelocity("LARM_JOINT4", 0.0,0.0);
		 robot->setJointAngle("LARM_JOINT4", 0.0);
		 robot->setJointVelocity("LARM_JOINT5", 0.0,0.0);
		 robot->setJointAngle("LARM_JOINT5", 0.0);
         robot->setJointVelocity("LARM_JOINT6", 0.0,0.0);
         robot->setJointAngle("LARM_JOINT6", 0.0);
         robot->setJointVelocity("LARM_JOINT7", 0.0,0.0);
         robot->setJointAngle("LARM_JOINT7", 0.0);
         
         trialCount++;
         std::cout << "trial count is  " << trialCount << std::endl;
         
}


void MyController::breakTask()
{
	LOG_MSG(("start of breakTask"));
	isCleaningUp = false;
	//takeAwayObjects();
	//trialCount++;
	std::string msg = "RoboCupReferee/reset/";
	if(m_ref != NULL){
		// m_ref->sendMsgToSrv(msg.c_str());
		 SimObj *robot;
		 robot = this->getObj("robot_000");


	}
if(trialCount < NUMBER_OF_REPETITION)
{
		 broadcastMsg("Task_end");
		 reposObjects();
}
	if(trialCount >= NUMBER_OF_REPETITION) {
		// resetRobotCondition();
		LOG_MSG(("Mission_complete"));
		broadcastMsg("Mission_complete");
		if(m_ref != NULL){
			msg = "RoboCupReferee/time/- END -";
            m_ref->sendMsgToSrv(msg.c_str());
		}
		Task_st = false;
		time_display = false;
	}
	else {
		Task_st = true;
	}
	LOG_MSG(("end of breakTask"));
}


void MyController::initRoomsObjects()
{
	std::vector<Table> vec;
	Table tab;
	std::vector<Target> vec2;
	Target tar;
    Vector3d posf;
    SimObj* entity ;
    /*
	tab.name = "Buffet1";
	tab.length = 32.1;
	tab.width  = 54.2;
	tab.height = 59.5;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = 1;
	tab.reachable[RIGHT] = 1;
	tab.reachable[LEFT]  = -1;
	entity = getObj(tab.name.c_str());
	entity->getPosition(posf);
	tab.x = posf.x()+10;
	tab.y = posf.y();
	tab.z = posf.z();
	vec.push_back(tab);
    
	tab.name = "Dinner table1";
	tab.length = 135;
	tab.width  = 75.9;
	tab.height = 59.2;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = 1;
	tab.reachable[RIGHT] = 1;
	tab.reachable[LEFT]  = 1;

	vec.push_back(tab);
     */
	tab.name = "Couch_table1";
	tab.length = 128;
	tab.width  = 65.8;
	tab.height = 59.8;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = 1;
	tab.reachable[RIGHT] = 1;
	tab.reachable[LEFT]  = 1;

	vec.push_back(tab);

	tar.name = "apple_0";
	tar.length = 6.51;
	tar.width  = 6.86;
	tar.height = 7.75;

	vec2.push_back(tar);

	tar.name = "petbottle_0";
	tar.length = 7.11;
	tar.width  = 7.11;
	tar.height = 22.3;

	vec2.push_back(tar);

	tar.name = "can_0";
	tar.length = 5.14;
	tar.width  = 5.14;
	tar.height = 9.07;

	vec2.push_back(tar);

	tar.name = "mugcup_0";
	tar.length = 11.7;
	tar.width  = 11.7;
	tar.height = 7.98;

	vec2.push_back(tar);

	m_tables["livingroom"] = vec;
	m_targets["livingroom"] = vec2;

	vec.clear();
	vec2.clear();

	tab.name = "Buffet2";
	tab.length = 32.1;
	tab.width  = 54.2;
	tab.height = 59.5;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = 1;
	tab.reachable[RIGHT] = -1;
	tab.reachable[LEFT]  = 1;
	entity = getObj(tab.name.c_str());
	entity->getPosition(posf);
	tab.x = posf.x()-10;
	tab.y = posf.y();
	tab.z = posf.z();
	vec.push_back(tab);

	tab.name = "Side board1";
	tab.length = 165;
	tab.width  = 50.8;
	tab.height = 59.7;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = -1;
	tab.reachable[RIGHT] = -1;
	tab.reachable[LEFT]  = 1;
	entity = getObj(tab.name.c_str());
	entity->getPosition(posf);
	tab.x = posf.x()-10;
	tab.y = posf.y();
	tab.z = posf.z();
	vec.push_back(tab);

	tar.name = "apple_1";
	tar.length = 6.51;
	tar.width  = 6.86;
	tar.height = 7.75;

	vec2.push_back(tar);

	tar.name = "petbottle_1";
	tar.length = 7.11;
	tar.width  = 7.11;
	tar.height = 22.3;

	vec2.push_back(tar);

	tar.name = "can_1";
	tar.length = 5.14;
	tar.width  = 5.14;
	tar.height = 9.07;

	vec2.push_back(tar);

	tar.name = "mugcup_1";
	tar.length = 11.7;
	tar.width  = 11.7;
	tar.height = 7.98;

	vec2.push_back(tar);

	m_tables["lobby"] = vec;
	m_targets["lobby"] = vec2;

	vec.clear();
	vec2.clear();

	tab.name = "Kitchen Table1";
	tab.length = 140;
	tab.width  = 85;
	tab.height = 59.4;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = 1;
	tab.reachable[RIGHT] = 1;
	tab.reachable[LEFT]  = 1;

	vec.push_back(tab);

	tar.name = "apple_2";
	tar.length = 6.51;
	tar.width  = 6.86;
	tar.height = 7.75;

	vec2.push_back(tar);

	tar.name = "petbottle_2";
	tar.length = 7.11;
	tar.width  = 7.11;
	tar.height = 22.3;

	vec2.push_back(tar);

	tar.name = "can_2";
	tar.length = 5.14;
	tar.width  = 5.14;
	tar.height = 9.07;

	vec2.push_back(tar);

	tar.name = "mugcup_2";
	tar.length = 11.7;
	tar.width  = 11.7;
	tar.height = 7.98;

	vec2.push_back(tar);

	m_tables["kitchen"] = vec;
	m_targets["kitchen"] = vec2;

	vec.clear();
	vec2.clear();

	tab.name = "Side table1";
	tab.length = 32.1;
	tab.width  = 34.2;
	tab.height = 59.5;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = -1;
	tab.reachable[RIGHT] = -1;
	tab.reachable[LEFT]  = -1;
	entity = getObj(tab.name.c_str());
	entity->getPosition(posf);

    tab.x = posf.x()-10;
	tab.y = posf.y();
	tab.z = posf.z()-10;
	vec.push_back(tab);

	tab.name = "Side board2";
	tab.length = 167;
	tab.width  = 24.9;
	tab.height = 58.8;
	tab.reachable[UP]    = 1;
	tab.reachable[DOWN]  = -1;
	tab.reachable[RIGHT] = -1;
	tab.reachable[LEFT]  = -1;
	entity = getObj(tab.name.c_str());
	entity->getPosition(posf);
	tab.x = posf.x()+10;
	tab.y = posf.y();
	tab.z = posf.z()-10;
	vec.push_back(tab);

	tar.name = "apple_3";
	tar.length = 6.51;
	tar.width  = 6.86;
	tar.height = 7.75;

	vec2.push_back(tar);

	tar.name = "petbottle_3";
	tar.length = 7.11;
	tar.width  = 7.11;
	tar.height = 22.3;

	vec2.push_back(tar);

	tar.name = "can_3";
	tar.length = 5.14;
	tar.width  = 5.14;
	tar.height = 9.07;

	vec2.push_back(tar);

	tar.name = "mugcup_3";
	tar.length = 11.7;
	tar.width  = 11.7;
	tar.height = 7.98;

	vec2.push_back(tar);

	m_tables["bedroom"] = vec;
	m_targets["bedroom"] = vec2;
}
bool MyController::checkRobotFinished(){
	Vector3d posR;
	getObj(roboName.c_str())->getPosition(posR);

	return pow( ( posR.x() - humanPos.x() ), 2) + pow( ( posR.z() - humanPos.z() ), 2) < (radius * radius);
}

template<typename Type>
int MyController::contains(std::vector<Type> vec, Type key){
	bool found = false;
	int i;
	for(i = 0 ; i < vec.size() && !found; i++){
		found = vec[i] == key;
	}

	if(!found)
		i = -1;
	else
		i--;

	return i;
}

int MyController::contains(std::vector<Target> vec, std::string key){
	bool found = false;
	int i;
	for(i = 0 ; i < vec.size() && !found; i++){
		found = vec[i].name == key;
	}

	if(!found)
		i = -1;
	else
		i--;

	return i;
}

int MyController::contains(std::vector<Table> vec, std::string key){
	bool found = false;
	int i;
	for(i = 0 ; i < vec.size() && !found; i++){
		found = vec[i].name == key;
	}

	if(!found)
		i = -1;
	else
		i--;

	return i;
}
float MyController::mapRange(float s, float a1, float a2, float b1, float b2){
	return b1 + ( (s-a1) * (b2-b1) ) / (a2 - a1);
}
/*
  void MyController::takeAwayObjects(){
  for(std::vector<Target>::iterator it = m_entities[trialCount].begin(); it != m_entities[trialCount].end(); ++it){
  SimObj* target = getObj(it->name.c_str());
  target->setPosition(Vector3d(100000, 100000, 100000));
  }
  }
*/

extern "C" Controller * createController() {
	return new MyController;
}
