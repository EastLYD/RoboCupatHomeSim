#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <map>
#include <time.h> 
//#include  <random>
//#include  <iterator>

#define NUMBER_OF_REPETITION   10 /// 10
#define MAX_CHARS_PER_LINE 512



class MyController : public Controller {
public:
	void   onInit(InitEvent &evt);
	double onAction(ActionEvent&);

	void   onCheckObject();
	void   onRecvMsg(RecvMsgEvent &evt);
    void   onCheckCollision();
	void   breakTask();
    void   parseFile(const std::string fileNam_my);
    void   PlaceThings();
    void   CheckObjects();
    void   CheckTrashes();
    void   InitRobot();
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

    typedef std::vector  < std::string >  Location;
	std::map < std::string, Location > File_List;


	Vector3d robotInitialPos;
	Vector3d humanPos;
	float radius;
	std::string File_ID;

	std::string Object_Position;
	std::string Trash_Position;
	int Location_Status[2];

double Y_PetteBotle ,Y_Mug ,Y_Can  ,Y_General_trash ;


Vector3d m_Object_Right;
Vector3d m_Object_Center;
Vector3d m_Object_Left;
Vector3d m_RobotPos;
Vector3d m_Table;

Vector3d m_Trash_Right;
Vector3d m_Trash_Center;
Vector3d m_Trash_Left;


struct Objects_Coordinates {
  Vector3d Coord;
  std::string Object;
  bool On;
} ;

struct Trashes_Coordinates {
  Vector3d Coord;
  std::string Trash;
  bool On;
} ;

std::vector<Objects_Coordinates> Cm_Objects;
std::vector<Trashes_Coordinates> Cm_trashes;

std::vector<std::string> m_trashes;
	// ゴミ箱オブジェクト
std::vector<std::string> m_trashboxs;

Rotation rot;


};



void MyController::parseFile(const std::string fileNam_my)
{

std::ifstream fin;
	fin.open(fileNam_my.c_str());
	bool dest = false;
	//std::string On_table, Trash_Box;
	std::size_t found=0;
	std::size_t found2=0;
	std::size_t found3=0;
	std::string Number;
	std::string On_table;
	std::string Trash_Box;
	Location  elements;
	while(!fin.eof()){
		//std::map < int, std::pair<std::string, std::string> > One_File;
       //MapType One_File;
		std::size_t found=0;
		std::size_t found2=0;
		char Cbuf[MAX_CHARS_PER_LINE];
		fin.getline(Cbuf, MAX_CHARS_PER_LINE);
		std::string buf = std::string(Cbuf);


		found = buf.find(":",found2);
		if (found != std::string::npos){
			Number = buf.substr(found2,found-found2);
		//	rooms.push_back(On_table);
		}


		found2 = buf.find(",",found);
		if (found2 != std::string::npos){
			On_table = buf.substr(found+1,found2-found-1);
			//buf = buf.substr(found+1);
		//	objects.push_back(object);
			// printf("found2 %d",found2);
			elements.push_back(On_table);
		}
		found3 = buf.find(".",found2);
		if (found2 != std::string::npos){
			Trash_Box = buf.substr(found2+1,found3-found2-1);
			//buf = buf.substr(found+1);
		//	objects.push_back(object);
			// printf("found2 %d",found2);
		  elements.push_back(Trash_Box);
		}
		    File_List[Number]=elements;
		Number = "";
		elements.clear();
	}


}

 void   MyController::InitRobot()
 {
 	RobotObj *r_my = getRobotObj(roboName.c_str());
			  r_my->setWheelVelocity(0.0,0.0);
			  r_my->setPosition(m_RobotPos);

			  r_my->setRotation(rot);
			  r_my->setJointVelocity("LARM_JOINT0", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT0", 0.0);
			  r_my->setJointVelocity("LARM_JOINT1", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT1", 0.0);
			  r_my->setJointVelocity("LARM_JOINT3", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT3", 0.0);
			  r_my->setJointVelocity("LARM_JOINT4", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT4", -1.57);
			  r_my->setJointVelocity("LARM_JOINT5", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT5", 0.0);
			  r_my->setJointVelocity("LARM_JOINT6", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT6", 0.0);
			  r_my->setJointVelocity("LARM_JOINT7", 0.0,0.0);
			  r_my->setJointAngle("LARM_JOINT7", 0.0);

			  r_my->setJointVelocity("RARM_JOINT0", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT0", 0.0);
			  r_my->setJointVelocity("RARM_JOINT1", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT1", 0.0);
			  r_my->setJointVelocity("RARM_JOINT3", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT3", 0.0);
			  r_my->setJointVelocity("RARM_JOINT4", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT4", -1.57);
			  r_my->setJointVelocity("RARM_JOINT5", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT5", 0.0);
			  r_my->setJointVelocity("RARM_JOINT6", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT6", 0.0);
			  r_my->setJointVelocity("RARM_JOINT7", 0.0,0.0);
			  r_my->setJointAngle("RARM_JOINT7", 0.0);

 }



 void   MyController::PlaceThings()
{
Rotation rot;
rot.setQuaternion(1, 0, 0, 0);
	SimObj *target_Obj1 = this->getObj("petbottle");
	SimObj *target_Obj2 = this->getObj("mugcup");
	SimObj *target_Obj3 = this->getObj("can");

	SimObj *target_trash1 = this->getObj("recycle");
	SimObj *target_trash2 = this->getObj("burnable");
	SimObj *target_trash3 = this->getObj("unburnable");



////////////////////////    Object Placement  ////////////////////////////

			int pos_OBJ = rand() % 3;
			Cm_Objects[pos_OBJ].Coord.y(Y_PetteBotle);
			target_Obj1->setPosition(Cm_Objects[pos_OBJ].Coord);
			Cm_Objects[pos_OBJ].Object = "petbottle";
			target_Obj1->setRotation(rot);
            Cm_Objects[pos_OBJ].On = true;



			bool choice_Obj = true;
			while (choice_Obj)
			{
			pos_OBJ = rand() % 3;
							if(Cm_Objects[pos_OBJ].On == false)
							{
							Cm_Objects[pos_OBJ].Coord.y(Y_Mug);
							target_Obj2->setPosition(Cm_Objects[pos_OBJ].Coord);
							Cm_Objects[pos_OBJ].Object = "mugcup";
							target_Obj2->setRotation(rot);
							Cm_Objects[pos_OBJ].On = true;
							 choice_Obj = false;
							}
			}

				for (int i= 0 ; i < Cm_Objects.size(); i++ )

				{
							if(Cm_Objects[i].On == false)
							{
							Cm_Objects[pos_OBJ].Coord.y(Y_Can);
						    target_Obj3->setPosition(Cm_Objects[i].Coord);
						    Cm_Objects[i].Object = "mugcup";
						    target_Obj3->setRotation(rot);
						    Cm_Objects[i].On = true;
							}
				}

////////////////////////////////////////////////////////////////////////////




/////////////////////  Trash Placement /////////////////////////////////////


			int pos_TRASH = rand() % 3;
            Cm_trashes[pos_TRASH].Coord.y(Y_General_trash);
			target_trash1->setPosition(Cm_trashes[pos_TRASH].Coord);
			Cm_trashes[pos_TRASH].Trash = "recycle";
			target_trash1->setRotation(rot);
            Cm_trashes[pos_TRASH].On = true;



			bool choice_Trash = true;
			while (choice_Trash)
			{
			pos_TRASH = rand() % 3;
							if(Cm_trashes[pos_TRASH].On == false)
							{
							Cm_trashes[pos_TRASH].Coord.y(Y_General_trash);
							target_trash2->setPosition(Cm_trashes[pos_TRASH].Coord);
							Cm_trashes[pos_TRASH].Trash = "burnable";
							target_trash2->setRotation(rot);
							Cm_trashes[pos_TRASH].On = true;
							 choice_Trash = false;
							}
			}


				for (int i= 0 ; i < Cm_trashes.size(); i++ )

				{
							if(Cm_trashes[i].On == false)
							{
							Cm_trashes[pos_TRASH].Coord.y(Y_General_trash);
						    target_trash3->setPosition(Cm_trashes[i].Coord);
						    Cm_trashes[i].Trash = "unburnable";
						    target_trash3->setRotation(rot);
						    Cm_trashes[i].On = true;
							}
				}
//////////////////////////////////////////////////////////////////////////////



trialCount++;
}



void MyController::onInit(InitEvent &evt)
{
   parseFile("File_Position.dat");

  m_Object_Right = Vector3d(-150, 59.15, -190.0);
  m_Object_Center = Vector3d(-150.0, 52.15, -60.0);
  m_Object_Left = Vector3d(-150.0, 54.25, 70.0);


m_Trash_Right = Vector3d(260.0, 36.35, -180.0);
m_Trash_Center = Vector3d(140.0, 36.35, -180.0);
m_Trash_Left = Vector3d(-120.0, 36.35, -180.0);


Y_PetteBotle = 59.15;
Y_Mug = 52.15;
Y_Can = 54.25;
Y_General_trash = 36.35 ;


m_RobotPos = Vector3d(100.0, 30.0,-100.0);

Objects_Coordinates m_Object_Right_c;
Objects_Coordinates m_Object_Center_c;
Objects_Coordinates m_Object_Left_c;




m_Object_Right_c.Coord = m_Object_Right;
m_Object_Right_c.On = false;
Cm_Objects.push_back(m_Object_Right_c);

m_Object_Center_c.Coord = m_Object_Center;
m_Object_Center_c.On = false;
Cm_Objects.push_back(m_Object_Center_c);

m_Object_Left_c.Coord = m_Object_Left;
m_Object_Left_c.On = false;
Cm_Objects.push_back(m_Object_Left_c);





Trashes_Coordinates m_Trash_Right_c;
Trashes_Coordinates m_Trash_Center_c;
Trashes_Coordinates m_Trash_Left_c;

m_Trash_Right_c.Coord = m_Trash_Right;
m_Trash_Right_c.On = false;
Cm_trashes.push_back(m_Trash_Right_c);

m_Trash_Center_c.Coord = m_Trash_Center;
m_Trash_Center_c.On = false;
Cm_trashes.push_back(m_Trash_Center_c);


m_Trash_Left_c.Coord = m_Trash_Left;
m_Trash_Left_c.On = false;
Cm_trashes.push_back(m_Trash_Left_c);


  m_RobotPos = Vector3d(100.0, 30.0,-100.0);
  m_Table = Vector3d(0.0, 24.0,70.0);

rot.setQuaternion(1, 0, 0, 0);



	m_trashes.push_back("petbottle");
	m_trashes.push_back("mugcup");
	m_trashes.push_back("can");

	// ゴミ箱登録
	m_trashboxs.push_back("recycle");
	m_trashboxs.push_back("burnable");
	m_trashboxs.push_back("unburnable");






	m_roomState = 6;

	m_pointedObject = "";

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




	entNum = m_entNames.size();
	srand (2);

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

	int strPos1 = 0;
	int strPos2;
	int strPos3;
	std::string headss;
	std::string ss = all_msg;

	strPos2 = ss.find(":", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);
if (headss == "File") {
		// Contol of body movement by KINECT
		//moveBodyByKINECT(all_msg);

    strPos3 = ss.find(".", strPos2+1);
	File_ID.assign(ss, strPos2+1, strPos3);

	}

	//std::ifstream fin;
	// fin.open(fileNam_my.c_str());
	bool dest = false;

	if(sender == "man_000")
		{


	if(msg == "Start_motion")
		{
			clock_t t;
			t = clock();
			init = true;
                        srand(t);
						std::map < std::string, Location >::iterator it = File_List.begin();
						std::advance(it, rand() % File_List.size());

						 std::string File_name = "Send_";
						  File_name+= it->first;
						  Location Curent_Locations = it->second;
						 // Object_Position = Curent_Locations[0];
						 // Trash_Position =  Curent_Locations[1];
										for(int i =0;i<Curent_Locations.size();i++)
										{
										  if( Curent_Locations[i] == "Right")
										  {
										Location_Status[i] = 0;
										  }
										  if(Curent_Locations[i] == "Center")
										  {
										Location_Status[i] = 1;
										  }
										    if(Curent_Locations[i] == "Left")
										  {
										Location_Status[i] = 2;
										  }
										}


       sendMsg("SIGKINECT", File_name);

		}
}


if(msg == "Object_Grasped")
		{
 CheckObjects();

		}


if(msg == "Object_Trashed")
		{
 CheckTrashes();
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



void  MyController::CheckObjects()
{

//Cm_Objects[Location_Status[0]].Coord
//Object_Position
//Location_Status[0]

std:: string name;
	name = Cm_Objects[Location_Status[0]].Object;
	SimObj *Obj = getObj(name.c_str());
	// get trash's position
	Vector3d Obj_pos;
	Obj->getPosition(Obj_pos);

	if(Obj_pos.x()== Cm_Objects[Location_Status[0]].Coord.x() && Obj_pos.y()== Cm_Objects[Location_Status[0]].Coord.y() && Obj_pos.z()== Cm_Objects[Location_Status[0]].Coord.z())
		{
			std::string msg = "RoboCupReferee/Robot is in ["  "]" "/-400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}


		}
	else
		{
			std::string msg = "RoboCupReferee/Robot is in [" + Cm_Objects[Location_Status[0]].Object + "]" "/+400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}




}


void  MyController::CheckTrashes()
{


    std:: string name;
	name = Cm_trashes[Location_Status[1]].Trash;
	SimObj *Trash = getObj(name.c_str());
	// get trash's position
	Vector3d Tr_pos;
	Trash->getPosition(Tr_pos);

	if(Tr_pos.x()== Cm_trashes[Location_Status[1]].Coord.x() &&  Tr_pos.z()== Cm_trashes[Location_Status[1]].Coord.z())
		{
			std::string msg = "RoboCupReferee/Robot is in ["  "]" "/-400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}
	else
		{
			std::string msg = "RoboCupReferee/Robot is in [" + Cm_trashes[Location_Status[1]].Trash + "]" "/+400";

			if(m_ref != NULL){
				m_ref->sendMsgToSrv(msg.c_str());
			}
			else{
				LOG_MSG((msg.c_str()));
			}
		}

}

void MyController::onCheckCollision(){
}



/*
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

*/


/*
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
*/

/*
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

*/

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
		// reposObjects();
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
