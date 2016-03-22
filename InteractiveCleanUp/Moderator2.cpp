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

#define NUMBER_OF_REPETITION   41
#define MAX_CHARS_PER_LINE 512

#define TRIAL_NUMBER_TEXT_FILE_NAME "trialnum.txt"

#define ALLOW_ADD_SCORE_WITHOUT_THROW_AWAY true
#define ADD_SCORE_GRASP_RIGHT_OBJECT	+400
#define ADD_SCORE_GRASP_WRONG_OBJECT	-400
#define ADD_SCORE_RIGHT_OBJECT_IN_RIGHT_TRASH_BOX	+600
#define ADD_SCORE_RIGHT_OBJECT_IN_WRONG_TRASH_BOX	-200
#define ADD_SCORE_WRONG_OBJECT_IN_RIGHT_TRASH_BOX	0
#define ADD_SCORE_WRONG_OBJECT_IN_WRONG_TRASH_BOX	0

#define SRAND_INITIAL_NUMBER 2
#define SRAND_FACTOR 5

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
	void   CheckTrasheBoxs();
	void   InitRobot();
	// void takeAwayObjects();

private:
	BaseService *m_ref;		// Referee service
	double retValue;		// Refresh rate of the modification
	bool   colState;		// Collision state
	bool   pcolState;		// Collision state
	std::string roboName;	// Robot's name
	std::string mdName;		// Moderator's name
	std::string humanName;	// Human's name
	std::string ktName;		// kinect's name
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

	Vector3d m_TrashBox_Right;
	Vector3d m_TrashBox_Center;
	Vector3d m_TrashBox_Left;


	struct Objects_Coordinates {
		Vector3d Coord;
		std::string ObjectName;
		bool On;
	};

	struct TrashBoxs_Coordinates {
		Vector3d Coord;
		std::string TrashBoxName;
		bool On;
	};

	std::vector<Objects_Coordinates> Cm_Objects;
	std::vector<TrashBoxs_Coordinates> Cm_trashBoxs;

	std::vector<std::string> m_objects;
		// ゴミ箱オブジェクト
	std::vector<std::string> m_trashboxs;

	Rotation rot;

	typedef std::map<std::string, double> JointMap;
	JointMap initialJointMap;
	Rotation initialRotation;
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

void MyController::InitRobot()
{
	RobotObj *r_my = getRobotObj(roboName.c_str());
	bool allJointInitialized = false;

	CParts * lparts = r_my->getParts("LARM_LINK7");
	lparts->releaseObj();
	CParts * rparts = r_my->getParts("RARM_LINK7");
	rparts->releaseObj();

	r_my->setWheelVelocity(0.0,0.0);
	r_my->setPosition(m_RobotPos);

	while(!allJointInitialized)
	{
		allJointInitialized = true;
		JointMap::iterator initialJointMap_iterator = initialJointMap.begin();
		JointMap currentJointMap = r_my->getAllJointAngles();
		JointMap::iterator currentJointMap_iterator = currentJointMap.begin();

		while(initialJointMap_iterator != initialJointMap.end())
		{
			std::string initialJointName = (*initialJointMap_iterator).first;
			//LOG_MSG(("<debug> \"%s\" is Joint.", initialJointName.c_str()));
			if(	(*initialJointMap_iterator).first != (*currentJointMap_iterator).first
				|| (*initialJointMap_iterator).second != (*currentJointMap_iterator).second)
			{
				initialJointName = (*initialJointMap_iterator).first;
				double initialJointAngle = (*initialJointMap_iterator).second;
				r_my->setJointVelocity(initialJointName.c_str(), 0.0, 0.0);
				r_my->setJointAngle(initialJointName.c_str(), initialJointAngle);
				//LOG_MSG(("<debug> \"%s\" is initialized.", initialJointName.c_str()));
				allJointInitialized = false;
			}
			initialJointMap_iterator++;
			currentJointMap_iterator++;
			Rotation myRotation;
			r_my->getRotation(myRotation);
			if( 	myRotation.qw() != initialRotation.qw()
				|| myRotation.qx() != initialRotation.qx()
				|| myRotation.qy() != initialRotation.qy()
				|| myRotation.qz() != initialRotation.qz())
			{
				r_my->setRotation(initialRotation);
				r_my->setWheelVelocity(0.0,0.0);
			}
		}
	}
}


void MyController::PlaceThings()
{
	Rotation rot;
	rot.setQuaternion(0.707, 0, 0.707, 0);
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
	Cm_Objects[pos_OBJ].ObjectName = "petbottle";
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
			Cm_Objects[pos_OBJ].ObjectName = "mugcup";
			target_Obj2->setRotation(rot);
			Cm_Objects[pos_OBJ].On = true;
			choice_Obj = false;
		}
	}

	for (int i= 0 ; i < Cm_Objects.size(); i++ )
	{
		if(Cm_Objects[i].On == false)
		{
			Cm_Objects[i].Coord.y(Y_Can);
			target_Obj3->setPosition(Cm_Objects[i].Coord);
			Cm_Objects[i].ObjectName = "can";
			target_Obj3->setRotation(rot);
			Cm_Objects[i].On = true;
		}
	}

////////////////////////////////////////////////////////////////////////////




/////////////////////  Trash Placement /////////////////////////////////////


	int pos_TRASH = rand() % 3;
	Cm_trashBoxs[pos_TRASH].Coord.y(Y_General_trash);
	target_trash1->setPosition(Cm_trashBoxs[pos_TRASH].Coord);
	Cm_trashBoxs[pos_TRASH].TrashBoxName = "recycle";
	target_trash1->setRotation(rot);
	Cm_trashBoxs[pos_TRASH].On = true;



	bool choice_Trash = true;
	while (choice_Trash)
	{
		pos_TRASH = rand() % 3;
		if(Cm_trashBoxs[pos_TRASH].On == false)
		{
			Cm_trashBoxs[pos_TRASH].Coord.y(Y_General_trash);
			target_trash2->setPosition(Cm_trashBoxs[pos_TRASH].Coord);
			Cm_trashBoxs[pos_TRASH].TrashBoxName = "burnable";
			target_trash2->setRotation(rot);
			Cm_trashBoxs[pos_TRASH].On = true;
			choice_Trash = false;
		}
	}


	for (int i= 0 ; i < Cm_trashBoxs.size(); i++ )
	{
		if(Cm_trashBoxs[i].On == false)
		{
			Cm_trashBoxs[pos_TRASH].Coord.y(Y_General_trash);
			target_trash3->setPosition(Cm_trashBoxs[i].Coord);
			Cm_trashBoxs[i].TrashBoxName = "unburnable";
			target_trash3->setRotation(rot);
			Cm_trashBoxs[i].On = true;
		}
	}
//////////////////////////////////////////////////////////////////////////////
	for (int i= 0 ; i < Cm_Objects.size(); i++ )
	{
		Cm_Objects[i].On = false;
	}

	for (int i= 0 ; i < Cm_trashBoxs.size(); i++ )
	{
		Cm_trashBoxs[i].On = false;
	}

}



void MyController::onInit(InitEvent &evt)
{
	parseFile("File_Position.dat");

	m_Object_Right = Vector3d(40, 59.15, 50.0);
	m_Object_Center = Vector3d(0.0, 52.15, 50.0);
	m_Object_Left = Vector3d(-40.0, 54.25, 50.0);


	m_TrashBox_Right = Vector3d(-150.0, 36.35, -190.0);
	m_TrashBox_Center = Vector3d(-150.0, 36.35, -60.0);
	m_TrashBox_Left = Vector3d(-150.0, 36.35, 70.0);


	Y_PetteBotle = 59.55;
	Y_Mug = 53.95;
	Y_Can = 56.5;
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



	TrashBoxs_Coordinates m_TrashBox_Right_c;
	TrashBoxs_Coordinates m_TrashBox_Center_c;
	TrashBoxs_Coordinates m_TrashBox_Left_c;

	m_TrashBox_Right_c.Coord = m_TrashBox_Right;
	m_TrashBox_Right_c.On = false;
	Cm_trashBoxs.push_back(m_TrashBox_Right_c);

	m_TrashBox_Center_c.Coord = m_TrashBox_Center;
	m_TrashBox_Center_c.On = false;
	Cm_trashBoxs.push_back(m_TrashBox_Center_c);


	m_TrashBox_Left_c.Coord = m_TrashBox_Left;
	m_TrashBox_Left_c.On = false;
	Cm_trashBoxs.push_back(m_TrashBox_Left_c);


	m_RobotPos = Vector3d(100.0, 30.0,-100.0);
	m_Table = Vector3d(0.0, 24.0,70.0);

	rot.setQuaternion(1, 0, 0 , 0);



	m_objects.push_back("petbottle");
	m_objects.push_back("mugcup");
	m_objects.push_back("can");

	// ゴミ箱登録
	m_trashboxs.push_back("recycle");
	m_trashboxs.push_back("burnable");
	m_trashboxs.push_back("unburnable");



	m_roomState = 6;

	m_pointedObject = "";

	int i, cnt;
	Task_st = true;
	time_display = true;
	retValue = 0.1;
	colState = false;
	pcolState = false;
	roboName = "robot_000";
	mdName   = "moderator_0";
	humanName = "man_000";
	ktName = "Kinect_000";

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

	isCleaningUp = false;

	startTime =  0.0;
	endTime   = 180.0; // [sec]

	take_time = 0.0;
	init = false;

	cnt = m_entities.size();

	for(i=0;i<cnt;i++) {
		int found;
		SimObj* entity = getObj(m_entities[i].c_str());
		Vector3d pos;
		entity->getPosition(pos);


		if((m_entities[i] != mdName) &&
			(m_entities[i] != ktName) &&
			(m_entities[i] != roboName) &&
			(m_entities[i] != "petbottle") &&
			(m_entities[i] != "mugcup") &&
			(m_entities[i] != "can")  ){
			m_entNames.push_back(m_entities[i]);
			//std::cout << "entitie: " << m_entities[i] << " pushed!" << std::endl;
		}

	}
	entNum = m_entNames.size();

	initialJointMap = getObj(roboName.c_str())->getAllJointAngles();
	getObj(roboName.c_str())->getRotation(initialRotation); 

	std::ifstream trialNumberFile(TRIAL_NUMBER_TEXT_FILE_NAME);
	if(trialNumberFile.fail())
	{
		trialNumberFile.close();
		std::cerr << "\"" << TRIAL_NUMBER_TEXT_FILE_NAME << "\" is can not opend" << std::endl;
		std::cout << "Trial count 0" << std::endl;
		std::ofstream trialNumberOutput;
		trialNumberOutput.open(TRIAL_NUMBER_TEXT_FILE_NAME);
		while(!trialNumberOutput.fail()){
			std::cerr << "\"" << TRIAL_NUMBER_TEXT_FILE_NAME << "\" is can not opend" << std::endl;
			trialNumberOutput.open(TRIAL_NUMBER_TEXT_FILE_NAME);
		}
		trialCount = 0;
	}
	else
	{
		std::string trial_str;
		std::getline(trialNumberFile, trial_str);
		std::stringstream ss;
		ss << trial_str;
		ss >> trialCount;
	}
	std::cout << "Trial count " << trialCount << std::endl;
	//srand(trialCount * SRAND_FACTOR);
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
		srand(SRAND_INITIAL_NUMBER + trialCount * SRAND_FACTOR);
		broadcastMsg("Task_start");
		// printf("tast_start moderator \n");
		PlaceThings();
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
	//     std::cout << "entNum : " << entNum << std::endl;
	// onCheckObject();
	for(int k=0;k<entNum;k++){
		SimObj* locObj = getObj(m_entNames[k].c_str());
		CParts *parts = locObj->getMainParts();
		bool state = parts->getCollisionState();
	//	std::cout << "the entity : " << m_entNames[k] << std::endl;
	//	std::cout << "the entity stat : " << state << std::endl;
		if ( unable_collision == false)
		{
			if(state){
				colState=true;	// collided with main body of robot

				if(rsLen == 0.0) {
					//pcolState=true;
					r_my->setRotation(prv1Rot);
				}
				else {
					//pcolState=false;
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

			//time_ss << std::setw(2) << std::setfill('0') << msec;
		}
		if(m_ref != NULL){
			std::string mess;
			mess =time_ss.str();
			m_ref->sendMsgToSrv(mess.c_str());
			 //std::cout << "Message to referee : " << time_ss.str() << std::endl;
		}
		else{
			//LOG_MSG((time_ss.str().c_str()));
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

	bool dest = false;




	if(msg == "Start_motion")
	{
		//srand(SRAND_INITIAL_NUMBER + trialCount * SRAND_FACTOR);
		std::cout << "List size " << File_List.size() <<std::endl;
		std::map < std::string, Location >::iterator it = File_List.begin();
		//std::advance(it, rand() % File_List.size());
		//std::advance(it, trialCount); // Moderator1
		std::advance(it, (trialCount)+41); // Modetator2
		std::cout << " the trial count " << trialCount << std::endl;
		LOG_MSG(("Show Task"));
		std::string File_name = "Send_";
		File_name+= it->first;
		File_name+= ".";
		std::cout << " Moderator File Name " << File_name << std::endl;
		sendMsg("Kinect_000", File_name.c_str());
		Location Curent_Locations = it->second;

		// Object_Position = Curent_Locations[0];
		// Trash_Position =  Curent_Locations[1];
		//std::cout << " Curent_Locations[0] " << Curent_Locations[0] << std::endl;
        //std::cout << " Curent_Locations[1] " << Curent_Locations[1] << std::endl;
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
		sendMsg("Kinect_000","Start_motion");
		
	}


	if(msg == "Object_Grasped")
	{
		//CheckObjects();
	}


	if(msg == "Object_Trashed")
	{
		//CheckTrasheBoxs();
	}


	if (sender == "robot_000" && msg == "Task_finished") {
		LOG_MSG(("Task_end"));
	
		sleep(1);

		startTime = 0.0;
		Task_st = true;

		if(ALLOW_ADD_SCORE_WITHOUT_THROW_AWAY) CheckObjects();
		CheckTrasheBoxs();  
		breakTask();
		broadcastMsg("Task_end");
	}

	if( (sender == "robot_000" || sender == "RoboCupReferee")  && msg == "Give_up")
	{
		LOG_MSG(("Task_end"));
		broadcastMsg("Task_end");
		startTime = 0.0;
		breakTask();
	}
	
	if(msg == "init_time")
	{
		init = true;
	}


}

// Right Object + 400
// Wrong Object - 400


// Right Trash Box + 400
// Wrong Trash Box - 400

//enum checkedResult{
//	NO_ACTION = 0,
//	CORRECT_ACTION,
//	INCOREECT_ACTION,
//};
enum placedDirection{
	RIGHT = 0,
	CENTER,
	LEFT,
};

void  MyController::CheckObjects()
{

	//Cm_Objects[Location_Status[0]].Coord
	//Object_Position
	//Location_Status[0]
	for( int i = 0; i < 3 ; i++){
		std:: string name;
		name = Cm_Objects[i].ObjectName;
		SimObj *Obj = getObj(name.c_str());
		// get trash's position
		Vector3d objectPosition;
		Obj->getPosition(objectPosition);

		//Checking how the object was gone
		if(objectPosition.x()== Cm_Objects[i].Coord.x() && objectPosition.y()== Cm_Objects[i].Coord.y() && objectPosition.z()== Cm_Objects[i].Coord.z())
		{
			//LOG_MSG(("<For debug> \"%s\" is placed on the table.", name.c_str()));			
		}
		else
		{
			//LOG_MSG(("<For debug> \"%s\" was gone.", name.c_str()));

			//Is it correct object?
			if(i == Location_Status[0])
			{
				std::stringstream ss;
				ss << ADD_SCORE_GRASP_RIGHT_OBJECT;
				std::string msg = "RoboCupReferee/Robot took the right Object [" + Cm_Objects[Location_Status[0]].ObjectName + "]" "/+" + ss.str();

				if(m_ref != NULL){
					m_ref->sendMsgToSrv(msg.c_str());
				}
				else{
					LOG_MSG((msg.c_str()));
				}				
			}
			else{				
				std::stringstream ss;
				ss << ADD_SCORE_GRASP_WRONG_OBJECT;
				std::string msg = "RoboCupReferee/Robot took the wrong Object "  "" "/" + ss.str();

				if(m_ref != NULL){
					m_ref->sendMsgToSrv(msg.c_str());
				}
				else{
					LOG_MSG((msg.c_str()));
				}
			}
		}
	}
}


void  MyController::CheckTrasheBoxs()
{
	bool objectInTrashBox = false;
	for( int trashBoxIndex = 0; trashBoxIndex < 3; trashBoxIndex++){
		std:: string trashBoxName;
		trashBoxName = Cm_trashBoxs[trashBoxIndex].TrashBoxName;
		SimObj *TrashBox = getObj(trashBoxName.c_str());
		// get trash box position
		Vector3d trashBoxPosition;
		TrashBox->getPosition(trashBoxPosition);

		for( int objectIndex = 0; objectIndex < 3; objectIndex++){ 
			std:: string objectName;
			objectName = Cm_Objects[objectIndex].ObjectName;
			SimObj *Obj = getObj(objectName.c_str());
			// get object position
			Vector3d objectPosition;
			Obj->getPosition(objectPosition);
			
			// Are objects in trash boxes?
			if(objectPosition.x()== Cm_trashBoxs[trashBoxIndex].Coord.x() &&  objectPosition.z()== Cm_trashBoxs[trashBoxIndex].Coord.z())
			{
				//LOG_MSG(("<For debug> \"%s\" is in \"%s\".", objectName.c_str(), trashBoxName.c_str()));
				objectInTrashBox = true;
				if( trashBoxIndex == Location_Status[1] && objectIndex == Location_Status[0] ){
					//LOG_MSG(("<For debug> \"Correct\" object was trashed to \"Correct\" trash box."));
					std::stringstream ss;
					if(ADD_SCORE_RIGHT_OBJECT_IN_RIGHT_TRASH_BOX > 0) ss << "+";
					ss << ADD_SCORE_RIGHT_OBJECT_IN_RIGHT_TRASH_BOX;
					std::string msg = "RoboCupReferee/Robot threw away right obj in right trash box " "/" + ss.str();
					if(m_ref != NULL){
						m_ref->sendMsgToSrv(msg.c_str());
					}
					else{
						LOG_MSG((msg.c_str()));
					}
				}
				else if( trashBoxIndex == Location_Status[1] ){
					//LOG_MSG(("<For debug> \"Wrong\" object was trashed to \"Correct\" trash box."));	
					std::stringstream ss;
					if(ADD_SCORE_WRONG_OBJECT_IN_RIGHT_TRASH_BOX > 0) ss << "+";
					ss << ADD_SCORE_WRONG_OBJECT_IN_RIGHT_TRASH_BOX;
					std::string msg = "RoboCupReferee/Robot threw away wrong obj in right trash box " "/" + ss.str();
					if(m_ref != NULL){
						m_ref->sendMsgToSrv(msg.c_str());
					}
					else{
						LOG_MSG((msg.c_str()));
					}
				}
				else if( objectIndex == Location_Status[0] ){
					//LOG_MSG(("<For debug> \"Correct\" object was trashed to \"Wrong\" trash box."));	
					std::stringstream ss;
					if(ADD_SCORE_RIGHT_OBJECT_IN_WRONG_TRASH_BOX > 0) ss << "+";
					ss << ADD_SCORE_RIGHT_OBJECT_IN_WRONG_TRASH_BOX;
					std::string msg = "RoboCupReferee/Robot threw away right obj in wrong trash box " "/" + ss.str();
					if(m_ref != NULL){
						m_ref->sendMsgToSrv(msg.c_str());
					}
					else{
						LOG_MSG((msg.c_str()));
					}
				}
				else{
					//LOG_MSG(("<For debug> \"Wrong\" object was trashed to \"Wrong\" trash box."));	
					std::stringstream ss;
					if(ADD_SCORE_WRONG_OBJECT_IN_WRONG_TRASH_BOX > 0) ss << "+";
					ss << ADD_SCORE_WRONG_OBJECT_IN_WRONG_TRASH_BOX;
					std::string msg = "RoboCupReferee/Robot threw away wrong obj in wrong trash box " "/" + ss.str();
					if(m_ref != NULL){
						m_ref->sendMsgToSrv(msg.c_str());
					}
					else{
						LOG_MSG((msg.c_str()));
					}
				}
			}
			else
			{
			}
		}
	}
	if(!objectInTrashBox){
		//LOG_MSG(("<For debug> No object was trashed to any trash box."));			
	}
}

void MyController::onCheckCollision(){
}




void MyController::breakTask()
{
	LOG_MSG(("start of breakTask"));
	isCleaningUp = false;
	//takeAwayObjects();
	trialCount++;
	std::ofstream ofs(TRIAL_NUMBER_TEXT_FILE_NAME);
	ofs << trialCount << std::endl;
	ofs.close();
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
		InitRobot();
		// PlaceThings();
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




extern "C" Controller * createController() {
	return new MyController;
}
