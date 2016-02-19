#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

class KinectService : public Controller  {

public:
	double onAction(ActionEvent&);
	void onInit(InitEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);


private: 
	bool Send_Data;
	bool Read_File;
	bool Start_motion;
	int Data_Size;
	std::vector<std::string> list_motion;
	std::vector<std::string> list_current;
	std::ifstream in_stream_current;	
	std::ifstream in_stream_motion;
	std::string line;
	std::string File_ID;
	std::string File_Current;
	double Current_t;
	clock_t Start_t;
};

void KinectService::onInit(InitEvent &evt){

	     Send_Data = false;
	     Read_File = false;
	     Start_motion = false;
	     Data_Size = 0;
	     File_Current = "Current_motion.dat";


				in_stream_current.open(File_Current.c_str());
				std::string previousLine = "";
				while(getline(in_stream_current,line)) // To get you all the lines for current motion.
        {
						list_current.push_back(line.erase(line);
        }	
				in_stream_current.close();
	    Data_Size = 0;
        Start_t = clock();
}



double KinectService::onAction(ActionEvent &evt){

	Current_t = (clock() - Start_t) / (double)(CLOCKS_PER_SEC / 1000);
	std::ostringstream os;
	os <<  Current_t;
	std::string str = os.str();
	if (Start_motion ) //// show object and trash box
	{  

		if(Data_Size < list_motion.size())
		{
			int strPos1 = 0;
			int strPos2;
			int strPos3;
			std::string headss;

			strPos2 = list_motion[Data_Size].find("  ", strPos1);

			headss.assign(list_motion[Data_Size], strPos1, strPos2-strPos1);

			if(Send_Data && headss == "KINECT_DATA_Sensor" )
			{	
				std::string Message;

				Message = list_motion[Data_Size] + str + "..";

				sendMsg("robot_000", Message.c_str());
			}
			int strPos12 = 0;
			int strPos22;
			int strPos32;
			std::string headss2;
			strPos22 = list_current[Data_Size].find(" ", strPos12);
			headss2.assign(list_motion[Data_Size], strPos12, strPos22-strPos12);

			if ( headss2 == "KINECT_DATA" )
			{
			sendMsg("man_000", list_motion[Data_Size].c_str());
			}
		}
		else 
		{
			Data_Size = 0;
			Start_motion = false;
		}
		Data_Size++;
	}
	else     //// current posture of avatar    avatar stands in front of objects
	{
		int strPos1 = 0;
		int strPos2;
		int strPos3;
		std::string headss;
		//std::string ss = msg;
		if(Data_Size < list_current.size())
		{
			strPos2 = list_current[Data_Size].find("  ", strPos1);
			headss.assign(list_current[Data_Size], strPos1, strPos2-strPos1);
			if(Send_Data && headss == "KINECT_DATA_Sensor" )
			{
				std::string Message;
				Message = list_current[Data_Size] + str + ".";
				sendMsg("robot_000", Message.c_str());
			}
			int strPos12 = 0;
			int strPos22;
			int strPos32;
			std::string headss2;
			//std::string ss = msg;

			strPos22 = list_current[Data_Size].find(" ", strPos12);
			headss2.assign(list_current[Data_Size], strPos12, strPos22-strPos12);
			if ( headss2 == "KINECT_DATA" )
			{
				sendMsg("man_000", list_current[Data_Size].c_str());
			}
		}
		else 
		{
			Data_Size = 0;
		}
	Data_Size++;
	}
	return 0.1;
}

void KinectService::onRecvMsg(RecvMsgEvent &evt){

	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();

	int strPos1 = 0;
	int strPos2;
	int strPos3;
	std::string headss;
	std::string ss = msg;

	strPos2 = ss.find("_", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);
	if (headss == "Send") {
		// Contol of body movement by KINECT
		strPos3 = ss.find(".", strPos2+1);
		File_ID.assign(ss, strPos2+1, strPos3);
		File_ID += "dat";
	}
	if (msg == "Start_motion" && sender == "moderator_0" )
	{
		in_stream_motion.open(File_ID.c_str());
		std::string previousLine = "";
		while(getline(in_stream_motion,line)) // To get you all the lines.
		{
			list_motion.push_back(line.erase(line);
		}	
		in_stream_motion.close();
		Data_Size = 0;
		Start_motion = true;
	}
	if (msg == "Task_start"  && sender == "moderator_0")
	{
		Data_Size = 0;
		Start_t = clock();
		list_motion.clear();
		Send_Data = false;
	}
	if (msg == "Get_Data" && sender == "robot_000")
	{
		Send_Data = true;
	}	
	if (msg == "Data_Off" && sender == "robot_000")
	{
		Send_Data = false;
	}

}

extern "C" Controller * createController() {
	return new KinectService;
}
