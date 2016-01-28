#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

class Kinect_Service : public Controller  {

public:
    double onAction();
	void onInit();
	void onRecvMsg(RecvMsgEvent &evt);
    bool Send_Data;
	bool Read_File;
	int Data_Size;
	std::vector<std::string> list;
    std::ifstream in_stream;
    std::string line;
private: 

};


void Kinect_Service::onInit(){

	     Send_Data = false;
	     Read_File = false;
	     Data_Size = 0;

}



double Kinect_Service::onAction(){

//int connectedNum = getConnectedControllerNum();

		if(Send_Data)
	{
		//std::vector<std::string> names = getAllConnectedEntitiesName();
		//for (int i = 0; i < connectedNum; i++){

	   if(Data_Size < list.size())
	   {
		sendMsg("man_000", list[Data_Size]);
	   // std::cout << "The Line    " << list[Data_Size] <<std::endl;
	   }

	//	}
		Data_Size++;
std::cout << "Data line  " << Data_Size <<std::endl;
	}
//std::cout << "Data status  " << Send_Data <<std::endl;
//std::cout << "Data line  " << Data_Size <<std::endl;
	return 0.1;
}

void Kinect_Service::onRecvMsg(RecvMsgEvent &evt){

	std::string sender = evt.getSender();
		std::string msg = evt.getMsg();


    int strPos1 = 0;
	int strPos2;
	int strPos3;
	std::string headss,File_ID;
	std::string ss = msg;

	strPos2 = ss.find("_", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);
if (headss == "Send") {
		// Contol of body movement by KINECT
		//moveBodyByKINECT(all_msg);

    strPos3 = ss.find(".", strPos2+1);
	File_ID.assign(ss, strPos2+1, strPos3);
	File_ID += ".dat";
	}

		if (strcmp("Send_Data", msg.c_str()) == 0)
		{
			Send_Data = true;
			printf("Send_Data");
		}

		if (strcmp("Read_File", msg.c_str()) == 0)
		{
				in_stream.open("Motion_file.dat");
				std::string previousLine = "";
				while(getline(in_stream,line)) // To get you all the lines.
        {
						list.push_back(line);
        }	
				in_stream.close();
				  Data_Size = 0;
				  printf("Read_File");
		}

}