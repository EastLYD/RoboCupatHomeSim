#pragma once
#include "stdafx.h"
#include <fstream>

std::ofstream ofs("log.csv", std::ios::app);
public ref class Referee : public sigverse::SIGService  
{  
	
public:
	Referee(System::String^ name) : SIGService(name)
	{
		remainingTime = "";
		tmp_score = gcnew System::Collections::Generic::List<int>();
		tmp_msg = gcnew System::Collections::Generic::List<System::String^>();
		m_total = 0;
		m_score = 0;
		tmp_total = 0;
		trialCount = 0;
		numberOfRepetition = 0;
		penalty = 0;
		writeLog = true;
	};
	~Referee();
	int getScore();
	int getTmpTotal();
	int getScoreSize();
	int getMessageSize();
	int getTotal();
	int getTrialCount();
	int getNumberOfRepetition();
	int getPenalty();
	void setTotal(int total){ m_total = total; }
	System::String^ getMessage();
	System::String^ getRemainingTime();
	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	int m_total;
	int m_score;
	int tmp_total;
	int trialCount;
	int numberOfRepetition;
	int penalty;
	
private:
	System::Collections::Generic::List<int>^ tmp_score;
	System::Collections::Generic::List<System::String^>^ tmp_msg;
	System::String^ remainingTime;
	std::string sysString2stdStrng(System::String^ sys_str);
	bool writeLog;
};

  
Referee::~Referee()  
{  
  this->disconnect();  
}  

std::string Referee::sysString2stdStrng(System::String^ sys_str)
{
	std::string tmp;
	System::IntPtr mptr;
	mptr = System::Runtime::InteropServices::Marshal::StringToHGlobalAnsi(sys_str);
	tmp = static_cast<const char*>(mptr.ToPointer());
	System::Runtime::InteropServices::Marshal::FreeHGlobal(mptr);
	return tmp;
}

int Referee::getScore()
{
	int score = tmp_score[0];
	tmp_score->RemoveAt(0);

	return score;
}

int Referee::getTotal()
{
	return m_total;

}

int Referee::getTmpTotal()
{
	return tmp_total;
}

int Referee::getTrialCount()
{
	return trialCount;
}
int Referee::getNumberOfRepetition()
{
	return numberOfRepetition;
}
int Referee::getPenalty()
{
	return penalty;
}
System::String^ Referee::getMessage()
{
	System::String^ msg = tmp_msg[0];
	tmp_msg->RemoveAt(0);

	return msg;
}

int Referee::getScoreSize()
{
	return tmp_score->Count;
}

int Referee::getMessageSize()
{
	return tmp_msg->Count;
}

double Referee::onAction()
{
	return 10.0;
}

System::String^ Referee::getRemainingTime()
{
	return remainingTime;
}

void Referee::onRecvMsg(sigverse::RecvMsgEvent ^evt)  
{  
	
	System::String ^name = evt->getSender();
	System::String ^msg  = evt->getMsg();

	array<System::String^>^ SepString={"/"};

	// split
	array<System::String^>^ split_msg = msg->Split(SepString, System::StringSplitOptions::None);

	if(split_msg[0] == "RoboCupReferee"){
		// name
		if(split_msg[1] == "time"){
			remainingTime = split_msg[2];
		}
		else if (split_msg[1] == "reset"){
			tmp_msg->Add(split_msg[1]);
			int score = tmp_total + penalty;
			if (score > 0){
				m_total += score;
			}
			tmp_total = 0;
			penalty = 0;
		}
		else if (split_msg[1] == "trial"){
			trialCount = int::Parse(split_msg[2]);
			numberOfRepetition = int::Parse(split_msg[3]);
		}
		else{
			tmp_msg->Add(split_msg[1]);
			// score
			int score = int::Parse(split_msg[2]);
			tmp_score->Add(score);
			if (score >= 0){
				tmp_total += score;
			}
			else{
				penalty += score;
			}
		}
	}
	writeLog = true;
	if (split_msg[0] == "RoboCupReferee"){
		if (split_msg[1] == "time"){
			writeLog = false;
		}
	}
	if (writeLog == true){
		std::string tmp = sysString2stdStrng(remainingTime);
		ofs << tmp.c_str() << ",";

		tmp = sysString2stdStrng(evt->getSender());
		ofs << tmp.c_str() << ",";

		array<System::String^>^ split_msg = msg->Split(SepString, System::StringSplitOptions::None);
		//sendMsg("SIGViewer", split_msg->Length.ToString());
		for (int i = 0; i < split_msg->Length; i++){
			tmp = sysString2stdStrng(split_msg[i]);
			ofs << tmp.c_str() << ",";
		}
		ofs << "\n";
	}
}
