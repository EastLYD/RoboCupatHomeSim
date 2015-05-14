#pragma once
#include "stdafx.h"
#include <fstream>

std::ofstream ofs("log.csv", std::ios::app);
public ref class Referee : public sigverse::SIGService  
{  
	
public:
	Referee(System::String^ name) : SIGService(name)
	{
		tmp_score = gcnew System::Collections::Generic::List<int>();
		tmp_msg  = gcnew System::Collections::Generic::List<System::String^>();
		m_total = 0;
		tmp_total = 0;
		m_score = 0;
		trialCount = 0;
		numberOfRepetition = 0;
		penalty = 0;
	};
	~Referee();
	double getScore();
	double getTmpTotal();
	int getScoreSize();
	int getMessageSize();
	double getPenalty();
	double getTotal();
	int getTrialCount();
	int getNumberOfRepetition();
	void setTotal(int total){ m_total = total; }
	System::String^ getMessage();
	System::String^ getRemainingTime();
	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	double m_total;
	double m_score;
	double tmp_total;
	int trialCount;
	int numberOfRepetition;
	double penalty;
	
private:
	System::Collections::Generic::List<int>^ tmp_score;
	System::Collections::Generic::List<System::String^>^ tmp_msg;
	System::String^ remainingTime;
};

  
Referee::~Referee()  
{  
  this->disconnect();  
}  

double Referee::getScore()
{
	int score = tmp_score[0];
	tmp_score->RemoveAt(0);
	
	return score;
}

double Referee::getTotal()
{
	if (tmp_total > 0){
		return m_total + tmp_total;
	}
	return m_total;
}

double Referee::getTmpTotal()
{
	return tmp_total;
}

double Referee::getPenalty()
{
	return penalty;
}

int Referee::getTrialCount()
{
	return trialCount;
}
int Referee::getNumberOfRepetition()
{
	return numberOfRepetition;
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
	std::string tmp1;
	System::IntPtr mptr1 = System::Runtime::InteropServices::Marshal::StringToHGlobalAnsi(evt->getSender());
	tmp1 = static_cast<const char*>(mptr1.ToPointer());
	System::Runtime::InteropServices::Marshal::FreeHGlobal(mptr1);
	ofs << tmp1.c_str() << ",";

	array<System::String^>^ split_msg = msg->Split(SepString, System::StringSplitOptions::None);
	//sendMsg("SIGViewer", split_msg->Length.ToString());
	for (int i = 0; i < split_msg->Length; i++){
		std::string tmp;
		System::IntPtr mptr = System::Runtime::InteropServices::Marshal::StringToHGlobalAnsi(split_msg[i]);
		tmp = static_cast<const char*>(mptr.ToPointer());
		System::Runtime::InteropServices::Marshal::FreeHGlobal(mptr);
		ofs << tmp.c_str() << ",";
	}
	ofs << "\n";

	if(split_msg[0] == "RoboCupReferee"){
		// name
		if(split_msg[1] == "time"){
			remainingTime = split_msg[2];
		}
		else if (split_msg[1] == "reset"){
			tmp_msg->Add(split_msg[1]);
			if (tmp_total > 0){
				m_total += tmp_total;
			}
			tmp_total = 0;
		}
		else if (split_msg[1] == "trial"){
			trialCount = int::Parse(split_msg[2]);
			numberOfRepetition = int::Parse(split_msg[3]);
		}
		else{
			tmp_msg->Add(split_msg[1]);
		
			// score
			m_score = int::Parse(split_msg[2]);
			tmp_score->Add(m_score);
			if (m_score > 0){
				tmp_total += m_score;
			}
			else{
				penalty += m_score;
				if (penalty < -1200){
					penalty = -1200;
				}
			}
		}
	}
}
