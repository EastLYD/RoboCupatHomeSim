#pragma once
#include "stdafx.h"

public ref class Referee : public sigverse::SIGService  
{  
	
public:
	Referee(System::String^ name) : SIGService(name)
	{
		tmp_score = gcnew System::Collections::Generic::List<int>();
		tmp_msg  = gcnew System::Collections::Generic::List<System::String^>();
		m_total = 0;
		m_score = 0;
		trialCount = 0;
		numberOfRepetition = 0;
	};
	~Referee();
	int getScore();
	int getScoreSize();
	int getMessageSize();
	int getTotal();
	int getTrialCount();
	int getNumberOfRepetition();
	void setTotal(int total){ m_total = total; }
	System::String^ getMessage();
	System::String^ getRemainingTime();
	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	int m_total;
	int m_score;
	int trialCount;
	int numberOfRepetition;
	
private:
	System::Collections::Generic::List<int>^ tmp_score;
	System::Collections::Generic::List<System::String^>^ tmp_msg;
	System::String^ remainingTime;

};

  
Referee::~Referee()  
{  
  this->disconnect();  
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
	array<System::String^>^ split_msg = msg->Split(SepString, System::StringSplitOptions::None);
	
	if(split_msg[0] == "RoboCupReferee"){
		// name
		if(split_msg[1] == "time"){
			remainingTime = split_msg[2];
		}
		else if (split_msg[1] == "start"){
			tmp_msg->Add(split_msg[1]);
		}
		else if (split_msg[1] == "end"){
			tmp_msg->Add(split_msg[1]);
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
			m_total += score;
		}
	}
}
