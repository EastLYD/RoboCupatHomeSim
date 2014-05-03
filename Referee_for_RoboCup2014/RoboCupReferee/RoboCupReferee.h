#pragma once
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>

#define POINT 4//1 or 2 or 4

public ref class Referee : public sigverse::SIGService  
{  
	
public:
	Referee(System::String^ name) : SIGService(name)
	{
		tmp_score = gcnew System::Collections::Generic::List<int>();
		tmp_msg  = gcnew System::Collections::Generic::List<System::String^>();
		m_total = 0;
		trial_count = 1;
	};
	~Referee();
	int getScore();
	int getScoreSize();
	int getTotal();
	void setTotal(int total){ m_total = total; }
	System::String^ getMessage();
	System::String^ getRemainingTime();
	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	
private:
	System::Collections::Generic::List<int>^ tmp_score;
	System::Collections::Generic::List<System::String^>^ tmp_msg;
	System::String^ remainingTime;
	int m_total;
	int trial_count;
	FILE* fp;
};

  
Referee::~Referee()  
{  
  this->disconnect();  
  fclose(fp);
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
		else if(split_msg[1] == "start"){
			if(trial_count==1)	fp = fopen("score.csv","w");
			if(fp!=NULL)	fprintf(fp,"trial,%d\n",trial_count);
		}
		else if(split_msg[1] == "end"){
			if(m_total<0)	m_total = 0;
			if(fp!=NULL)	fprintf(fp,"total,%d\n",m_total);
			m_total = 0;
			trial_count++;
		}
		else{
			tmp_msg->Add(split_msg[1]);
		
			// score
			int score = int::Parse(split_msg[2]);
			if(score>0)	score = score/POINT;
			tmp_score->Add(score);
			m_total += score;
			if(fp!=NULL)	fprintf(fp,"score,%d\n",score);
		}
	}
}
