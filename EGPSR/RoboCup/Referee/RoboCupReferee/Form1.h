#pragma once

#include "RoboCupReferee.h"

namespace RoboCupReferee {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		cli::array<System::String^>^ args;
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			m_srv = nullptr;
			m_connected = false;

			// 接続を試みる
			m_srv = gcnew Referee("RoboCupReferee");
			args = System::Environment::GetCommandLineArgs();

			//if(m_srv->connect("socio4.iir.nii.ac.jp", 9333)){
			if (m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
				m_connected = true;
				this->connect->Enabled = false;

				m_srv->connectToViewer();
				m_srv->setAutoExitProc(true);
			}
		}
		void setText(System::String^ text)
		{
			this->label1->Text = text;
		}
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			m_srv = nullptr;
			if (components)
			{
				delete components;
			}
		}
	private: System::ComponentModel::IContainer^  components;
	protected:

	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>

	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Button^  connect;


	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::ListBox^  listBox1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::ListBox^  listBox2;


			 //sigverse::SIGService ^srv = gcnew sigverse::SIGService("RobocupReferee");
			 Referee ^m_srv;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::RadioButton^  radioButton2;
	private: System::Windows::Forms::RadioButton^  radioButton3;
	private: System::Windows::Forms::TextBox^  textBox5;
	private: System::Windows::Forms::Label^  label8;
			 bool m_connected;
			 //srv->connect("socio4.iir.nii.ac.jp", 9333);

#pragma region Windows Form Designer generated code
			 /// <summary>
			 /// Required method for Designer support - do not modify
			 /// the contents of this method with the code editor.
			 /// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->connect = (gcnew System::Windows::Forms::Button());
				 this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->button1 = (gcnew System::Windows::Forms::Button());
				 this->button2 = (gcnew System::Windows::Forms::Button());
				 this->listBox1 = (gcnew System::Windows::Forms::ListBox());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->textBox1 = (gcnew System::Windows::Forms::TextBox());
				 this->listBox2 = (gcnew System::Windows::Forms::ListBox());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->textBox2 = (gcnew System::Windows::Forms::TextBox());
				 this->label4 = (gcnew System::Windows::Forms::Label());
				 this->textBox3 = (gcnew System::Windows::Forms::TextBox());
				 this->label5 = (gcnew System::Windows::Forms::Label());
				 this->label6 = (gcnew System::Windows::Forms::Label());
				 this->textBox4 = (gcnew System::Windows::Forms::TextBox());
				 this->button3 = (gcnew System::Windows::Forms::Button());
				 this->label7 = (gcnew System::Windows::Forms::Label());
				 this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
				 this->radioButton2 = (gcnew System::Windows::Forms::RadioButton());
				 this->radioButton3 = (gcnew System::Windows::Forms::RadioButton());
				 this->textBox5 = (gcnew System::Windows::Forms::TextBox());
				 this->label8 = (gcnew System::Windows::Forms::Label());
				 this->SuspendLayout();
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label1->Location = System::Drawing::Point(316, 416);
				 this->label1->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(71, 27);
				 this->label1->TabIndex = 0;
				 this->label1->Text = L"Total";
				 this->label1->Click += gcnew System::EventHandler(this, &Form1::label1_Click);
				 // 
				 // connect
				 // 
				 this->connect->Enabled = false;
				 this->connect->Font = (gcnew System::Drawing::Font(L"HGPｺﾞｼｯｸE", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->connect->Location = System::Drawing::Point(1, 515);
				 this->connect->Margin = System::Windows::Forms::Padding(4);
				 this->connect->Name = L"connect";
				 this->connect->Size = System::Drawing::Size(118, 42);
				 this->connect->TabIndex = 1;
				 this->connect->Text = L"Connect";
				 this->connect->UseVisualStyleBackColor = true;
				 this->connect->Click += gcnew System::EventHandler(this, &Form1::button1_Click_1);
				 // 
				 // timer1
				 // 
				 this->timer1->Enabled = true;
				 this->timer1->Interval = 10;
				 this->timer1->Tick += gcnew System::EventHandler(this, &Form1::timer1_Tick);
				 // 
				 // button1
				 // 
				 this->button1->Font = (gcnew System::Drawing::Font(L"HGPｺﾞｼｯｸE", 14.25F));
				 this->button1->Location = System::Drawing::Point(121, 515);
				 this->button1->Margin = System::Windows::Forms::Padding(4);
				 this->button1->Name = L"button1";
				 this->button1->Size = System::Drawing::Size(157, 42);
				 this->button1->TabIndex = 2;
				 this->button1->Text = L"Disconnect";
				 this->button1->UseVisualStyleBackColor = true;
				 this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click_2);
				 // 
				 // button2
				 // 
				 this->button2->Font = (gcnew System::Drawing::Font(L"HGPｺﾞｼｯｸE", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->button2->Location = System::Drawing::Point(399, 515);
				 this->button2->Margin = System::Windows::Forms::Padding(4);
				 this->button2->Name = L"button2";
				 this->button2->Size = System::Drawing::Size(120, 42);
				 this->button2->TabIndex = 3;
				 this->button2->Text = L"Initialize";
				 this->button2->UseVisualStyleBackColor = true;
				 this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
				 // 
				 // listBox1
				 // 
				 this->listBox1->BackColor = System::Drawing::SystemColors::InfoText;
				 this->listBox1->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->listBox1->ForeColor = System::Drawing::SystemColors::Menu;
				 this->listBox1->FormattingEnabled = true;
				 this->listBox1->HorizontalScrollbar = true;
				 this->listBox1->ItemHeight = 26;
				 this->listBox1->Location = System::Drawing::Point(8, 56);
				 this->listBox1->Margin = System::Windows::Forms::Padding(4);
				 this->listBox1->Name = L"listBox1";
				 this->listBox1->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->listBox1->Size = System::Drawing::Size(384, 238);
				 this->listBox1->TabIndex = 4;
				 this->listBox1->SelectedIndexChanged += gcnew System::EventHandler(this, &Form1::listBox1_SelectedIndexChanged);
				 // 
				 // label2
				 // 
				 this->label2->AutoSize = true;
				 this->label2->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label2->Location = System::Drawing::Point(390, 32);
				 this->label2->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label2->Name = L"label2";
				 this->label2->Size = System::Drawing::Size(56, 20);
				 this->label2->TabIndex = 5;
				 this->label2->Text = L"Score";
				 // 
				 // textBox1
				 // 
				 this->textBox1->BackColor = System::Drawing::SystemColors::InfoText;
				 this->textBox1->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->textBox1->ForeColor = System::Drawing::SystemColors::InactiveBorder;
				 this->textBox1->Location = System::Drawing::Point(394, 409);
				 this->textBox1->Margin = System::Windows::Forms::Padding(4);
				 this->textBox1->Name = L"textBox1";
				 this->textBox1->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->textBox1->Size = System::Drawing::Size(113, 37);
				 this->textBox1->TabIndex = 6;
				 this->textBox1->Text = L"0";
				 this->textBox1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // listBox2
				 // 
				 this->listBox2->BackColor = System::Drawing::SystemColors::InfoText;
				 this->listBox2->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->listBox2->ForeColor = System::Drawing::SystemColors::Menu;
				 this->listBox2->FormattingEnabled = true;
				 this->listBox2->HorizontalScrollbar = true;
				 this->listBox2->ItemHeight = 26;
				 this->listBox2->Location = System::Drawing::Point(385, 56);
				 this->listBox2->Margin = System::Windows::Forms::Padding(4);
				 this->listBox2->Name = L"listBox2";
				 this->listBox2->Size = System::Drawing::Size(127, 238);
				 this->listBox2->TabIndex = 7;
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label3->Location = System::Drawing::Point(17, 32);
				 this->label3->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(87, 20);
				 this->label3->TabIndex = 8;
				 this->label3->Text = L"Comment";
				 // 
				 // textBox2
				 // 
				 this->textBox2->BackColor = System::Drawing::SystemColors::InfoText;
				 this->textBox2->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->textBox2->ForeColor = System::Drawing::SystemColors::InactiveBorder;
				 this->textBox2->Location = System::Drawing::Point(222, 347);
				 this->textBox2->Margin = System::Windows::Forms::Padding(4);
				 this->textBox2->Name = L"textBox2";
				 this->textBox2->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->textBox2->Size = System::Drawing::Size(86, 37);
				 this->textBox2->TabIndex = 6;
				 this->textBox2->Text = L"00:00";
				 this->textBox2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label4
				 // 
				 this->label4->AutoSize = true;
				 this->label4->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label4->Location = System::Drawing::Point(148, 354);
				 this->label4->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label4->Name = L"label4";
				 this->label4->Size = System::Drawing::Size(66, 27);
				 this->label4->TabIndex = 0;
				 this->label4->Text = L"Time";
				 this->label4->Click += gcnew System::EventHandler(this, &Form1::label1_Click);
				 // 
				 // textBox3
				 // 
				 this->textBox3->BackColor = System::Drawing::SystemColors::InfoText;
				 this->textBox3->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->textBox3->ForeColor = System::Drawing::SystemColors::InactiveBorder;
				 this->textBox3->Location = System::Drawing::Point(348, 463);
				 this->textBox3->Margin = System::Windows::Forms::Padding(4);
				 this->textBox3->Name = L"textBox3";
				 this->textBox3->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->textBox3->Size = System::Drawing::Size(57, 37);
				 this->textBox3->TabIndex = 10;
				 this->textBox3->Text = L"0";
				 this->textBox3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // label5
				 // 
				 this->label5->AutoSize = true;
				 this->label5->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label5->Location = System::Drawing::Point(135, 470);
				 this->label5->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label5->Name = L"label5";
				 this->label5->Size = System::Drawing::Size(205, 27);
				 this->label5->TabIndex = 9;
				 this->label5->Text = L"Number of Trials";
				 // 
				 // label6
				 // 
				 this->label6->AutoSize = true;
				 this->label6->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 20.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label6->Location = System::Drawing::Point(412, 466);
				 this->label6->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label6->Name = L"label6";
				 this->label6->Size = System::Drawing::Size(33, 34);
				 this->label6->TabIndex = 11;
				 this->label6->Text = L"/";
				 this->label6->Click += gcnew System::EventHandler(this, &Form1::label6_Click);
				 // 
				 // textBox4
				 // 
				 this->textBox4->BackColor = System::Drawing::SystemColors::InfoText;
				 this->textBox4->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->textBox4->ForeColor = System::Drawing::SystemColors::InactiveBorder;
				 this->textBox4->Location = System::Drawing::Point(450, 463);
				 this->textBox4->Margin = System::Windows::Forms::Padding(4);
				 this->textBox4->Name = L"textBox4";
				 this->textBox4->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->textBox4->Size = System::Drawing::Size(57, 37);
				 this->textBox4->TabIndex = 12;
				 this->textBox4->Text = L"0";
				 this->textBox4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 // 
				 // button3
				 // 
				 this->button3->Font = (gcnew System::Drawing::Font(L"HGPｺﾞｼｯｸE", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->button3->Location = System::Drawing::Point(281, 515);
				 this->button3->Margin = System::Windows::Forms::Padding(4);
				 this->button3->Name = L"button3";
				 this->button3->Size = System::Drawing::Size(117, 42);
				 this->button3->TabIndex = 13;
				 this->button3->Text = L"Give Up";
				 this->button3->UseVisualStyleBackColor = true;
				 this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
				 // 
				 // label7
				 // 
				 this->label7->AutoSize = true;
				 this->label7->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label7->Location = System::Drawing::Point(1, 312);
				 this->label7->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label7->Name = L"label7";
				 this->label7->Size = System::Drawing::Size(290, 27);
				 this->label7->TabIndex = 14;
				 this->label7->Text = L"Use of special functions";
				 // 
				 // radioButton1
				 // 
				 this->radioButton1->AutoSize = true;
				 this->radioButton1->Checked = true;
				 this->radioButton1->Location = System::Drawing::Point(315, 320);
				 this->radioButton1->Margin = System::Windows::Forms::Padding(4);
				 this->radioButton1->Name = L"radioButton1";
				 this->radioButton1->Size = System::Drawing::Size(62, 19);
				 this->radioButton1->TabIndex = 15;
				 this->radioButton1->TabStop = true;
				 this->radioButton1->Text = L"None";
				 this->radioButton1->UseVisualStyleBackColor = true;
				 this->radioButton1->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButton1_CheckedChanged);
				 // 
				 // radioButton2
				 // 
				 this->radioButton2->AutoSize = true;
				 this->radioButton2->Location = System::Drawing::Point(381, 320);
				 this->radioButton2->Margin = System::Windows::Forms::Padding(4);
				 this->radioButton2->Name = L"radioButton2";
				 this->radioButton2->Size = System::Drawing::Size(65, 19);
				 this->radioButton2->TabIndex = 16;
				 this->radioButton2->Text = L"Either";
				 this->radioButton2->UseVisualStyleBackColor = true;
				 this->radioButton2->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButton2_CheckedChanged);
				 // 
				 // radioButton3
				 // 
				 this->radioButton3->AutoSize = true;
				 this->radioButton3->Location = System::Drawing::Point(451, 320);
				 this->radioButton3->Margin = System::Windows::Forms::Padding(4);
				 this->radioButton3->Name = L"radioButton3";
				 this->radioButton3->Size = System::Drawing::Size(59, 19);
				 this->radioButton3->TabIndex = 17;
				 this->radioButton3->Text = L"Both";
				 this->radioButton3->UseVisualStyleBackColor = true;
				 this->radioButton3->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButton3_CheckedChanged);
				 // 
				 // textBox5
				 // 
				 this->textBox5->BackColor = System::Drawing::SystemColors::InfoText;
				 this->textBox5->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->textBox5->ForeColor = System::Drawing::SystemColors::InactiveBorder;
				 this->textBox5->Location = System::Drawing::Point(394, 347);
				 this->textBox5->Margin = System::Windows::Forms::Padding(4);
				 this->textBox5->Name = L"textBox5";
				 this->textBox5->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->textBox5->Size = System::Drawing::Size(113, 37);
				 this->textBox5->TabIndex = 19;
				 this->textBox5->Text = L"0";
				 this->textBox5->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
				 this->textBox5->TextChanged += gcnew System::EventHandler(this, &Form1::textBox5_TextChanged);
				 // 
				 // label8
				 // 
				 this->label8->AutoSize = true;
				 this->label8->Font = (gcnew System::Drawing::Font(L"HGSｺﾞｼｯｸE", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(128)));
				 this->label8->Location = System::Drawing::Point(316, 354);
				 this->label8->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label8->Name = L"label8";
				 this->label8->Size = System::Drawing::Size(76, 27);
				 this->label8->TabIndex = 18;
				 this->label8->Text = L"Score";
				 // 
				 // Form1
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->BackColor = System::Drawing::SystemColors::ButtonFace;
				 this->ClientSize = System::Drawing::Size(520, 570);
				 this->Controls->Add(this->textBox5);
				 this->Controls->Add(this->label8);
				 this->Controls->Add(this->radioButton3);
				 this->Controls->Add(this->radioButton2);
				 this->Controls->Add(this->radioButton1);
				 this->Controls->Add(this->label7);
				 this->Controls->Add(this->button3);
				 this->Controls->Add(this->textBox4);
				 this->Controls->Add(this->label6);
				 this->Controls->Add(this->textBox3);
				 this->Controls->Add(this->label5);
				 this->Controls->Add(this->label3);
				 this->Controls->Add(this->listBox2);
				 this->Controls->Add(this->textBox2);
				 this->Controls->Add(this->textBox1);
				 this->Controls->Add(this->label2);
				 this->Controls->Add(this->listBox1);
				 this->Controls->Add(this->button2);
				 this->Controls->Add(this->button1);
				 this->Controls->Add(this->connect);
				 this->Controls->Add(this->label4);
				 this->Controls->Add(this->label1);
				 this->Margin = System::Windows::Forms::Padding(4);
				 this->Name = L"Form1";
				 this->RightToLeft = System::Windows::Forms::RightToLeft::No;
				 this->Text = L"Robocup@home 2015";
				 this->Load += gcnew System::EventHandler(this, &Form1::Form1_Load);
				 this->ResumeLayout(false);
				 this->PerformLayout();

			 }

#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {


		//
	}
	private: System::Void label1_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void button1_Click_1(System::Object^  sender, System::EventArgs^  e) {
		if (m_srv == nullptr){
			m_srv = gcnew Referee("RoboCupReferee");
			//if(m_srv->connect("socio4.iir.nii.ac.jp", 9333)){
			if (m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
				m_connected = true;
				m_srv->connectToViewer();
				m_srv->setAutoExitProc(true);
				this->connect->Enabled = false;
				this->button1->Enabled = true;
				m_srv->m_total = int::Parse(this->textBox1->Text);
				m_srv->trialCount = int::Parse(this->textBox3->Text);
				m_srv->numberOfRepetition = int::Parse(this->textBox4->Text);
			}
		}
	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		if (m_srv != nullptr){
			m_srv->checkRecvData(100);
			int ssize = m_srv->getScoreSize();
			// メッセージが来た
			for (int i = 0; i < ssize; i++){
				int score = m_srv->getScore();
				System::String^ msg = m_srv->getMessage();
				System::String^ listitem_msg = msg;// + "                    " + score.ToString();
				System::String^ listitem_score = score.ToString();
				this->listBox1->Items->Add(listitem_msg);
				this->listBox2->Items->Add(listitem_score);
				double total = m_srv->getTotal();
				double tmpTotal = m_srv->getTmpTotal();
				double penalty = m_srv->getPenalty();
				if (radioButton2->Checked == true){
					total /= 2;
					tmpTotal /= 2;
				}
				else if (radioButton3->Checked == true){
					total /= 4;
					tmpTotal /= 4;
				}
				double tmp = total + penalty;
				this->textBox1->Text = tmp.ToString();
				tmp = tmpTotal + penalty;
				this->textBox5->Text = tmp.ToString();
			}
			//commentを更新
			int msize = m_srv->getMessageSize();
			for (int i = 0; i < msize; i++){
				System::String^ comment = m_srv->getMessage();
				this->listBox1->Items->Add(comment);
				this->listBox2->Items->Add("");
			}
			//試行を繰り返す回数と現在の試行回数を更新
			int trialCount = m_srv->getTrialCount();
			this->textBox3->Text = trialCount.ToString();
			int numberOfRepetition = m_srv->getNumberOfRepetition();
			this->textBox4->Text = numberOfRepetition.ToString();

			System::String^ msg = m_srv->getRemainingTime();
			if (msg != ""){
				this->textBox2->Text = msg;
			}
			else{
				this->textBox2->Text = "00:00";
			}
		}
	}
	private: System::Void button1_Click_2(System::Object^  sender, System::EventArgs^  e) {
		m_srv->disconnect();
		this->connect->Enabled = true;
		this->button1->Enabled = false;
		m_connected = false;
		m_srv = nullptr;
	}
	private: System::Void listBox1_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void Form1_Load(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
		m_srv->setTotal(0);
		this->textBox1->Text = L"0";
		this->listBox1->Items->Clear();
		this->listBox2->Items->Clear();
	}
	private: System::Void label6_Click(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
		m_srv->sendMsgToCtr("moderator_0", "Give_up");
	}
	private: System::Void radioButton1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (radioButton1->Checked == true){
			radioButton2->Checked = false;
			radioButton3->Checked = false;
			double total = m_srv->getTotal();
			double tmpTotal = m_srv->getTmpTotal();
			this->textBox1->Text = total.ToString();
			this->textBox5->Text = tmpTotal.ToString();
		}

	}
	private: System::Void radioButton2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (radioButton2->Checked == true){
			radioButton1->Checked = false;
			radioButton3->Checked = false;
			double total = m_srv->getTotal() / 2;
			double tmpTotal = m_srv->getTmpTotal() / 2;
			double penalty = m_srv->getPenalty();
			double tmp = total + penalty;
			this->textBox1->Text = tmp.ToString();
			tmp = tmpTotal + penalty;
			this->textBox5->Text = tmp.ToString();
		}
	}
	private: System::Void radioButton3_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (radioButton3->Checked == true){
			radioButton1->Checked = false;
			radioButton2->Checked = false;
			double total = m_srv->getTotal() / 4;
			double tmpTotal = m_srv->getTmpTotal() / 4;
			double penalty = m_srv->getPenalty();
			double tmp = total + penalty;
			this->textBox1->Text = tmp.ToString();
			tmp = tmpTotal + penalty;
			this->textBox5->Text = tmp.ToString();
		}
	}
	private: System::Void textBox5_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	};
}

