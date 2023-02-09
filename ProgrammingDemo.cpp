// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <Windows.h>

using namespace std;

FRAME brels = { {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1} }; // Tsb = T01
FRAME trelw = { {1,0,0,0},{0,1,0,0},{0,0,1,0}, {0,0,0,1 } }; // Twt = T56

void UTOI(JOINT vec4, FRAME& T);

void ITOU(FRAME T, JOINT& vec4);

void TMULT(FRAME brela, FRAME crelb, FRAME& crela);

void TINVERT(FRAME T, FRAME& Tinv);

void KIN(JOINT vec3, FRAME& wrelb);

void WHERE(JOINT posV, FRAME& trels, JOINT& resultPos);

void INVKIN(FRAME wrelb, JOINT current, JOINT& near1, JOINT& far1, bool& sol);

void SOLVE(FRAME& wrelb, JOINT& current, JOINT& near1, JOINT& far1, bool& sol);
void SOLVE2(FRAME trels, JOINT& current, JOINT& near1, JOINT& far1, bool& sol);

void PrintMatrix(FRAME matrix);

void PrintVector(JOINT vector);

void current2deg(JOINT& current);

int sgn(double theta1, double theta2);

void P_Blend(JOINT paramArr[5], JOINT& th_vel, TRAJ& th_ACC, double tin4, int order, TRAJ& joint_tb, TRAJ& joint_thb);

void PrintVector2(TRAJ vector);

double linearreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack);

double firstreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack);

double lastreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack);

bool checkjointlimit(JOINT& pos);

bool checkvel(double limit, double value, int jointnum);

bool checkacc(double value, double minacc, int jointnum);

void writeData(ofstream& th_values, JOINT paramArr[5], JOINT th_ACC, TRAJ th_tb, TRAJ th1_thB, double T);

int main(int argc, char* argv[])
{
	printf("Keep this window in focus, and...\n");

	char ch;
	int c;

	const int ESC = 27;

	printf("Press any key to continue \n");
	printf("Press ESC to exit \n");

	c = _getch();

	while (1)
	{

		if (c != ESC)
		{

			JOINT JointVect;

			int typeIn;

			cout << "Enter # to indicate the function:" << endl << "(1) KIN / move to joint vector";
			cout << endl << "(2) INVKIN / move to position vector" << endl << "(3) Trajectory Planner";
			cout << endl << "(4) skip" << endl;
			cin >> typeIn;
			if (typeIn == 1)
			{
				cout << endl << "Input the vector parameters:" << endl << "theta1 = ";
				cin >> JointVect[0];
				cout << "theta2 = ";
				cin >> JointVect[1];
				cout << "d3 = ";
				cin >> JointVect[2];
				cout << "theta4 = ";
				cin >> JointVect[3];

				FRAME testT;
				JOINT current;
				GetConfiguration(current);
				if (RAD2DEG(current[0]) < 360)
					current2deg(current);
				KIN(JointVect, testT);
			}
			else if (typeIn == 2)
			{
				FRAME WTT;
				JOINT far1 = { 0 };
				bool sol;
				JOINT current;
				GetConfiguration(current);
				if (RAD2DEG(current[0]) < 360)
					current2deg(current);
				SOLVE(WTT, current, JointVect, far1, sol);
			}
			else if (typeIn == 3) //Trajectory Planning
			{
				JOINT paramArr[5];  //via1, via2,via3 and via4/goal frame
				double T, tin4; //total desired time, 
				FRAME T1, T2, T3, TG;
				JOINT far1 = { 0 };
				bool sol;
				JOINT current;

				cout << endl << "Input parameters for Via - Point #1:" << endl;
				GetConfiguration(paramArr[0]);
				if (RAD2DEG(paramArr[0][0]) < 360) // cause for some dumb reason getconfig is both radian and degree at random
					current2deg(paramArr[0]);
				SOLVE(T1, paramArr[0], paramArr[1], far1, sol);
				if (sol == false)
				{
					cout << endl << "Aborting Program..." << endl;
					break;
				}
				cout << endl << "Input parameters for Via - Point #2:" << endl;
				GetConfiguration(current);
				if (RAD2DEG(current[0]) < 360)
					current2deg(current);
				SOLVE(T2, current, paramArr[2], far1, sol);
				if (sol == false)
				{
					cout << endl << "Aborting Program..." << endl;
					break;
				}
				cout << endl << "Input parameters for Via - Point #3:" << endl;
				GetConfiguration(current);
				if (RAD2DEG(current[0]) < 360)
					current2deg(current);
				SOLVE(T3, current, paramArr[3], far1, sol);
				if (sol == false)
				{
					cout << endl << "Aborting Program..." << endl;
					break;
				}
				cout << endl << "Input parameters for Goal Frame:" << endl;
				GetConfiguration(current);
				if (RAD2DEG(current[0]) < 360)
					current2deg(current);
				SOLVE(TG, current, paramArr[4], far1, sol);
				if (sol == false)
				{
					cout << endl << "Aborting Program..." << endl;
					break;
				}

				cout << endl << "Input total duration in seconds: ";
				cin >> T;

				// This is just me guessing now:

				tin4 = T / 4; // the time it takes to get from each pt to the next

				JOINT th1_VEL;
				JOINT th2_VEL;
				JOINT d3_VEL;
				JOINT th4_VEL;
				TRAJ th1_ACC;
				TRAJ th2_ACC;
				TRAJ d3_ACC;
				TRAJ th4_ACC;
				TRAJ th1_TB;
				TRAJ th2_TB;
				TRAJ d3_TB;
				TRAJ th4_TB;
				TRAJ th1_thB;
				TRAJ th2_thB;
				TRAJ d3_thB;
				TRAJ th4_thB;

				bool check;

				for (int i = 0; i < 4; i++)
				{
					// velocities for each linear1 section
					th1_VEL[i] = (paramArr[i + 1][0] - paramArr[i][0]) / tin4;
					th2_VEL[i] = (paramArr[i + 1][1] - paramArr[i][1]) / tin4;
					d3_VEL[i] = (paramArr[i + 1][2] - paramArr[i][2]) / tin4;
					th4_VEL[i] = (paramArr[i + 1][3] - paramArr[i][3]) / tin4;

					// Minimum Acceleration for each section for acceleration check
					th1_ACC[i] = abs(4 * (paramArr[i + 1][0] - paramArr[i][0]) / pow(tin4, 2));
					th2_ACC[i] = abs(4 * (paramArr[i + 1][1] - paramArr[i][1]) / pow(tin4, 2));
					d3_ACC[i] = abs(4 * (paramArr[i + 1][2] - paramArr[i][2]) / pow(tin4, 2));
					th4_ACC[i] = abs(4 * (paramArr[i + 1][3] - paramArr[i][3]) / pow(tin4, 2));

					// Check to ensure assigned acceleration is big enough
					check = checkacc(550, th1_ACC[i], 1);
					if (!check)
						break;
					check = checkacc(550, th2_ACC[i], 2);
					if (!check)
						break;
					check = checkacc(180, d3_ACC[i], 3);
					if (!check)
						break;
					check = checkacc(550, th4_ACC[i], 4);
					if (!check)
						break;

					th1_ACC[i] = 550;// 600 - abs(4 * (paramArr[i + 1][0] - paramArr[i][0]) / pow(tin4, 2));
					th2_ACC[i] = 550;//600 - abs(4 * (paramArr[i + 1][1] - paramArr[i][1]) / pow(tin4, 2));
					d3_ACC[i] = 180;// 200-abs(4 * (paramArr[i + 1][2] - paramArr[i][2]) / pow(tin4, 2));
					th4_ACC[i] = 550;// 600 - abs(4 * (paramArr[i + 1][3] - paramArr[i][3]) / pow(tin4, 2));

				}
				if (!check)
					break;

				// PARABOLIC BLEND METHOD

				//Calculate  values for each joint
				P_Blend(paramArr, th1_VEL, th1_ACC, tin4, 0, th1_TB, th1_thB);
				P_Blend(paramArr, th2_VEL, th2_ACC, tin4, 1, th2_TB, th2_thB);
				P_Blend(paramArr, d3_VEL, d3_ACC, tin4, 2, d3_TB, d3_thB);
				P_Blend(paramArr, th4_VEL, th4_ACC, tin4, 3, th4_TB, th4_thB);

				PrintVector(th1_VEL);

				for (int i = 0; i < 4; i++)
				{
					check = checkvel(150, th1_VEL[i], 1);
					if (!check)
						break;
					check = checkvel(150, th2_VEL[i], 2);
					if (!check)
						break;
					check = checkvel(50, d3_VEL[i], 3);
					if (!check)
						break;
					check = checkvel(150, th4_VEL[i], 4);
					if (!check)
						break;
				}
				if (!check)
					break;

				//JOINT via1_VEL = { th1_VEL[0], th2_VEL[0], d3_VEL[0], th4_VEL[0] };
				JOINT via1_VELreal;
				JOINT via1_ACC = { th1_ACC[0], th2_ACC[0], d3_ACC[0], th4_ACC[0] };

				//JOINT via2_VEL = { th1_VEL[1], th2_VEL[1], d3_VEL[1], th4_VEL[1] };
				JOINT via2_VELreal;
				JOINT via2_ACC = { th1_ACC[1], th2_ACC[1], d3_ACC[1], th4_ACC[1] };

				JOINT via3_VELreal;
				JOINT via3_ACC = { th1_ACC[2], th2_ACC[2], d3_ACC[2], th4_ACC[2] };

				JOINT goal_VELreal;
				JOINT goal_ACC = { th1_ACC[3], th2_ACC[3], d3_ACC[3], th4_ACC[3] };


				MoveToConfiguration(paramArr[0]);
				DisplayConfiguration(paramArr[0]);


				double t = 0;
				double ttemp = 0;
				JOINT pos1 = { 0 };
				int test;
				JOINT real;
				bool joint_sol;

				vector<double> joint1;
				vector<double> joint2;
				vector<double> joint3;
				vector<double> joint4;
				vector<double> realj1;
				vector<double> realj2;
				vector<double> realj3;
				vector<double> realj4;
				vector<double> velj1;
				vector<double> velj2;
				vector<double> velj3;
				vector<double> velj4;

				joint1.clear();
				joint2.clear();
				joint3.clear();
				joint4.clear();
				realj1.clear();
				realj2.clear();
				realj3.clear();
				realj4.clear();
				velj1.clear();
				velj2.clear();
				velj3.clear();
				velj4.clear();

				// blend time1 :

				while (t < tin4)
				{
					//section 1: Joint theta 1,th2,d3 & th4 to via point 1
					via1_VELreal[0] = firstreg(t, th1_TB, 0, 0, tin4, pos1, paramArr[0], th1_VEL, th1_ACC, joint1);
					via1_VELreal[1] = firstreg(t, th2_TB, 0, 1, tin4, pos1, paramArr[0], th2_VEL, th2_ACC, joint2);
					via1_VELreal[2] = firstreg(t, d3_TB, 0, 2, tin4, pos1, paramArr[0], d3_VEL, d3_ACC, joint3);
					via1_VELreal[3] = firstreg(t, th4_TB, 0, 3, tin4, pos1, paramArr[0], th4_VEL, th4_ACC, joint4);
					joint_sol = checkjointlimit(pos1);

					velj1.push_back(via1_VELreal[0]);
					velj2.push_back(via1_VELreal[1]);
					velj3.push_back(via1_VELreal[2]);
					velj4.push_back(via1_VELreal[3]);

					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					test = GetConfiguration(real);

					test = MoveWithConfVelAcc(pos1, via1_VELreal, via1_ACC);
					//cout << test;
					realj1.push_back(real[0]);
					realj2.push_back(real[1]);
					realj3.push_back(real[2]);
					realj4.push_back(real[3]);
					joint_sol = checkjointlimit(real);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/
					t = t + 0.02;
					Sleep(10);
				}
				//StopRobot();
				//ResetRobot();

				JOINT pos2 = {};
				cout << endl << "NEXT POSITION" << endl;
				while (t < 2 * tin4)
				{
					via2_VELreal[0] = linearreg(ttemp, th1_TB, 1, 0, tin4, pos2, paramArr[1], th1_VEL, th1_ACC, joint1);
					via2_VELreal[1] = linearreg(ttemp, th2_TB, 1, 1, tin4, pos2, paramArr[1], th2_VEL, th2_ACC, joint2);
					via2_VELreal[2] = linearreg(ttemp, d3_TB, 1, 2, tin4, pos2, paramArr[1], d3_VEL, d3_ACC, joint3);
					via2_VELreal[3] = linearreg(ttemp, th4_TB, 1, 3, tin4, pos2, paramArr[1], th4_VEL, th4_ACC, joint4);
					joint_sol = checkjointlimit(pos2);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					velj1.push_back(via2_VELreal[0]);
					velj2.push_back(via2_VELreal[1]);
					velj3.push_back(via2_VELreal[2]);
					velj4.push_back(via2_VELreal[3]);

					test = GetConfiguration(real);

					test = MoveWithConfVelAcc(pos2, via2_VELreal, via2_ACC);
					//cout << test;

					realj1.push_back(real[0]);
					realj2.push_back(real[1]);
					realj3.push_back(real[2]);
					realj4.push_back(real[3]);

					joint_sol = checkjointlimit(real);

					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					ttemp = ttemp + 0.02;
					t = t + 0.02;
					Sleep(10);
				}
				//StopRobot();
				//ResetRobot();
				cout << endl << "NEXT POSITION" << endl;
				JOINT pos3 = {};
				ttemp = 0;
				while (t < 3 * tin4)
				{
					via3_VELreal[0] = linearreg(ttemp, th1_TB, 2, 0, tin4, pos3, paramArr[2], th1_VEL, th1_ACC, joint1);
					via3_VELreal[1] = linearreg(ttemp, th2_TB, 2, 1, tin4, pos3, paramArr[2], th2_VEL, th2_ACC, joint2);
					via3_VELreal[2] = linearreg(ttemp, d3_TB, 2, 2, tin4, pos3, paramArr[2], d3_VEL, d3_ACC, joint3);
					via3_VELreal[3] = linearreg(ttemp, th4_TB, 2, 3, tin4, pos3, paramArr[2], th4_VEL, th4_ACC, joint4);
					joint_sol = checkjointlimit(pos3);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					velj1.push_back(via3_VELreal[0]);
					velj2.push_back(via3_VELreal[1]);
					velj3.push_back(via3_VELreal[2]);
					velj4.push_back(via3_VELreal[3]);

					test = GetConfiguration(real);
					test = MoveWithConfVelAcc(pos3, via3_VELreal, via3_ACC);
					//cout << test;

					realj1.push_back(real[0]);
					realj2.push_back(real[1]);
					realj3.push_back(real[2]);
					realj4.push_back(real[3]);
					joint_sol = checkjointlimit(real);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/
					ttemp = ttemp + 0.02;
					t = t + 0.02;
					Sleep(10);
				}
				//StopRobot();
				//ResetRobot();
				cout << endl << "NEXT POSITION" << endl;
				JOINT pos4 = {};
				ttemp = 0;
				while (t < 4 * tin4)
				{
					goal_VELreal[0] = lastreg(ttemp, th1_TB, 3, 0, tin4, pos4, paramArr[3], th1_VEL, th1_ACC, joint1);
					goal_VELreal[1] = lastreg(ttemp, th2_TB, 3, 1, tin4, pos4, paramArr[3], th2_VEL, th2_ACC, joint2);
					goal_VELreal[2] = lastreg(ttemp, d3_TB, 3, 2, tin4, pos4, paramArr[3], d3_VEL, d3_ACC, joint3);
					goal_VELreal[3] = lastreg(ttemp, th4_TB, 3, 3, tin4, pos4, paramArr[3], th4_VEL, th4_ACC, joint4);
					joint_sol = checkjointlimit(pos4);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					velj1.push_back(goal_VELreal[0]);
					velj2.push_back(goal_VELreal[1]);
					velj3.push_back(goal_VELreal[2]);
					velj4.push_back(goal_VELreal[3]);

					test = GetConfiguration(real);

					test = MoveWithConfVelAcc(pos4, goal_VELreal, goal_ACC);
					//cout << test;

					realj1.push_back(real[0]);
					realj2.push_back(real[1]);
					realj3.push_back(real[2]);
					realj4.push_back(real[3]);
					joint_sol = checkjointlimit(real);
					/*if (!joint_sol)
					{
						cout << endl << "Aborting Program..." << endl;
						break;
					}*/

					ttemp = ttemp + 0.02;
					t = t + 0.02;
					Sleep(10);
				}
				test = GetConfiguration(real);

				realj1.push_back(real[0]);
				realj2.push_back(real[1]);
				realj3.push_back(real[2]);
				realj4.push_back(real[3]);

				joint_sol = checkjointlimit(real);

				/*if (!joint_sol)
				{
					cout << endl << "Aborting Program..." << endl;
					break;
				}*/

				StopRobot();
				ResetRobot();

				ofstream th1_values;
				th1_values.open("test.txt");
				if (th1_values.is_open())
				{
					th1_values << "Theta1 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << paramArr[i][0] << " ";
					}
					th1_values << "];" << endl << "Theta2 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << paramArr[i][1] << " ";
					}
					th1_values << "];" << endl << "d3 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << paramArr[i][2] << " ";
					}
					th1_values << "];" << endl << "Theta4 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << paramArr[i][3] << " ";
					}
					th1_values << "];" << endl << "Acceleration_Theta1 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th1_ACC[i] << " ";
					}
					th1_values << "];" << endl << "Acceleration_Theta2 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th2_ACC[i] << " ";
					}
					th1_values << "];" << endl << "Acceleration_d3 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << d3_ACC[i] << " ";
					}
					th1_values << "];" << endl << "Acceleration_Theta4 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th4_ACC[i] << " ";
					}
					th1_values << "];" << endl << "Velocity_Theta1 = [";
					for (int i = 0; i < 4; i++)
					{
						th1_values << th1_VEL[i] << " ";
					}
					th1_values << "];" << endl << "Velocity_Theta2 = [";
					for (int i = 0; i < 4; i++)
					{
						th1_values << th2_VEL[i] << " ";
					}
					th1_values << "];" << endl << "Velocity_d3 = [";
					for (int i = 0; i < 4; i++)
					{
						th1_values << d3_VEL[i] << " ";
					}
					th1_values << "];" << endl << "Velocity_Theta4 = [";
					for (int i = 0; i < 4; i++)
					{
						th1_values << th4_VEL[i] << " ";
					}
					th1_values << "];" << endl << "Tb_Theta1 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th1_TB[i] << " ";
					}
					th1_values << "];" << endl << "Tb_Theta2 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th2_TB[i] << " ";
					}
					th1_values << "];" << endl << "Tb_d3 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << d3_TB[i] << " ";
					}
					th1_values << "];" << endl << "Tb_Theta4 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th4_TB[i] << " ";
					}
					th1_values << "];" << endl << "Thetab_Theta1 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th1_thB[i] << " ";
					}
					th1_values << "];" << endl << "Thetab_Theta2 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th2_thB[i] << " ";
					}
					th1_values << "];" << endl << "Thetab_d3 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << d3_thB[i] << " ";
					}
					th1_values << "];" << endl << "Thetab_Theta4 = [";
					for (int i = 0; i < 5; i++)
					{
						th1_values << th4_thB[i] << " ";
					}
					th1_values << "];" << endl << "Timetotal = " << T;
					th1_values << endl << endl << endl << "Joint1 = [";
					for (int i = 0; i < joint1.size(); i++)
					{
						th1_values << joint1[i] << " ";
					}
					th1_values << "]" << endl << endl << "Joint2 = [";
					for (int i = 0; i < joint2.size(); i++)
					{
						th1_values << joint2[i] << " ";
					}
					th1_values << "]" << endl << endl << "Joint3 = [";
					for (int i = 0; i < joint3.size(); i++)
					{
						th1_values << joint3[i] << " ";
					}
					th1_values << "]" << endl << endl << "Joint4 = [";
					for (int i = 0; i < joint4.size(); i++)
					{
						th1_values << joint4[i] << " ";
					}
					th1_values << "]" << endl << endl << "RealJ1 = [";
					for (int i = 0; i < realj1.size(); i++)
					{
						th1_values << realj1[i] << " ";
					}
					th1_values << "]" << endl << endl << "RealJ2 = [";
					for (int i = 0; i < realj2.size(); i++)
					{
						th1_values << realj2[i] << " ";
					}
					th1_values << "]" << endl << endl << "RealJ3 = [";
					for (int i = 0; i < realj3.size(); i++)
					{
						th1_values << realj3[i] << " ";
					}
					th1_values << "]" << endl << endl << "RealJ4 = [";
					for (int i = 0; i < realj4.size(); i++)
					{
						th1_values << realj4[i] << " ";
					}
					th1_values << "]" << endl << endl << "VelJ1 = [";
					for (int i = 0; i < velj1.size(); i++)
					{
						th1_values << velj1[i] << " ";
					}
					th1_values << "]" << endl << endl << "VelJ2 = [";
					for (int i = 0; i < velj2.size(); i++)
					{
						th1_values << velj2[i] << " ";
					}
					th1_values << "]" << endl << endl << "VelJ3 = [";
					for (int i = 0; i < velj3.size(); i++)
					{
						th1_values << velj3[i] << " ";
					}
					th1_values << "]" << endl << endl << "VelJ4 = [";
					for (int i = 0; i < velj4.size(); i++)
					{
						th1_values << velj4[i] << " ";
					}
					th1_values << "]";
					joint1.clear();
					joint2.clear();
					joint3.clear();
					joint4.clear();
					realj1.clear();
					realj2.clear();
					realj3.clear();
					realj4.clear();

				}
				th1_values.close();
			}

			JOINT q1 = { 90, 0, -175, 0 };
			//JOINT q2 = {90, 90, -200, 45}; 


			printf("Press '1' to restart robot\n");
			printf("Press any key to continue \n");
			printf("Press ESC to exit \n");

			ch = _getch();

			if (ch == '1')
			{
				MoveToConfiguration(q1);
				DisplayConfiguration(q1);
				//ResetRobot();
			}
			//else if (c == '2')
			//{
			//	//MoveToConfiguration(JointVect);
			//	//DisplayConfiguration(JointVect);
			//}

			c = ch;
		}
		else
			break;


	}


	return 0;
}

bool checkvel(double limit, double value, int jointnum)
{
	if (value >= limit || value <= (-1 * limit))
	{
		cout << "ERROR: Velocity of Joint " << jointnum << " is exceeded in trajectory : Aborting Program" << endl;
		return false;
	}
	return true;
};

bool checkacc(double value, double minacc, int jointnum)
{
	if (value < minacc)
	{
		cout << "ERROR: Acceleration of Joint " << jointnum << " is too small in trajectory : Aborting Program" << endl;
		return false;
	}
	return true;
};

bool checkjointlimit(JOINT& pos)
{
	//near1 = JOINT
	//CHECK LIMITS
	bool sol = true;
	if (pos[0] > LIM1upper)
	{
		cout << endl << "ERROR: joint limit for theta1 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[0] = LIM1upper;
		//StopRobot();
		sol = false;
	}
	else if (pos[0] < LIM1lower)
	{
		cout << endl << "ERROR: joint limit for theta1 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[0] = LIM1lower;
		//StopRobot();
		sol = false;
	}

	if (pos[1] > LIM2upper)
	{
		cout << endl << "ERROR: joint limit for theta2 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[1] = LIM2upper;
		//StopRobot();
		sol = false;
	}
	else if (pos[1] < LIM2lower)
	{
		cout << endl << "ERROR: joint limit for theta2 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[1] = LIM2lower;
		//StopRobot();
		sol = false;
	}

	if (pos[2] > LIM3upper)
	{
		cout << endl << "ERROR: joint limit for d3 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[2] = LIM3upper;
		//StopRobot();
		sol = false;
	}
	else if (pos[2] < LIM3lower)
	{
		cout << endl << "ERROR: joint limit for d3 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[2] = LIM3lower;
		//StopRobot();
		sol = false;
	}

	if (pos[3] > LIM4upper)
	{
		cout << endl << "ERROR: joint limit for theta4 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[3] = LIM4upper;
		//StopRobot();
		sol = false;
	}
	else if (pos[3] < LIM4lower)
	{
		cout << endl << "ERROR: joint limit for theta4 exceeded" << endl;
		cout << "Not allowing movement past limit" << endl;
		pos[3] = LIM4lower;
		//StopRobot();
		sol = false;
	}

	return sol;
}

double firstreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack)
{
	double velreal;
	if (t <= TB[i])
	{
		//double tinb = t - (TB[i] + (tin4 - TB[i] - TB[i + 1]));
		//pos[joint] = posVect[joint] + (vel[i] * (t - tinb)) + (0.5 * acc[i+1] * pow(tinb, 2)); //uses: t
		pos[joint] = posVect[joint] + (vel[i] * t) + (0.5 * acc[i] * pow(t, 2)); //uses: t
		keeptrack.push_back(pos[joint]);
		// Acceleration value may be wrong lolz
		velreal = vel[i] + acc[i] * t;
	}
	else if (t < tin4 - 0.5 * TB[i + 1])
	{
		pos[joint] = posVect[joint] + vel[i] * t; //linear1
		keeptrack.push_back(pos[joint]);
		velreal = vel[i];
	}
	else
	{
		double tinb = t - (0.5 * TB[i] + (tin4 - 0.5 * TB[i] - 0.5 * TB[i + 1]));
		pos[joint] = posVect[joint] + vel[i] * (t - tinb) + 0.5 * acc[i + 1] * pow(tinb, 2);//uses: t       
		keeptrack.push_back(pos[joint]);
		velreal = vel[i] + acc[i + 1] * tinb;
	}
	return velreal;
};

double lastreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack)
{
	double velreal;
	if (t <= 0.5 * TB[i])
	{
		//double tinb = t - (TB[i] + (tin4 - TB[i] - TB[i + 1]));
		//pos[joint] = posVect[joint] + (vel[i] * (t - tinb)) + (0.5 * acc[i+1] * pow(tinb, 2)); //uses: t
		pos[joint] = posVect[joint] + (vel[i] * t) + (0.5 * acc[i] * pow(t, 2)); //uses: t
		keeptrack.push_back(pos[joint]);
		// Acceleration value may be wrong lolz
		velreal = vel[i] + acc[i] * t;
	}
	else if (t < tin4 - TB[i + 1])
	{
		pos[joint] = posVect[joint] + vel[i] * t; //linear1
		keeptrack.push_back(pos[joint]);
		velreal = vel[i];
	}
	else
	{
		double tinb = t - (0.5 * TB[i] + (tin4 - 0.5 * TB[i] - 0.5 * TB[i + 1]));
		pos[joint] = posVect[joint] + vel[i] * (t - tinb) + 0.5 * acc[i + 1] * pow(tinb, 2);//uses: t       
		keeptrack.push_back(pos[joint]);
		velreal = vel[i] + acc[i + 1] * tinb;
	}
	return velreal;
};

double linearreg(double t, TRAJ TB, int i, int joint, double tin4, JOINT& pos, JOINT posVect, TRAJ vel, TRAJ acc, vector<double>& keeptrack)
{
	double velreal;
	if (t <= 0.5 * TB[i])
	{
		//double tinb = t - (TB[i] + (tin4 - TB[i] - TB[i + 1]));
		//pos[joint] = posVect[joint] + (vel[i] * (t - tinb)) + (0.5 * acc[i+1] * pow(tinb, 2)); //uses: t
		pos[joint] = posVect[joint] + (vel[i] * t) + (0.5 * acc[i] * pow(t, 2)); //uses: t
		keeptrack.push_back(pos[joint]);
		// Acceleration value may be wrong lolz
		velreal = vel[i] + acc[i] * t;
	}
	else if (t < tin4 - 0.5 * TB[i + 1])
	{
		pos[joint] = posVect[joint] + vel[i] * t; //linear1
		keeptrack.push_back(pos[joint]);
		velreal = vel[i];
	}
	else
	{
		double tinb = t - (0.5 * TB[i] + (tin4 - 0.5 * TB[i] - 0.5 * TB[i + 1]));
		pos[joint] = posVect[joint] + vel[i] * (t - tinb) + 0.5 * acc[i + 1] * pow(tinb, 2);//uses: t       
		keeptrack.push_back(pos[joint]);
		velreal = vel[i] + acc[i + 1] * tinb;
	}
	return velreal;
};

void P_Blend(JOINT paramArr[5], JOINT& th_vel, TRAJ& th_ACC, double tin4, int order, TRAJ& joint_tb, TRAJ& joint_thb)
{
	for (int i = 0; i < 4; i++)
	{
		if (i == 0)
		{
			th_ACC[i] = sgn(paramArr[i + 1][order], paramArr[i][order]) * abs(th_ACC[i]);
		}
		else if (i == 3)
		{
			th_ACC[i + 1] = sgn(paramArr[i][order], paramArr[i + 1][order]) * abs(th_ACC[i]);
			th_ACC[i] = sgn(th_vel[i], th_vel[i - 1]) * abs(th_ACC[i]);
		}
		else
		{
			th_ACC[i] = sgn(th_vel[i], th_vel[i - 1]) * abs(th_ACC[i]);
		}
	}

	// Calculate all TB values for the inputted joint values in the trajectory
	// Order variable is for which part of the array to use
	joint_tb[0] = tin4 - sqrt(pow(tin4, 2) - (2 * (paramArr[1][order] - paramArr[0][order])) / th_ACC[0]); //tb = tb of the very first section (theta1)
	joint_tb[4] = tin4 - sqrt(pow(tin4, 2) + (2 * (paramArr[4][order] - paramArr[3][order])) / th_ACC[4]); //this is the tb for the last section!aka tn/tlast

	//initial and final velocities:                          
	th_vel[0] = (paramArr[1][order] - paramArr[0][order]) / (tin4 - 0.5 * joint_tb[0]);
	th_vel[3] = (paramArr[4][order] - paramArr[3][order]) / (tin4 - 0.5 * joint_tb[4]);

	th_ACC[3] = sgn(th_vel[3], th_vel[2]) * abs(th_ACC[3]);

	//tb -> after via1 **EQN 7.24 from txtbk aka tb for interior points
	joint_tb[1] = ((th_vel[1] - th_vel[0]) / th_ACC[1]);
	joint_tb[2] = ((th_vel[2] - th_vel[1]) / th_ACC[2]);// / 2;
	joint_tb[3] = ((th_vel[3] - th_vel[2]) / th_ACC[3]);// / 2;
	//Last blend time

	for (int i = 0; i < 5; i++) // if joints are in same position
	{
		if (abs(th_ACC[i]) < 0.0001)
		{
			th_ACC[i] = 0;
			joint_tb[i] = 0;
			if (i == 0)
			{
				th_vel[0] = 0;
				joint_tb[1] = ((th_vel[1] - th_vel[0]) / th_ACC[1]) / 2;
			}
			else if (i == 4)
			{
				th_vel[3] = 0;
				joint_tb[3] = ((th_vel[3] - th_vel[2]) / th_ACC[3]) / 2;
			}
		}
	}

	// Calculate all Thetab vlues for the inputed joint values in the trajectory
	// Order variable is for which part of the array to use
	//all theta b's are calculated using the same eqn 7.20
	joint_thb[0] = paramArr[0][order] + (0.5 * th_ACC[0] * pow(joint_tb[0], 2)); //thb [0] = theta b of inital
	joint_thb[1] = paramArr[1][order] + (0.5 * th_ACC[1] * pow(joint_tb[1], 2)); //thb [1] = theta b of via1
	joint_thb[2] = paramArr[2][order] + (0.5 * th_ACC[2] * pow(joint_tb[2], 2)); //thb [2] = theta b of via2
	joint_thb[3] = paramArr[3][order] + (0.5 * th_ACC[3] * pow(joint_tb[3], 2)); //thb [3] = theta b of via3
	joint_thb[4] = paramArr[4][order] + (0.5 * th_ACC[4] * pow(joint_tb[4], 2)); //thb[4] = theta b for goal 

};

void writeData(ofstream& th_values, JOINT paramArr[5], JOINT th_ACC, TRAJ th_tb, TRAJ th1_thB, double T)
{
	for (int i = 0; i < 4; i++)
	{
		th_values << "Thetazero = [";
		th_values << paramArr[i][0] << " "; ///saves theta 0
	}
	th_values << "];" << endl << "ThetaF = [";
	for (int i = 1; i < 5; i++)
	{
		th_values << paramArr[i][0] << " "; //saves via1,via2,via3, etc. values
	}
	th_values << "];" << endl << "Acceleration = [";
	for (int i = 0; i < 5; i++)
	{
		th_values << th_ACC[i] << " "; //saves acceleration values
	}
	th_values << "];" << endl << "Tb = [";
	for (int i = 0; i < 5; i++)
	{
		th_values << th_tb[i] << " "; //saves tb values
	}
	th_values << "];" << endl << "Thetab = [";
	for (int i = 0; i < 5; i++)
	{
		th_values << th1_thB[i] << " "; //saves thetab values
	}
	th_values << "];" << endl << "Timetotal = " << T;
};

void UTOI(JOINT vec4, FRAME& T)
{
	//isolate the angle from the given vector:
	double angle = vec4[3];
	//degrees -> radians
	angle = angle * PI / 180;

	// get the rotation matrix by defn (rotating around z-axis)
	double tempM[4][4] = { {cos(angle), -sin(angle) ,0, vec4[0]},
		{sin(angle), cos(angle), 0, vec4[1]},
		{0, 0, 1, vec4[2]}, {0, 0, 0, 1} };

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T[i][j] = tempM[i][j];
		}
	}
};

void ITOU(FRAME T, JOINT& vec4)
{
	// get the x y z coordinates from the T matrix
	vec4[0] = T[0][3];
	vec4[1] = T[1][3];
	vec4[2] = T[2][3];

	// get angle
	double angle = atan2(T[1][0], T[0][0]);
	angle = RAD2DEG(angle);
	vec4[3] = angle;
};

void TMULT(FRAME brela, FRAME crelb, FRAME& crela)
{

	for (int i = 0; i < 4; i++)
	{
		for (int k = 0; k < 4; k++)
		{
			crela[i][k] = 0;
			for (int j = 0; j < 4; j++)
			{
				crela[i][k] += brela[i][j] * crelb[j][k];
			}

		}
	}
};

void TINVERT(FRAME T, FRAME& Tinv)
{
	//Extract Rotation Matrix
	//Extract Position Vector
	rot Rotation{};
	position P{};
	//Initialize Row of T matrix
	JOINT Row4{ 0, 0, 0, 1 };
	rot RotationT{};
	position RotTP{};

	for (int i = 0; i < 3; i++)
	{
		P[i] = T[i][3];
		for (int j = 0; j < 3; j++)
		{
			Rotation[i][j] = T[i][j];
		}
	}

	//Transpose Rotation Matrix
	for (int k = 0; k < 3; k++)
	{
		for (int l = 0; l < 3; l++)
		{
			RotationT[k][l] = Rotation[l][k];
		}
	}

	//Create new position vector
	for (int n = 0; n < 3; n++)
	{
		for (int o = 0; o < 3; o++)
		{
			RotTP[n] += (-1 * RotationT[n][o]) * P[o];
		}
	}

	//Assemble T inverse
	for (int p = 0; p < 4; p++)
	{
		for (int q = 0; q < 4; q++)
		{
			if (p < 3 && q < 3)
			{
				Tinv[p][q] = RotationT[p][q];
			}
			if (q == 3)
			{
				Tinv[p][q] = RotTP[p];
			}
			if (p == 3)
			{
				Tinv[p][q] = Row4[q];
			}
		}
	}
}

void KIN(JOINT vec3, FRAME& wrelb)
{
	double theta1 = vec3[0]; // left in degrees for now
	double theta2 = vec3[1]; // left in degrees
	double d3 = vec3[2];
	double theta4 = vec3[3];

	//CHECK LIMITS FOR THETA1
	if (theta1 > LIM1upper)
	{
		theta1 = DEG2RAD(LIM1upper);
		cout << endl << "ERROR: joint limit for theta1 exceeded" << endl;
		cout << "Reassigning to max joint value" << endl;
	}
	else if (theta1 < LIM1lower)
	{
		theta1 = DEG2RAD(LIM1lower);
		cout << endl << "ERROR: joint limit for theta1 exceeded" << endl;
		cout << "Reassigning to max joint value" << endl;
	}
	else
	{
		theta1 = DEG2RAD(theta1);
	}
	//CHECK LIMITS FOR THETA2
	if (theta2 > LIM2upper)
	{
		theta2 = DEG2RAD(LIM2upper);
		cout << endl << "ERROR: joint limit for theta2 exceeded" << endl;
		cout << "Reassigning to max joint value" << endl;
	}
	else if (theta2 < LIM2lower)
	{
		theta2 = DEG2RAD(LIM2lower);
		cout << endl << "ERROR: joint limit for theta2 exceeded" << endl;
		cout << "Reassigning to min joint value" << endl;
	}
	else
	{
		theta2 = DEG2RAD(theta2);
	}
	//CHECK LIMITS FOR D3
	if (d3 > LIM3upper)
	{
		d3 = LIM3upper;
		cout << endl << "ERROR: joint limit for d3 exceeded" << endl;
		cout << "Reassigning to max joint value" << endl;
	}
	else if (d3 < LIM3lower)
	{
		d3 = LIM3lower;
		cout << endl << "ERROR: joint limit for d3 exceeded" << endl;
		cout << "Reassigning to min joint value" << endl;
	}
	//CHECK LIMITS FOR THETA4
	if (theta4 > LIM4upper)
	{
		theta4 = DEG2RAD(LIM4upper);
		cout << endl << "ERROR: joint limit for theta4 exceeded" << endl;
		cout << "Reassigning to max joint value" << endl;
	}
	else if (theta4 < LIM4lower)
	{
		theta4 = DEG2RAD(LIM4lower);
		cout << endl << "ERROR: joint limit for theta4 exceeded" << endl;
		cout << "Reassigning to min joint value" << endl;
	}
	else
	{
		theta4 = DEG2RAD(theta4);
	}

	// B=1 and W=4
	FRAME T12 = { {cos(theta1), -sin(theta1),0,0}, {sin(theta1),cos(theta1),0,0 }, { 0,0,1,L1 }, { 0,0,0,1 } };
	FRAME T23 = { {cos(theta2), -sin(theta2),0,L3}, {sin(theta2),cos(theta2),0,0 }, { 0,0,1,0 }, { 0,0,0,1 } };
	FRAME T34 = { {cos(theta4), -sin(theta4),0,L4}, {-sin(theta4),-cos(theta4),0,0 }, { 0,0,-1,-(EXTENDED - L2 + L7 + d3) }, { 0,0,0,1 } }; // 420+d3
	FRAME T45 = { {1,0,0,0}, {0,1,0,0}, {0,0,1,(L6 - L7 + L8 / 2)}, {0,0,0,1} }; // = 55

	//multiply the matrix all together
	FRAME T13;
	FRAME T14;

	TMULT(T12, T23, T13);
	TMULT(T13, T34, T14);
	TMULT(T14, T45, wrelb); //wrelb is T15
	JOINT resultingJV = { theta1, theta2, d3, theta4 };
	JOINT resultingPV;
	ITOU(wrelb, resultingPV);
	cout << endl << "Resulting position vector: ";
	PrintVector(resultingPV);
	cout << endl << "Moving to Joint Vector" << endl;
	MoveToConfiguration(resultingJV);
	DisplayConfiguration(resultingJV);
};

void WHERE(JOINT posV, FRAME& trels, JOINT& resultPos)
{
	//display graphically:
	JOINT q;
	for (int i = 0; i < 4; i++)
	{
		q[i] = posV[i];
	}
	//MoveToConfiguration(q);
	//DisplayConfiguration(q);

	//multiply the matrix all together
	FRAME T04;
	KIN(posV, T04);

	FRAME TS4;
	TMULT(brels, T04, TS4);
	TMULT(TS4, trelw, trels); // trels is TS5 which is tool wrt station

	ITOU(trels, resultPos);

	cout << "Resulting position and orientation:" << endl;
	PrintVector(resultPos);

};

void INVKIN(FRAME wrelb, JOINT current, JOINT& near1, JOINT& far1, bool& sol)
{
	JOINT realsolution = { 0 };
	double x, y, z, phi;
	ITOU(wrelb, realsolution);
	//cout << "Real Solution" << endl;
	//PrintVector(realsolution);

	x = realsolution[0];
	y = realsolution[1];
	z = realsolution[2];
	phi = realsolution[3];

	double Zneutral = (L1 - (EXTENDED - L2 + L6 + L8 / 2));
	double Zupper = Zneutral - LIM3lower;
	double Zlower = Zneutral - LIM3upper;

	sol = true;
	/*if (z < Zlower)
	{
		cout << "**" << endl << "ERROR: z value is below range of robot." << endl;
		cout << "Reassigning z value to 30 mm" << endl << "**" << endl;
		z = 30;
	}
	else if (z > Zupper)
	{
		cout << "**" << endl << "ERROR: z value is above range of robot." << endl;
		cout << "Reassigning z value to 130 mm" << endl << "**" << endl;
		z = 130;
	}*/
	if (sqrt(x * x + y * y) < (L3 - L4))
	{
		sol = false;
		//must lie outside of a circle of radius l1-l2 in order for a solution ot exist
	}

	if (sqrt(x * x + y * y) > (L3 + L4))
	{
		sol = false;
		//there is no solution because the end defector position is inside the possible length
		//must lie within a circle of radius l1+l2 in order for a solution to exist

	}

	double c2 = ((x * x + y * y) - (L3 * L3) - (L4 * L4)) / (2 * L4 * L3);
	double s2 = sqrt(1 - (c2 * c2));
	double positivetheta2 = atan2(s2, c2);
	double negativetheta2 = atan2((-1 * s2), c2);

	if (x == 0 && y == 0)
	{
		c2 = (-1 * L2) / ((L1));
		positivetheta2 = 0;
		negativetheta2 = PI;
	}

	//Convert to Degrees
	double theta2Finalpos = RAD2DEG(positivetheta2);
	double theta2Finalneg = RAD2DEG(negativetheta2);
	double k11 = L3 + L4 * cos(positivetheta2);
	double k12 = L3 + L4 * cos(negativetheta2);
	double k21 = L4 * sin(positivetheta2);
	double k22 = L4 * sin(negativetheta2);
	double theta11 = atan2(y, x) - atan2(k21, k11);
	double theta12 = atan2(y, x) - atan2(k22, k12);
	double theta1Finalpos = RAD2DEG(theta11);
	double theta1Finalneg = RAD2DEG(theta12);
	double theta4pos = theta1Finalpos + theta2Finalpos - phi;
	double theta4neg = theta1Finalneg + theta2Finalneg - phi;

	JOINT diff{};
	JOINT diff2{};
	JOINT resultpos{};
	JOINT resultneg{};

	resultpos[0] = theta1Finalpos;
	resultpos[1] = theta2Finalpos;
	resultpos[2] = Zneutral - z;
	resultpos[3] = theta4pos;

	resultneg[0] = theta1Finalneg;
	resultneg[1] = theta2Finalneg;
	resultneg[2] = Zneutral - z;
	resultneg[3] = theta4neg;

	//Check which solution is closer
	for (int i = 0; i < 4; i++)
	{
		if (current[i] == 0)
		{
			diff[i] = resultpos[i];
		}
		else
		{
			diff[i] = abs((resultpos[i] - current[i]) / current[i]);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (current[i] == 0)
		{
			diff[i] = resultneg[i];
		}
		else
		{
			diff2[i] = abs((resultneg[i] - current[i]) / current[i]);
		}

	}

	//total distance postitive solution
	double eptotal1 = diff[0] + diff[1] + diff[2] + diff[3];
	//total distance negative solution
	double eptotal2 = diff2[0] + diff2[1] + diff2[2] + diff2[3];

	if (eptotal1 < eptotal2)
	{
		//initialize near1
		for (int i = 0; i < 4; i++)
		{
			near1[i] = resultpos[i];
		}
		for (int i = 0; i < 4; i++)
		{
			far1[i] = resultneg[i];
		}
	}
	else if (eptotal2 < eptotal1)
	{
		for (int i = 0; i < 4; i++)
		{
			near1[i] = resultneg[i];
		}
		for (int i = 0; i < 4; i++)
		{
			far1[i] = resultpos[i];
		}
	}
	else if (eptotal2 == eptotal1)
	{
		cout << "near1 and far1 vectors are the same" << endl;
		for (int i = 0; i < 4; i++)
		{
			near1[i] = resultpos[i];
			far1[i] = resultneg[i];
		}

	}

	//CHECK LIMITS
	if (near1[0] > LIM1upper || near1[0] < LIM1lower)
	{
		cout << endl << "ERROR: joint limit for theta1 exceeded" << endl;
		sol = false;
	}
	else if (near1[1] > LIM2upper || near1[1] < LIM2lower)
	{
		cout << endl << "ERROR: joint limit for theta2 exceeded" << endl;
		sol = false;
	}
	else if (near1[2] > LIM3upper || near1[2] < LIM3lower)
	{
		cout << endl << "ERROR: joint limit for d3 exceeded" << endl;
		sol = false;
	}
	else if (near1[3] > LIM4upper || near1[3] < LIM4lower)
	{
		cout << endl << "ERROR: joint limit for theta4 exceeded" << endl;
		sol = false;
	}

};

void SOLVE(FRAME& wrelb, JOINT& current, JOINT& near1, JOINT& far1, bool& sol)
{
	//trels = T frame relative to S frame
	//T frame and S frame should be globally defined variables
	//Solve should use TMULT, TINVERT and INVKIN

	//Ask for desired joint from user input
	JOINT desired = { 0 };
	cout << endl << "Enter Position Vector Parameters." << endl;
	cout << "x: ";
	cin >> desired[0];
	cout << "y: ";
	cin >> desired[1];
	cout << "z: ";
	cin >> desired[2];
	cout << "phi: ";
	cin >> desired[3];

	UTOI(desired, wrelb);

	//Calculate joint parameters
	INVKIN(wrelb, current, near1, far1, sol);

	if (sol == false)
	{
		cout << "Coordinates exists outside of the range of the robot.";
		cout << endl << "SOLUTION DOES NOT EXIST" << endl;
	}
	else
	{
		//Move to robot to near1
		cout << endl << "CURRENT: ";
		PrintVector(current);
		cout << "far1: ";
		PrintVector(far1);
		cout << "near1: ";
		PrintVector(near1);
		cout << "Moving to near1 Joint Vector" << endl << endl;
		MoveToConfiguration(near1);
		DisplayConfiguration(near1);
	}
}

void SOLVE2(FRAME trels, JOINT& current, JOINT& near1, JOINT& far1, bool& sol)
{
	//trels = T frame relative to S frame
	//T frame and S frame should be globally defined variables
	//Solve should use TMULT, TINVERT and INVKIN
	//Calculate Tool wrt to wrist frame

	FRAME wrelt = { 0 };
	TINVERT(trelw, wrelt);

	//Multiple trels by wrelt to get wrels
	FRAME wrels = { 0 }; //wrist frame wrt to station frame
	TMULT(trels, wrelt, wrels); //wrelsT is wrist relative to tool frame 

	//Multiple srelb by wrels to get wrelb
	FRAME wrelb = { 0 };
	FRAME srelb;
	TINVERT(brels, srelb);
	TMULT(srelb, wrels, wrelb);

	//Calculate joint parameters
	INVKIN(wrelb, current, near1, far1, sol);

	if (sol == false)
	{
		cout << "No solution" << endl;
	}
	else
	{
		//Move to robot to near1
		cout << "Moving to solution: " << near1[0] << ", " << near1[1] << ", " << near1[2] << ", " << near1[3] << endl;
		MoveToConfiguration(near1);
		DisplayConfiguration(near1);
	}
};

void PrintMatrix(FRAME matrix)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << matrix[i][j] << " ";
		}
		cout << endl;
	}
};

void PrintVector(JOINT vector)
{
	for (int i = 0; i < 4; i++)
	{
		cout << vector[i] << " ";
	}
	cout << endl;
};

void PrintVector2(TRAJ vector)
{
	for (int i = 0; i < 5; i++)
	{
		cout << vector[i] << " ";
	}
	cout << endl;
};

void current2deg(JOINT& current)
{
	current[0] = RAD2DEG(current[0]);
	current[1] = RAD2DEG(current[1]);
	current[3] = RAD2DEG(current[3]);
};

int sgn(double theta2, double theta1)
{
	if ((theta2 - theta1) < 0)
	{
		return -1;
	}
	else if ((theta2 - theta1) > 0)
	{
		return 1;
	}
	else
		return 1;
};