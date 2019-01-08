#ifndef ALLEGRO_HAND_INTERFACE_H
#define ALLEGRO_HAND_INTERFACE_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"

typedef char TCHAR;

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

Eigen::VectorXd GetJointPosition(void);
void SetDesiredJointPosition(Eigen::VectorXd q_desired);


#endif

