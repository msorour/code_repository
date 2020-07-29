#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "../include/canAPI.h"
#include "../include/rDeviceAllegroHandCANDef.h"
#include "../include/interface.h"
#include <BHand/BHand.h>

#include "../include/Eigen/Dense"
#include <iostream>
#include "../include/allegro_hand_parameters.h"
#include "../include/useful_implementations.h"
#include "../include/AllegroRightHandModel.h"
#include <time.h>
/*
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
*/
typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

//using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 3;

const double tau_cov_const_v2 = 800.0; // 800.0 for SAH020xxxxx
const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx

const double enc_dir[MAX_DOF]   = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
const double motor_dir[MAX_DOF] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
const int enc_offset[MAX_DOF]   = { -1591, -277, 545, 168, -904, 53, -233, -1476, 2, -987, -230, -106, -1203, 361, 327, 565 };

const int enc_offset_calibrated[MAX_DOF] = { 34269, 34327, 34660, 32593, 32820, 32818, 32816, 33049, 30856, 33227, 32754, 32759, 31955, 32752, 33200, 31242 };




Eigen::VectorXd q_error(16), q_desired(16), q_current(16), torque_desired(16);
Eigen::VectorXd q_min(16), q_mean(16), q_home(16);

Eigen::Vector4d index_q_des,  index_q_err,  index_torque_cmd;
Eigen::Vector4d middle_q_des, middle_q_err, middle_torque_cmd;
Eigen::Vector4d pinky_q_des,  pinky_q_err,  pinky_torque_cmd;
Eigen::Vector4d thumb_q_des,  thumb_q_err,  thumb_torque_cmd;

Eigen::Vector4d index_q_des_mean, middle_q_des_mean, pinky_q_des_mean, thumb_q_des_mean;
Eigen::Vector4d index_q_des_home, middle_q_des_home, pinky_q_des_home, thumb_q_des_home;
Eigen::Vector4d index_q_des_min, middle_q_des_min, pinky_q_des_min, thumb_q_des_min;
Eigen::Vector4d index_q_des_max, middle_q_des_max, pinky_q_des_max, thumb_q_des_max;



// ALLEGRO HAND
Eigen::Matrix4d thumb_DGM, index_DGM, middle_DGM, pinky_DGM;
Eigen::MatrixXd thumb_position_jacobian(3,4), index_position_jacobian(3,4), middle_position_jacobian(3,4), pinky_position_jacobian(3,4);

Eigen::Vector4d thumb_joint_position_desired,  thumb_joint_velocity_desired,  thumb_joint_torque_command,  thumb_joint_velocity_command,  thumb_joint_position,  thumb_joint_velocity;
Eigen::Vector4d index_joint_position_desired,  index_joint_velocity_desired,  index_joint_torque_command,  index_joint_velocity_command,  index_joint_position,  index_joint_velocity;
Eigen::Vector4d middle_joint_position_desired, middle_joint_velocity_desired, middle_joint_torque_command, middle_joint_velocity_command, middle_joint_position, middle_joint_velocity;
Eigen::Vector4d pinky_joint_position_desired,  pinky_joint_velocity_desired,  pinky_joint_torque_command,  pinky_joint_velocity_command,  pinky_joint_position,  pinky_joint_velocity;

Eigen::VectorXd pose_rpy_Ptt(6), pose_rpy_Ptt_desired(6), pose_rpy_Ptt_init(6), pose_rpy_Ptt_traj(6), pose_rpy_Ptt_error(6);
Eigen::VectorXd pose_rpy_Pit(6), pose_rpy_Pit_desired(6), pose_rpy_Pit_init(6), pose_rpy_Pit_traj(6), pose_rpy_Pit_error(6);
Eigen::VectorXd pose_rpy_Pmt(6), pose_rpy_Pmt_desired(6), pose_rpy_Pmt_init(6), pose_rpy_Pmt_traj(6), pose_rpy_Pmt_error(6);
Eigen::VectorXd pose_rpy_Ppt(6), pose_rpy_Ppt_desired(6), pose_rpy_Ppt_init(6), pose_rpy_Ppt_traj(6), pose_rpy_Ppt_error(6);

Eigen::VectorXd velocity_Ptt(6), velocity_Ptt_desired(6), velocity_Ptt_init(6), velocity_Ptt_traj(6), velocity_Ptt_error(6);
Eigen::VectorXd velocity_Pit(6), velocity_Pit_desired(6), velocity_Pit_init(6), velocity_Pit_traj(6), velocity_Pit_error(6);
Eigen::VectorXd velocity_Pmt(6), velocity_Pmt_desired(6), velocity_Pmt_init(6), velocity_Pmt_traj(6), velocity_Pmt_error(6);
Eigen::VectorXd velocity_Ppt(6), velocity_Ppt_desired(6), velocity_Ppt_init(6), velocity_Ppt_traj(6), velocity_Ppt_error(6);

// safe maximum and minimum joint values
Eigen::Vector4d thumb_joint_safe_max((Eigen::Vector4d() << thumb_joint_0_max-joint_safety_margin, thumb_joint_1_max-joint_safety_margin, 
                                                           thumb_joint_2_max-joint_safety_margin, thumb_joint_3_max-joint_safety_margin).finished());
Eigen::Vector4d thumb_joint_safe_min((Eigen::Vector4d() << thumb_joint_0_min+joint_safety_margin, thumb_joint_1_min+joint_safety_margin, 
                                                           thumb_joint_2_min+joint_safety_margin, thumb_joint_3_min+joint_safety_margin).finished());

Eigen::Vector4d index_joint_safe_max((Eigen::Vector4d() << index_joint_0_max-joint_safety_margin, index_joint_1_max-joint_safety_margin, 
                                                           index_joint_2_max-joint_safety_margin, index_joint_3_max-joint_safety_margin).finished());
Eigen::Vector4d index_joint_safe_min((Eigen::Vector4d() << index_joint_0_min+joint_safety_margin, index_joint_1_min+joint_safety_margin, 
                                                           index_joint_2_min+joint_safety_margin, index_joint_3_min+joint_safety_margin).finished());

Eigen::Vector4d middle_joint_safe_max((Eigen::Vector4d() << middle_joint_0_max-joint_safety_margin, middle_joint_1_max-joint_safety_margin, 
                                                           middle_joint_2_max-joint_safety_margin, middle_joint_3_max-joint_safety_margin).finished());
Eigen::Vector4d middle_joint_safe_min((Eigen::Vector4d() << middle_joint_0_min+joint_safety_margin, middle_joint_1_min+joint_safety_margin, 
                                                           middle_joint_2_min+joint_safety_margin, middle_joint_3_min+joint_safety_margin).finished());

Eigen::Vector4d pinky_joint_safe_max((Eigen::Vector4d() << pinky_joint_0_max-joint_safety_margin, pinky_joint_1_max-joint_safety_margin, 
                                                           pinky_joint_2_max-joint_safety_margin, pinky_joint_3_max-joint_safety_margin).finished());
Eigen::Vector4d pinky_joint_safe_min((Eigen::Vector4d() << pinky_joint_0_min+joint_safety_margin, pinky_joint_1_min+joint_safety_margin, 
                                                           pinky_joint_2_min+joint_safety_margin, pinky_joint_3_min+joint_safety_margin).finished());

// safe joint range (stroke)
Eigen::Vector4d thumb_joint_safe_range(thumb_joint_safe_max-thumb_joint_safe_min);
Eigen::Vector4d index_joint_safe_range(index_joint_safe_max-index_joint_safe_min);
Eigen::Vector4d middle_joint_safe_range(middle_joint_safe_max-middle_joint_safe_min);
Eigen::Vector4d pinky_joint_safe_range(pinky_joint_safe_max-pinky_joint_safe_min);

// safe mean value per joint
Eigen::Vector4d thumb_joint_safe_mean((thumb_joint_safe_max+thumb_joint_safe_min)/2);
Eigen::Vector4d index_joint_safe_mean((index_joint_safe_max+index_joint_safe_min)/2);
Eigen::Vector4d middle_joint_safe_mean((middle_joint_safe_max+middle_joint_safe_min)/2);
Eigen::Vector4d pinky_joint_safe_mean((pinky_joint_safe_max+pinky_joint_safe_min)/2);

Eigen::Vector3d position_Ptt, position_Ptt_desired, position_Ptt_init, position_Ptt_traj, position_Ptt_error;
Eigen::Vector3d position_Pit, position_Pit_desired, position_Pit_init, position_Pit_traj, position_Pit_error;
Eigen::Vector3d position_Pmt, position_Pmt_desired, position_Pmt_init, position_Pmt_traj, position_Pmt_error;
Eigen::Vector3d position_Ppt, position_Ppt_desired, position_Ppt_init, position_Ppt_traj, position_Ppt_error;

Eigen::Vector3d velocity_Ptt_desired_3d, velocity_Pit_desired_3d, velocity_Pmt_desired_3d, velocity_Ppt_desired_3d;
Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity(4,4);
Eigen::Matrix4d finger_null_space_projector;
Eigen::Vector4d finger_task_gradient;

double time_now, start_time, trajectory_duration, end_time;
double time_past;
double Kp_finger, lambda;
clock_t begin, end, now;
  


/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int  GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

/////////////////////////////////////////////////////////////////////////////////////////
// Read keyboard input (one char) from stdin
char Getch(){
  char buf=0;
  struct termios old={0};
  fflush(stdout);
  if(tcgetattr(0, &old)<0)
    perror("tcsetattr()");
  old.c_lflag&=~ICANON;
  old.c_lflag&=~ECHO;
  old.c_cc[VMIN]=1;
  old.c_cc[VTIME]=0;
  if(tcsetattr(0, TCSANOW, &old)<0)
    perror("tcsetattr ICANON");
  if(read(0,&buf,1)<0)
    perror("read()");
  old.c_lflag|=ICANON;
  old.c_lflag|=ECHO;
  if(tcsetattr(0, TCSADRAIN, &old)<0)
    perror ("tcsetattr ~ICANON");
  printf("%c\n",buf);
  return buf;
}

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst){
  char id_des;
  char id_cmd;
  char id_src;
  int len;
  unsigned char data[8];
  unsigned char data_return = 0;
  int i;

  while (ioThreadRun){
    /* wait for the event */
    while (0 == get_message(CAN_Ch, &id_cmd, &id_src, &id_des, &len, data, FALSE)){
      switch (id_cmd){
        case ID_CMD_QUERY_ID:
        {
          printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
          printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
          printf("                      hardware type: 0x%02x\n", data[7]);
        }
        break;

        case ID_CMD_AHRS_POSE:
        {
          printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
          printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
          printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
        }
        break;

        case ID_CMD_AHRS_ACC:
        {
          printf(">CAN(%d): AHRS Acc(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
          printf("               Acc(y): 0x%02x%02x\n", data[2], data[3]);
          printf("               Acc(z): 0x%02x%02x\n", data[4], data[5]);
        }
        break;

        case ID_CMD_AHRS_GYRO:
        {
          printf(">CAN(%d): AHRS Angular Vel(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
          printf("               Angular Vel(y): 0x%02x%02x\n", data[2], data[3]);
          printf("               Angular Vel(z): 0x%02x%02x\n", data[4], data[5]);
        }
        break;

        case ID_CMD_AHRS_MAG:
        {
          printf(">CAN(%d): AHRS Magnetic Field(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
          printf("               Magnetic Field(y): 0x%02x%02x\n", data[2], data[3]);
          printf("               Magnetic Field(z): 0x%02x%02x\n", data[4], data[5]);
        }
        break;

        case ID_CMD_QUERY_CONTROL_DATA:
        {
          if (id_src >= ID_DEVICE_SUB_01 && id_src <= ID_DEVICE_SUB_04){
            vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 0] = (int)(data[0] | (data[1] << 8));
            vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 1] = (int)(data[2] | (data[3] << 8));
            vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 2] = (int)(data[4] | (data[5] << 8));
            vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 3] = (int)(data[6] | (data[7] << 8));
            data_return |= (0x01 << (id_src-ID_DEVICE_SUB_01));
            recvNum++;
          }
          if (data_return == (0x01 | 0x02 | 0x04 | 0x08)){
            // convert encoder count to joint angle
            for (i=0; i<MAX_DOF; i++){
              //q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0);
              q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-enc_offset_calibrated[i])*(333.3/65536.0)*(3.141592/180.0);
            }

            // MY STUFF !
            // CONTROL LOOP!
            //ComputeTorque();
            //for(int i=0; i<16; i++)
            //  tau_des[i] = torque_desired(i);
            
            // convert desired torque to desired current and PWM count
            for (i=0; i<MAX_DOF; i++){
              cur_des[i] = tau_des[i] * motor_dir[i];
              if (cur_des[i] > 1.0) cur_des[i] = 1.0;
              else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
            }

            // send torques
            for (int i=0; i<4;i++){
              // the index order for motors is different from that of encoders
              switch (HAND_VERSION){
                case 1:
                case 2:
                  vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v2);
                  vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v2);
                  vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v2);
                  vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v2);
                  break;
                case 3:
                default:
                  vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v3);
                  vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v3);
                  vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v3);
                  vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v3);
                  break;
              }
              write_current(CAN_Ch, i, &vars.pwm_demand[4*i]);
              usleep(5);
            }
            sendNum++;
            curTime += delT;

            data_return = 0;
          }
        }
        break;
      }
    }
  }
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN(){
#if defined(PEAKCAN)
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
  CAN_Ch = 1;
#elif defined(SOFTINGCAN)
  CAN_Ch = 1;
#else
  CAN_Ch = 1;
#endif
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
  printf(">CAN(%d): open\n", CAN_Ch);

  int ret = command_can_open(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_canopen !!! \n");
      return false;
    }

  ioThreadRun = true;

  /* initialize condition variable */
  int ioThread_error = pthread_create(&hThread, NULL, ioThreadProc, 0);
  printf(">CAN: starts listening CAN frames\n");

  printf(">CAN: query system id\n");
  ret = command_can_query_id(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_can_query_id !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: AHRS set\n");
  ret = command_can_AHRS_set(CAN_Ch, AHRS_RATE_100Hz, AHRS_MASK_POSE | AHRS_MASK_ACC);
  if(ret < 0)
    {
      printf("ERROR command_can_AHRS_set !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: system init\n");
  ret = command_can_sys_init(CAN_Ch, 3/*msec*/);
  if(ret < 0)
    {
      printf("ERROR command_can_sys_init !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: start periodic communication\n");
  ret = command_can_start(CAN_Ch);

  if(ret < 0)
    {
      printf("ERROR command_can_start !!! \n");
      command_can_stop(CAN_Ch);
      command_can_close(CAN_Ch);
      return false;
    }
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN(){
  printf(">CAN: stop periodic communication\n");
  int ret = command_can_stop(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_can_stop !!! \n");
    }

  if (ioThreadRun)
    {
      printf(">CAN: stoped listening CAN frames\n");
      ioThreadRun = false;
      int status;
      pthread_join(hThread, (void **)&status);
      hThread = 0;
    }

  printf(">CAN(%d): close\n", CAN_Ch);
  ret = command_can_close(CAN_Ch);
  if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm(){
  if (RIGHT_HAND)
    pBHand = bhCreateRightHand();
  else
    pBHand = bhCreateLeftHand();

  if (!pBHand) return false;
  pBHand->SetTimeInterval(delT);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm(){
  if (pBHand)
    {
#ifndef _DEBUG
      delete pBHand;
#endif
      pBHand = NULL;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname){
  if (!cname) return 0;

  if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
    return 0;
  else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
    return 1;
  else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
    return 2;
  else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
    return 3;
  else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
    return 4;
  else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
    return 5;
  else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
    return 6;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
    return 7;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
    return 8;
  else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
    return 9;
  else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
    return 10;
  else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
    return 11;
  else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
    return 12;
  else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
    return 13;
  else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
    return 14;
  else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
    return 15;
  else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
    return 16;
  else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
    return 17;
  else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
    return 18;
  else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
    return 19;
  else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
    return 20;
  else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
    return 21;
  else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
    return 22;
  else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
    return 23;
  else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
    return 24;
  else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
    return 25;
  else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
    return 26;
  else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
    return 271;
  else
    return 0;
}








////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction(){
  printf("--------------------------------------------------\n");
  printf("myAllegroHand: ");
  if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

  printf("Keyboard Commands:\n");
  printf("H: Home Position (PD control)\n");
  printf("m: Mean Joint Position\n");
  printf("n: Min Joint Position\n");
  printf("x: Max Joint Position\n");
  printf("Q: Quit this program\n");

  printf("--------------------------------------------------\n\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop(){
  bool bRun = true;
  while (bRun){
    int c = Getch();
    switch(c){
      case 'q':
        if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
        bRun = false;
        break;
      case 'h':
        //q_desired = q_home;
        index_q_des  = index_q_des_home;
        middle_q_des = middle_q_des_home;
        pinky_q_des  = pinky_q_des_home;
        thumb_q_des  = thumb_q_des_home;
        break;
	    case 'm':
	      //q_desired = q_mean;
	      index_q_des  = index_q_des_mean;
        middle_q_des = middle_q_des_mean;
        pinky_q_des  = pinky_q_des_mean;
        thumb_q_des  = thumb_q_des_mean;
        break;
	    case 'n':
	      index_q_des  = index_q_des_min;
        middle_q_des = middle_q_des_min;
        pinky_q_des  = pinky_q_des_min;
        thumb_q_des  = thumb_q_des_min;
	      break;
	    case 'x':
	      index_q_des  = index_q_des_max;
        middle_q_des = middle_q_des_max;
        pinky_q_des  = pinky_q_des_max;
        thumb_q_des  = thumb_q_des_max;
	      break;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque(){
  if (!pBHand) return;
  for(int i=0; i<16; i++)
    tau_des[i] = torque_desired(i);
  
  /*
  // get current joint value per finger
  for(int i=0; i<4; i++)
    index_joint_position(i) = q[i];
  for(int i=4; i<8; i++)
    middle_joint_position(i-4) = q[i];
  for(int i=8; i<12; i++)
    pinky_joint_position(i-8) = q[i];
  for(int i=12; i<16; i++)
    thumb_joint_position(i-12) = q[i];
  
  
  
  
  
  
  
	
	Kp_finger = 1.7;
	lambda = -2.0;
  
  
  while(time_now < end_time){
    now = clock();
    time_now = (double)(now)/CLOCKS_PER_SEC;
    
    thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
	  index_DGM  = finger_direct_geometric_model("index",index_joint_position);
	  middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
	  pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
	
	  pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
	  pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
	  pose_rpy_Pmt = transformation_matrix_to_pose_rpy(middle_DGM);
	  pose_rpy_Ppt = transformation_matrix_to_pose_rpy(pinky_DGM);
    
    thumb_position_jacobian  = finger_position_jacobian("thumb",thumb_joint_position);
	  index_position_jacobian  = finger_position_jacobian("index",index_joint_position);
	  middle_position_jacobian = finger_position_jacobian("middle",middle_joint_position);
	  pinky_position_jacobian  = finger_position_jacobian("pinky",pinky_joint_position);
    
    position_Ptt << pose_rpy_Ptt(0), pose_rpy_Ptt(1), pose_rpy_Ptt(2);
	  position_Pit << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
	  position_Pmt << pose_rpy_Pmt(0), pose_rpy_Pmt(1), pose_rpy_Pmt(2);
	  position_Ppt << pose_rpy_Ppt(0), pose_rpy_Ppt(1), pose_rpy_Ppt(2);
	
	  position_Ptt_desired << 0.02,  0.00, 0.08;
	  position_Pit_desired << 0.04,  0.05, 0.20;
	  position_Pmt_desired << 0.04,   0.0, 0.20;
	  position_Ppt_desired << 0.04, -0.05, 0.20;
	
	
	  //Eigen::Vector3d OnlineMP_L5B(double ti, double tf, double tn, double Pi, double Pf);
	
	  position_Ptt_error = position_Ptt_desired - position_Ptt;
	  position_Pit_error = position_Pit_desired - position_Pit;
	  position_Pmt_error = position_Pmt_desired - position_Pmt;
	  position_Ppt_error = position_Ppt_desired - position_Ppt;
	
	  velocity_Ptt_desired_3d = Kp_finger*position_Ptt_error;
	  velocity_Pit_desired_3d = Kp_finger*position_Pit_error;
	  velocity_Pmt_desired_3d = Kp_finger*position_Pmt_error;
	  velocity_Ppt_desired_3d = Kp_finger*position_Ppt_error;
	
	  finger_null_space_projector = I4 - Pinv_damped(thumb_position_jacobian, 0.001)*thumb_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(thumb_joint_position, thumb_joint_safe_mean, thumb_joint_safe_range);
	  thumb_joint_velocity_command  = Pinv_damped(thumb_position_jacobian, 0.001)*velocity_Ptt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	
	  finger_null_space_projector = I4 - Pinv_damped(index_position_jacobian, 0.001)*index_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(index_joint_position, index_joint_safe_mean, index_joint_safe_range);
	  index_joint_velocity_command  = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	
	  finger_null_space_projector = I4 - Pinv_damped(middle_position_jacobian, 0.001)*middle_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(middle_joint_position, middle_joint_safe_mean, middle_joint_safe_range);
	  middle_joint_velocity_command  = Pinv_damped(middle_position_jacobian, 0.001)*velocity_Pmt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	
	  finger_null_space_projector = I4 - Pinv_damped(pinky_position_jacobian, 0.001)*pinky_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(pinky_joint_position, pinky_joint_safe_mean, pinky_joint_safe_range);
	  pinky_joint_velocity_command  = Pinv_damped(pinky_position_jacobian, 0.001)*velocity_Ppt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
    
    
    
    
    
    
    // for safety
    for(int i=0; i<4; i++){
      if( (index_joint_position(i) >= index_q_des_max(i)) or (index_joint_position(i) <= index_q_des_min(i)) ){
        //index_joint_velocity_command(i) = 0.0;
        std::cout<<"WARNING: index joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (middle_joint_position(i) >= middle_q_des_max(i)) or (middle_joint_position(i) <= middle_q_des_min(i)) ){
        //middle_joint_velocity_command(i) = 0.0;
        std::cout<<"WARNING: middle joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (pinky_joint_position(i) >= pinky_q_des_max(i)) or (pinky_joint_position(i) <= pinky_q_des_min(i)) ){
        //pinky_joint_velocity_command(i) = 0.0;
        std::cout<<"WARNING: pinky joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (thumb_joint_position(i) >= thumb_q_des_max(i)) or (thumb_joint_position(i) <= thumb_q_des_min(i)) ){
        //thumb_joint_velocity_command(i) = 0.0;
        std::cout<<"WARNING: thumb joint limit violation at joint " << i << std::endl;
      }
    }
    
    
    // joint torque values to set after safety check 
    //std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
    torque_desired << index_joint_velocity_command, middle_joint_velocity_command, pinky_joint_velocity_command, thumb_joint_velocity_command;
    //std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
    
    
    
    

    for(int i=0; i<16; i++)
      tau_des[i] = torque_desired(i);
    //for(int i=0; i<16; i++)
    //  tau_des[i] = 0.0;
    
    //std::cout<<"joint error      = " << q_error.transpose() << std::endl;
    //std::cout<<"joint error norm = " << q_error.norm() << std::endl;
    //std::cout<<"current joint position = " << q_current.transpose() << std::endl;
    std::cout<<"position_Ptt     = " << position_Ptt.transpose() << std::endl;
    std::cout<<"position_Pit     = " << position_Pit.transpose() << std::endl;
    std::cout<<"position_Pmt     = " << position_Pmt.transpose() << std::endl;
    std::cout<<"position_Ppt     = " << position_Ppt.transpose() << std::endl;
    
    
    //std::cout<<"thumb_joint_position  = " << thumb_joint_position.transpose() << std::endl;
  }
  for(int i=0; i<100000; i++)
    std::cout<<" control loop end ... " << std::endl;
  */
}




/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[]){
  q_home << 0.0971069, 0.0856564, 0.764694, 0.531159, 0.0347952, -0.174863, 1.01953, 0.0976395, -0.0189953, -0.0258301, 0.819284, 0.326027, 0.645042, 0.623384, 0.302327, 0.955269;
  
  torque_desired << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0;
  
  q_min << 
  index_joint_0_min+joint_safety_margin,  index_joint_1_min+joint_safety_margin,  index_joint_2_min+joint_safety_margin,  index_joint_3_min+joint_safety_margin,     // index
  middle_joint_0_min+joint_safety_margin, middle_joint_1_min+joint_safety_margin, middle_joint_2_min+joint_safety_margin, middle_joint_3_min+joint_safety_margin,    // middle
  pinky_joint_0_min+joint_safety_margin,  pinky_joint_1_min+joint_safety_margin,  pinky_joint_2_min+joint_safety_margin,  pinky_joint_3_min+joint_safety_margin,     // pinky
  thumb_joint_0_min+joint_safety_margin,  thumb_joint_1_min+joint_safety_margin,  thumb_joint_2_min+joint_safety_margin,  thumb_joint_3_min+joint_safety_margin;     // thumb

  q_mean << 
  (index_joint_0_max+index_joint_0_min)/2,   (index_joint_1_max+index_joint_1_min)/2,   (index_joint_2_max+index_joint_2_min)/2,   (index_joint_3_max+index_joint_3_min)/2,     // index
  (middle_joint_0_max+middle_joint_0_min)/2, (middle_joint_1_max+middle_joint_1_min)/2, (middle_joint_2_max+middle_joint_2_min)/2, (middle_joint_3_max+middle_joint_3_min)/2,   // middle
  (pinky_joint_0_max+pinky_joint_0_min)/2,   (pinky_joint_1_max+pinky_joint_1_min)/2,   (pinky_joint_2_max+pinky_joint_2_min)/2,   (pinky_joint_3_max+pinky_joint_3_min)/2,     // pinky
  (thumb_joint_0_max+thumb_joint_0_min)/2,   (thumb_joint_1_max+thumb_joint_1_min)/2,   (thumb_joint_2_max+thumb_joint_2_min)/2,   (thumb_joint_3_max+thumb_joint_3_min)/2;     // thumb  
  
  q_desired = q_home;
  
  index_q_des_home  << 0.0971069, 0.0856564, 0.764694, 0.531159;
  middle_q_des_home << 0.0347952, -0.174863, 1.01953, 0.0976395;
  pinky_q_des_home  << -0.0189953, -0.0258301, 0.819284, 0.326027;
  thumb_q_des_home  << 0.645042, 0.623384, 0.302327, 0.955269;
  
  index_q_des_mean  << (index_joint_0_max+index_joint_0_min)/2,   (index_joint_1_max+index_joint_1_min)/2,   (index_joint_2_max+index_joint_2_min)/2,   (index_joint_3_max+index_joint_3_min)/2;
  middle_q_des_mean << (middle_joint_0_max+middle_joint_0_min)/2, (middle_joint_1_max+middle_joint_1_min)/2, (middle_joint_2_max+middle_joint_2_min)/2, (middle_joint_3_max+middle_joint_3_min)/2;
  pinky_q_des_mean  << (pinky_joint_0_max+pinky_joint_0_min)/2,   (pinky_joint_1_max+pinky_joint_1_min)/2,   (pinky_joint_2_max+pinky_joint_2_min)/2,   (pinky_joint_3_max+pinky_joint_3_min)/2;
  thumb_q_des_mean  << (thumb_joint_0_max+thumb_joint_0_min)/2,   (thumb_joint_1_max+thumb_joint_1_min)/2,   (thumb_joint_2_max+thumb_joint_2_min)/2,   (thumb_joint_3_max+thumb_joint_3_min)/2;
  
  index_q_des_min  << index_joint_0_min+joint_safety_margin,  index_joint_1_min+joint_safety_margin,  index_joint_2_min+joint_safety_margin,  index_joint_3_min+joint_safety_margin;
  middle_q_des_min << middle_joint_0_min+joint_safety_margin, middle_joint_1_min+joint_safety_margin, middle_joint_2_min+joint_safety_margin, middle_joint_3_min+joint_safety_margin;
  pinky_q_des_min  << pinky_joint_0_min+joint_safety_margin,  pinky_joint_1_min+joint_safety_margin,  pinky_joint_2_min+joint_safety_margin,  pinky_joint_3_min+joint_safety_margin;
  thumb_q_des_min  << thumb_joint_0_min+joint_safety_margin,  thumb_joint_1_min+joint_safety_margin,  thumb_joint_2_min+joint_safety_margin,  thumb_joint_3_min+joint_safety_margin;
  
  index_q_des_max  << index_joint_0_max-joint_safety_margin,  index_joint_1_max-joint_safety_margin,  index_joint_2_max-joint_safety_margin,  index_joint_3_max-joint_safety_margin;
  middle_q_des_max << middle_joint_0_max-joint_safety_margin, middle_joint_1_max-joint_safety_margin, middle_joint_2_max-joint_safety_margin, middle_joint_3_max-joint_safety_margin;
  pinky_q_des_max  << pinky_joint_0_max-joint_safety_margin,  pinky_joint_1_max-joint_safety_margin,  pinky_joint_2_max-joint_safety_margin,  pinky_joint_3_max-joint_safety_margin;
  thumb_q_des_max  << thumb_joint_0_max-joint_safety_margin,  thumb_joint_1_max-joint_safety_margin,  thumb_joint_2_max-joint_safety_margin,  thumb_joint_3_max-joint_safety_margin;
  
  index_q_des  = index_q_des_home;
  middle_q_des = middle_q_des_home;
  pinky_q_des  = pinky_q_des_home;
  thumb_q_des  = thumb_q_des_home;
  
  
  
  
  
  /*
  std_msgs::Float32MultiArray joint_torque_cmd, joint_position_cmd, joint_velocity_cmd;
  
  // A node use for ROS transport
	std::unique_ptr<ros::NodeHandle> rosNode;

	// A ROS subscriber
	ros::Subscriber JointTorqueCommandSub, JointPositionCommandSub, JointVelocityCommandSub;
	
	// ROS publishers
	ros::Publisher JointPositionPub;
	ros::Publisher JointVelocityPub;
	ros::Publisher JointTorquePub;

	// A ROS callbackqueue that helps process messages
	ros::CallbackQueue rosQueue;

	// A thread the keeps running the rosQueue
	std::thread rosQueueThread;
  */
  
  
  
  
  
  
  PrintInstruction();

  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  curTime = 0.0;

  //if (CreateBHandAlgorithm() && OpenCAN())
  //  MainLoop();
  
  
  
  
  
  
  
  
  
  
  if(CreateBHandAlgorithm() && OpenCAN()){
  //if(!pBHand) return;
  
  
  //small delay till communication is okay!
  begin = clock();
  start_time = (double)(begin)/CLOCKS_PER_SEC;
  time_now = start_time;
  end_time = start_time + 2.0;
  while(time_now < end_time){
    now = clock();
    time_now = (double)(now)/CLOCKS_PER_SEC;
  }
  
  thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
  index_DGM  = finger_direct_geometric_model("index",index_joint_position);
  middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
  pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);

  pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
  pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
  pose_rpy_Pmt = transformation_matrix_to_pose_rpy(middle_DGM);
  pose_rpy_Ppt = transformation_matrix_to_pose_rpy(pinky_DGM);
  
  thumb_position_jacobian  = finger_position_jacobian("thumb",thumb_joint_position);
  index_position_jacobian  = finger_position_jacobian("index",index_joint_position);
  middle_position_jacobian = finger_position_jacobian("middle",middle_joint_position);
  pinky_position_jacobian  = finger_position_jacobian("pinky",pinky_joint_position);
  
  position_Ptt_init << pose_rpy_Ptt(0), pose_rpy_Ptt(1), pose_rpy_Ptt(2);
  position_Pit_init << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
  position_Pmt_init << pose_rpy_Pmt(0), pose_rpy_Pmt(1), pose_rpy_Pmt(2);
  position_Ppt_init << pose_rpy_Ppt(0), pose_rpy_Ppt(1), pose_rpy_Ppt(2);
  
  
  Eigen::Vector3d x_traj, y_traj, z_traj;
  
  
  begin = clock();
  start_time = (double)(begin)/CLOCKS_PER_SEC;
  time_now = start_time;
  trajectory_duration = 5.0;
  end_time = start_time + trajectory_duration;
  
  
  //while(1){
  while(time_now < end_time){
    now = clock();
    time_now = (double)(now)/CLOCKS_PER_SEC;
  
  
  // get current joint value per finger
  for(int i=0; i<4; i++)
    index_joint_position(i) = q[i];
  for(int i=4; i<8; i++)
    middle_joint_position(i-4) = q[i];
  for(int i=8; i<12; i++)
    pinky_joint_position(i-8) = q[i];
  for(int i=12; i<16; i++)
    thumb_joint_position(i-12) = q[i];
  
  /*
  Eigen::VectorXd encoder_value(16);
  Eigen::Vector4d thumb_encoder_value;
  for(int i=0; i<16; i++)
    encoder_value(i) = (double)(vars.enc_actual[i]*enc_dir[i]);
  //std::cout<<"encoder_value     = " << encoder_value.transpose() << std::endl;
  thumb_encoder_value << encoder_value(12), encoder_value(13), encoder_value(14), encoder_value(15);
  std::cout<<"thumb_encoder_value  = " << thumb_encoder_value.transpose() << std::endl;
  */
  
  
  
  
  
	
	Kp_finger = 1.3;
	lambda = -2.0;
	
	
  
  
  //while(time_now < end_time){
  
    
    
    thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
	  index_DGM  = finger_direct_geometric_model("index",index_joint_position);
	  middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
	  pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
	
	  pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
	  pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
	  pose_rpy_Pmt = transformation_matrix_to_pose_rpy(middle_DGM);
	  pose_rpy_Ppt = transformation_matrix_to_pose_rpy(pinky_DGM);
    
    thumb_position_jacobian  = finger_position_jacobian("thumb",thumb_joint_position);
	  index_position_jacobian  = finger_position_jacobian("index",index_joint_position);
	  middle_position_jacobian = finger_position_jacobian("middle",middle_joint_position);
	  pinky_position_jacobian  = finger_position_jacobian("pinky",pinky_joint_position);
    
    position_Ptt << pose_rpy_Ptt(0), pose_rpy_Ptt(1), pose_rpy_Ptt(2);
	  position_Pit << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
	  position_Pmt << pose_rpy_Pmt(0), pose_rpy_Pmt(1), pose_rpy_Pmt(2);
	  position_Ppt << pose_rpy_Ppt(0), pose_rpy_Ppt(1), pose_rpy_Ppt(2);
	
	  position_Ptt_desired << 0.02,  0.00, 0.08;
	  position_Pit_desired << 0.04,  0.05, 0.20;
	  position_Pmt_desired << 0.04,   0.0, 0.20;
	  position_Ppt_desired << 0.04, -0.05, 0.20;
	
	  /*
	  x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(0), position_Ptt_desired(0));
	  y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(1), position_Ptt_desired(1));
	  z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(2), position_Ptt_desired(2));
	  position_Ptt_traj << x_traj(0), y_traj(0), z_traj(0);
	  
	  
	  x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(0), position_Pmt_desired(0));
	  y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(1), position_Pmt_desired(1));
	  z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(2), position_Pmt_desired(2));
	  position_Pmt_traj << x_traj(0), y_traj(0), z_traj(0);
	  */
	  //std::cout<<"position_Ptt(1)=" << position_Ptt(1)<<", position_Ptt_init(1)=" << position_Ptt_init(1)<<", position_Ptt_desired(1)=" << position_Ptt_desired(1)<<", position_Ptt_traj(1)=" << position_Ptt_traj(1) << std::endl;
    
    
	  position_Ptt_error = position_Ptt_desired - position_Ptt;
	  position_Pit_error = position_Pit_desired - position_Pit;
	  position_Pmt_error = position_Pmt_desired - position_Pmt;
	  position_Ppt_error = position_Ppt_desired - position_Ppt;
	  
	  /*
	  position_Ptt_error = position_Ptt_traj - position_Ptt;
	  position_Pit_error = position_Pit_desired - position_Pit;
	  position_Pmt_error = position_Pmt_desired - position_Pmt;
	  position_Ppt_error = position_Ppt_desired - position_Ppt;
	  */
	  /*
	  position_Ptt_error = position_Ptt_desired - position_Ptt;
	  position_Pit_error = position_Pit_desired - position_Pit;
	  position_Pmt_error = position_Pmt_traj - position_Pmt;
	  position_Ppt_error = position_Ppt_desired - position_Ppt;
	  */
	  //std::cout<<"position_Ptt_error = " << position_Pmt_error.transpose() << std::endl;
	  
	  
	  
	  velocity_Ptt_desired_3d = Kp_finger*position_Ptt_error;
	  velocity_Pit_desired_3d = Kp_finger*position_Pit_error;
	  velocity_Pmt_desired_3d = Kp_finger*position_Pmt_error;
	  velocity_Ppt_desired_3d = Kp_finger*position_Ppt_error;
	  
	  /*
	  if(position_Ptt_error.norm()<0.01 and position_Ptt_error.norm()>0.003)
	    velocity_Ptt_desired_3d = 3.0*position_Ptt_error;
	  std::cout<<"position_Ptt_error.norm() = " << position_Ptt_error.norm() << std::endl;
	  */
	  
	  finger_null_space_projector = I4 - Pinv_damped(thumb_position_jacobian, 0.001)*thumb_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(thumb_joint_position, thumb_joint_safe_mean, thumb_joint_safe_range);
	  thumb_joint_velocity_command  = Pinv_damped(thumb_position_jacobian, 0.001)*velocity_Ptt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	  
	  finger_null_space_projector = I4 - Pinv_damped(index_position_jacobian, 0.001)*index_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(index_joint_position, index_joint_safe_mean, index_joint_safe_range);
	  index_joint_velocity_command  = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	
	  finger_null_space_projector = I4 - Pinv_damped(middle_position_jacobian, 0.001)*middle_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(middle_joint_position, middle_joint_safe_mean, middle_joint_safe_range);
	  middle_joint_velocity_command  = Pinv_damped(middle_position_jacobian, 0.001)*velocity_Pmt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
	
	  finger_null_space_projector = I4 - Pinv_damped(pinky_position_jacobian, 0.001)*pinky_position_jacobian;
	  finger_task_gradient = avoid_joint_limit_task_gradient(pinky_joint_position, pinky_joint_safe_mean, pinky_joint_safe_range);
	  pinky_joint_velocity_command  = Pinv_damped(pinky_position_jacobian, 0.001)*velocity_Ppt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
    
    
    
    /*
    q_desired << index_q_des, middle_q_des, pinky_q_des, thumb_q_des;
    q_current << index_joint_position, middle_joint_position, pinky_joint_position, thumb_joint_position;
    
    q_error = q_desired - q_current;
    torque_desired = 1.7*q_error;
    */
    
    
    // for safety
    for(int i=0; i<4; i++){
      if( (index_joint_position(i) >= index_q_des_max(i)) or (index_joint_position(i) <= index_q_des_min(i)) ){
        //index_joint_velocity_command(i) = 0.0;
        //std::cout<<"WARNING: index joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (middle_joint_position(i) >= middle_q_des_max(i)) or (middle_joint_position(i) <= middle_q_des_min(i)) ){
        //middle_joint_velocity_command(i) = 0.0;
        //std::cout<<"WARNING: middle joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (pinky_joint_position(i) >= pinky_q_des_max(i)) or (pinky_joint_position(i) <= pinky_q_des_min(i)) ){
        //pinky_joint_velocity_command(i) = 0.0;
        //std::cout<<"WARNING: pinky joint limit violation at joint " << i << std::endl;
      }
    }
    for(int i=0; i<4; i++){
      if( (thumb_joint_position(i) >= thumb_q_des_max(i)) or (thumb_joint_position(i) <= thumb_q_des_min(i)) ){
        //thumb_joint_velocity_command(i) = 0.0;
        //std::cout<<"WARNING: thumb joint limit violation at joint " << i << std::endl;
      }
    }
    
    
    // joint torque values to set after safety check 
    //std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
    torque_desired << index_joint_velocity_command, middle_joint_velocity_command, pinky_joint_velocity_command, thumb_joint_velocity_command;
    //std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
    
    
    
    

    for(int i=0; i<16; i++)
      tau_des[i] = torque_desired(i);
    //for(int i=0; i<16; i++)
    //  tau_des[i] = 0.0;
    
    //std::cout<<"joint error      = " << q_error.transpose() << std::endl;
    //std::cout<<"joint error norm = " << q_error.norm() << std::endl;
    //std::cout<<"current joint position = " << q_current.transpose() << std::endl;
    //std::cout<<"position_Ptt     = " << position_Ptt.transpose() << std::endl;
    //std::cout<<"position_Pit     = " << position_Pit.transpose() << std::endl;
    //std::cout<<"position_Pmt     = " << position_Pmt.transpose() << std::endl;
    //std::cout<<"position_Ppt     = " << position_Ppt.transpose() << std::endl;
    
    
    
    //std::cout<<"thumb_joint_position  = " << thumb_joint_position.transpose() << std::endl;
  }
  //for(int i=0; i<100000; i++)
  //  std::cout<<" control loop end ... " << std::endl;
  
  
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  CloseCAN();
  DestroyBHandAlgorithm();

  return 0;
}

