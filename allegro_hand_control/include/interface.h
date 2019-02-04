

#ifndef _INTERFACE_H
#define _INTERFACE_H

#include "../include/rDeviceAllegroHandCANDef.h"
#include "../include/allegro_hand_parameters.h"
#include <BHand/BHand.h>
#include "../include/Eigen/Dense"

void SetMinJointValue();
void SetMaxJointValue();
void SetMeanJointValue();
void JointControl(Eigen::VectorXd q_desired);

#endif
