#include "my_ekf/ekf_estimation.h"

using namespace std;
using namespace Eigen;

Ekf_Estimation::Ekf_Estimation():
 is_Initialized(false),
 imu_Initialized(false)
{
    ROS_INFO("EKF_filter constructed!");
}

Ekf_Estimation::~Ekf_Estimation() {
    ROS_INFO("EKF_filter destructed!");
}
