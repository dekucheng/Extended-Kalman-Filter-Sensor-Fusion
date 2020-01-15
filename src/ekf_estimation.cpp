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



void Ekf_Estimation::addmeasurement(const Matrix<double, 6, 1>& noise_odom_, const ros::Time& odom_stamp){
    double dt = odom_stamp - last_filter_time;
    if(dt < 0.01) {
        ROS_WARN("odom time is older than filter time skip this update!");
        return;
    } 

    odom_delta = noise_odom_.block<3,1>(0,0);

    odom_measured_state(0) = state(0) + delta_hat_pose(1) * 
            cos(odom_delta(2) + delta_hat_pose(0));
    odom_measured_state(1) = state(1) + delta_hat_pose(1) * 
            sin(odom_delta(2) + delta_hat_pose(0));
    odom_measured_state(2) = state(2) + delta_hat_pose(0) + delta_hat_pose(2);

    H_odom << 

}




Ekf_Estimation::Init(const Vector3d& first_odom, const ros::Time& first_time_stamp) {
    state = first_odom;
    last_filter_time = first_time_stamp;

    // Initialize filter matrices
    G << 1, 0, 0, 
         0, 1, 0,
         0, 0, 1;

    Cov << 0.01,   0        0,
           0,      0.01,    0,
           0,      0,       0.01;
    
    R << 1000000, 0,       0,
         0,       1000000, 0,
         0,       0,       1000000;
    cout << "Ekf Filter Initialized!" << endl;
    cout << "G matrix is: " << G << endl;
    cout << "Cov Matrix is: " << Cov << endl;
    cout << "R Matrix is: " << R << endl;
}
