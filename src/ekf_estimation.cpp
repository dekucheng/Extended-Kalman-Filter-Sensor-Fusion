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

void Ekf_Estimation::motion_update() {
    prior = G * state + Vector3d(0.001, 0.001, 0.001);
    Cov = G * Cov * G.transpose() + R;
}

bool Ekf_Estimation::check_time(const ros::Time& t) {
    double dt = (t - last_filter_time).toSec();
    if(dt < 0.001) {
        ROS_WARN("odom time is older than filter time skip this update!");
        cout << "filter time is: " << last_filter_time.toSec() << "received odom time is :" << t.toSec() << endl;
        return false;
    } 
    else {
        cout << "ready to update!! " << endl;

        // ready to update
        last_filter_time = t;
        return true;
    }
}
void Ekf_Estimation::addmeasurement(const Matrix<double, 6, 1>& noise_odom_){

    odom_delta = noise_odom_.block<3,1>(0,0);

    // odom_measured_state(0) = state(0) + delta_hat_pose(1) * 
    //         cos(odom_delta(2) + delta_hat_pose(0));
    // odom_measured_state(1) = state(1) + delta_hat_pose(1) * 
    //         sin(odom_delta(2) + delta_hat_pose(0));
    // odom_measured_state(2) = state(2) + delta_hat_pose(0) + delta_hat_pose(2);
            
}

void Ekf_Estimation::update() {
    // update motion model to get prior
    motion_update();

    update_odom_matrices();
    cout << "Cov" << endl;
    cout << Cov << endl;
    cout << "H_odom" << endl;
    cout << H_odom << endl;
    cout << "h_u" << endl;
    cout << h_u << endl;
    cout << "odom_delta" << endl;
    cout << odom_delta << endl;
    cout << "inovation" << endl;
    cout << odom_delta-h_u << endl;

    K = Cov*H_odom.transpose()*(H_odom*Cov*H_odom.transpose() + Q_odom).inverse();
    cout << "Kalman Gain is :" << endl;
    cout << K << endl;
    state = prior + K*(odom_delta - h_u);

    cout << "K*Inovation" << endl;
    cout << K*(odom_delta - h_u) << endl;
    Cov = (Matrix3d::Identity() - K*H_odom) * Cov;
    for (int i=0; i < Cov.rows(); i++) {
        for (int j=0; j<Cov.cols(); j++) {
            if (Cov(i,j)>1000000) Cov(i,j)=1000000;
            if (Cov(i,j)<-1000000) Cov(i,j)=-1000000;
        }
    }

}

void Ekf_Estimation::update_odom_matrices() {
    double dx = prior(1) - state(1), dy = prior(0) - state(0);
    cout << "DEBUG!!!!  " << "dx" << dx << "dy" << dy << "0 0:" << -dy/(pow(dx, 2) + pow(dy, 2)) << " 0 1:" << dx/(pow(dx, 2) + pow(dy, 2)) << endl;
    H_odom << -dy/(pow(dx, 2) + pow(dy, 2)),            dx/(pow(dx, 2) + pow(dy, 2)),   1,
              dx*pow((pow(dx, 2) + pow(dy, 2)), -0.5),  dy*pow((pow(dx, 2) + pow(dy, 2)), -0.5), 0,
              0, 0, 1;
    Q_odom <<  0.0001, 0, 0,
               0,    0.0001, 0,
               0,    0,     0.0001;

    h_u << atan2(dy, dx), sqrt(dx*dx + dy*dy), prior(2)-state(2)-atan2(dy,dx);
}

Vector3d Ekf_Estimation::get_state() const {
    return state;
}



void Ekf_Estimation::Init(const Vector3d& first_odom, const ros::Time& first_time_stamp) {
    state = first_odom;
    last_filter_time = first_time_stamp;

    // Initialize filter matrices
    G << 1, 0, 0, 
         0, 1, 0,
         0, 0, 1;

    Cov << 0.01,   0,       0,
           0,      0.01,    0,
           0,      0,       0.01;
    
    R << 1000000, 0,       0,
         0,       1000000, 0,
         0,       0,       1000000;
    cout << "Ekf Filter Initialized!" << endl;
    cout << "G matrix is: " << G << endl;
    cout << "Cov Matrix is: " << Cov << endl;
    cout << "R Matrix is: " << R << endl;
    is_Initialized = true;
    cout << "Ekf Filter Initialized!!!" << endl;
}
