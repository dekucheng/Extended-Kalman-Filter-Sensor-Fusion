#include "my_ekf/ekf_estimation.h"

using namespace std;
using namespace Eigen;

Ekf_Estimation::Ekf_Estimation():
 is_Initialized(false),
 imu_Initialized(false),
 IF_PRINT(false)
{   
    // Init landmark pose in odom frame
    vector<landmark_pose_euler> lp;
    landmark_pose_euler lp_;
    lp_.id = 0;
    lp_.pose << 4, -1, 0.5, M_PI/2, -M_PI/2, 0;
    lp.push_back(lp_);
    lp_.id = 1;
    lp_.pose << 3, 1, 0.5, M_PI/2, -M_PI/2, 0;
    lp.push_back(lp_);
    lp_.id = 2;
    lp_.pose << 6, 2, 0.5, M_PI/2, 0, 0;
    lp.push_back(lp_);
    lp_.id = 3;
    lp_.pose << 5, 5, 0.5, M_PI/2, 0, 0;
    lp.push_back(lp_);
    lp_.id = 4;
    lp_.pose << 0, 4, 0.5, M_PI/2, M_PI/2, 0;
    lp.push_back(lp_);

    for(int i=0; i<lp.size(); i++) {
        Affine3d r = create_rotation_matrix(lp[i].pose(3), lp[i].pose(4), lp[i].pose(5));
        Affine3d t(Eigen::Translation3d(Eigen::Vector3d(lp[i].pose(0),lp[i].pose(1),lp[i].pose(2))));

        Matrix4d m = (t * r).matrix(); 
        known_landmark_poses[lp[i].id] = m;
        cout << "tag " << i << "pose is :" << endl;
        cout << m << endl;
    }
     
    ROS_INFO("Ekf filter constructed!");
}

Ekf_Estimation::~Ekf_Estimation() {
    ROS_INFO("Ekf filter destructed!");
}

void Ekf_Estimation::motion_update() {
    prior = G * state + Vector3d(0.001, 0.001, 0.001);
    Cov = G * Cov * G.transpose() + R;

    for (int i=0; i < Cov.rows(); i++) {
        for (int j=0; j<Cov.cols(); j++) {
            if (Cov(i,j)>1000000) Cov(i,j)=1000000;
            if (Cov(i,j)<-1000000) Cov(i,j)=-1000000;
        }
    }
}

void Ekf_Estimation::motion_update(const Vector3d& odom_diff) {
    double rot1 = odom_diff(0), trans = odom_diff(1), rot2 = odom_diff(2);
    // motion update for the prior which to be fused
    prior(0) = state(0) + trans*cos(state(2)+rot1);
    prior(1) = state(1) + trans*sin(state(2)+rot1);
    prior(2) = state(2) + rot1 + rot2;
    // motion update for noise odom, which has no fusion data
    motion_model_state(0) = motion_model_state(0) + trans*cos(motion_model_state(2)+rot1);
    motion_model_state(1) = motion_model_state(1) + trans*sin(motion_model_state(2)+rot1);
    motion_model_state(2) = motion_model_state(2) + rot1 + rot2;
    motion_model_state(2) = normalize(motion_model_state(2));

    G_odom << 1, 0, -trans*sin(state(2)+rot1),
              0, 1,  trans*cos(state(2)+rot1),
              0, 0,                         1;
    Cov = G_odom * Cov * G_odom.transpose() + R_odom;
}

void Ekf_Estimation::addmeasurement(const Vector3d& odom_diff){
    odom_meas = odom_diff;
}

void Ekf_Estimation::addmeasurement(const double imu_meas_) {
    imu_meas = imu_meas_;
}

void Ekf_Estimation::addmeasurement(const vector<landmark_pose>& landmark_pose_set) {
    landmark_pose_measurement = landmark_pose_set;
}

bool Ekf_Estimation::update(const bool odom_update, const bool imu_update, const bool landmark_update, const ros::Time this_update_time) {
    // update motion model to get prior
    if (odom_update || imu_update || landmark_update) {
        // modify if use dt
        last_filter_time = this_update_time;
        // motion update
        motion_update();
        if (odom_update) {
            motion_update(odom_meas);
        }
        else {
            motion_update();
        }
        if (imu_update) {
            update_with_imu();
        }
        if (landmark_update) {
            getbasefromlandmarkmeas();
            update_with_landmark();
        }
        state = prior;
        return true;
    }
    else return false;
}


void Ekf_Estimation::update_with_imu() {

    double h_u = prior(2);
    K_imu = Cov*H_imu.transpose()*(H_imu*Cov*H_imu.transpose() + Q_imu).inverse();
    prior = prior + K_imu*(imu_meas - h_u);
    Cov = (Matrix3d::Identity() - K_imu*H_imu) * Cov;

    if (IF_PRINT) {
        cout << "after imu prior theta is :" << prior(2) << endl;
        cout << "imu measurement is :" << imu_meas << endl;
        cout << "K_imu*Inovation" << endl;
        cout << K_imu*(imu_meas - h_u) << endl;
        cout << "Kalman Gain of imu is :" << endl;
        cout << K_imu << endl;
        cout << "before imu prior theta is:" << prior(2) << endl;
    }
}
    // double dx = prior(1) - state(1), dy = prior(0) - state(0);
    // cout << "DEBUG!!!!  " << "dx" << dx << "dy" << dy << "0 0:" << -dy/(pow(dx, 2) + pow(dy, 2)) << " 0 1:" << dx/(pow(dx, 2) + pow(dy, 2)) << endl;
    // H_odom << -dy/(pow(dx, 2) + pow(dy, 2)),            dx/(pow(dx, 2) + pow(dy, 2)),   1,
    //           dx*pow((pow(dx, 2) + pow(dy, 2)), -0.5),  dy*pow((pow(dx, 2) + pow(dy, 2)), -0.5), 0,
    //           0, 0, 1;

    // h_u << atan2(dy, dx), sqrt(dx*dx + dy*dy), prior(2)-state(2)-atan2(dy,dx);

void Ekf_Estimation::update_with_landmark() {
    for (int i=0; i<landmark_meas.size(); i++) {
        Vector3d h_u(prior(0), prior(1), prior(2));
        K_landmark = Cov*H_landmark.transpose()*(H_landmark*Cov*H_landmark.transpose() + Q_landmark).inverse();
        prior = prior + K_landmark*(landmark_meas[i].pose - h_u);
        Cov = (Matrix3d::Identity() - K_landmark*H_landmark) * Cov;
    }
    // cout << " after fuse landmark detections the state is: " << endl;
    // cout << prior << endl;
}

Vector3d Ekf_Estimation::get_state() const {
    return state;
}

Vector3d Ekf_Estimation::get_noise_state() const {
    return motion_model_state;
}

void Ekf_Estimation::Init(const Vector3d& first_odom, const ros::Time& first_time_stamp) {
    state = motion_model_state = first_odom;
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


    R_odom <<  0.05,  0,   0,
               0,    0.05, 0,
               0,    0,   0.8;

    H_imu << 0, 0, 1;

    Q_imu << 0.001;


    H_landmark << 1, 0, 0,
                  0, 1, 0,
                  0, 0, 1;

    Q_landmark <<  0.01, 0,      0,
                   0,    0.01,   0,
                   0,    0,    0.1;

    is_Initialized = true;

    cout << "Ekf Filter Initialized!" << endl;
    cout << "G matrix is: " << G << endl;
    cout << "Cov Matrix is: " << Cov << endl;
    cout << "R Matrix is: " << R << endl;
    cout << "Ekf Filter Initialized!!!" << endl;
}

void Ekf_Estimation::Init_imu(const tf::StampedTransform& t) {
    tf::transformTFToEigen(t, base_imu_offset);
    imu_Initialized = true;  
    cout << "the transform from base to imu is :" << endl;
    cout << base_imu_offset.matrix() << endl;
}

void Ekf_Estimation::Init_landmark(const tf::StampedTransform &t) {
    tf::transformTFToEigen(t, base_camera_offset);
    landmark_Initialized = true;
    cout << "the transform from base to camera is :" << endl;
    cout << base_camera_offset.matrix() << endl;
}

void Ekf_Estimation::getbasefromlandmarkmeas() {
    // convert raw landmark measurement to base foot print pose in odom
    // clear previous measurement
    landmark_meas.clear();
    for (int i=0; i<landmark_pose_measurement.size(); i++) {
        landmark_pose &lp = landmark_pose_measurement[i];
        int id = lp.id;
        Matrix4d tag_pose = known_landmark_poses[id];
        Matrix4d meas_ = tag_pose * lp.pose.inverse() * base_camera_offset.matrix().inverse();

        double sin = -meas_(0,1), cos = meas_(0,0);
        base_pose_from_landmark bp;
        bp.id = id;
        bp.pose(0) = meas_(0,3);
        bp.pose(1) = meas_(1,3);
        bp.pose(2) = atan2(sin, cos);
        landmark_meas.push_back(bp);
    }
}