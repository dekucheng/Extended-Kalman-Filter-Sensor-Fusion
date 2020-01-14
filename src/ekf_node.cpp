#include "my_ekf/ekf_node.h"

using namespace std;
using namespace Eigen;

Ekf_Node::Ekf_Node():
    private_nh_("~"),
    odom_active_(false),
    imu_active_(false),
    odom_initializing_(false),
    imu_initializing_(false),
    odom_update_(false),
    imu_update_(false)
{
    double freq;
    private_nh_.param("spin_rate", freq, 50.0);
    private_nh_.param("time_out", timeout_, 1.0);
    private_nh_.param("odom_used", odom_used_, true);
    private_nh_.param("imu_used", imu_used_, true);
    private_nh_.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    private_nh_.param("max_path_plot", max_path_plot, 10000);


    // publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    noise_odom_path_pub_ = nh_.advertise<nav_msgs::Path>("noise_odom_path", 10);

    // subscriber
    odom_sub_ = nh_.subscribe("odom", 10, &Ekf_Node::odomCallback, this);
    imu_sub_= nh_.subscribe("imu", 10, &Ekf_Node::imuCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0/max(freq, 1.0)), &Ekf_Node::spin, this);

    my_filter.reset(new Ekf_Estimation());
}

void Ekf_Node::odomCallback(const OdomConstPtr& msg) 
{
    cout << "odom received!" << endl;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = msg->header.stamp;
    odom_time_  = ros::Time::now();
    cout << "the time difference between stamp and time is : " << odom_time_ - odom_stamp_ << endl;
        
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    odom_meas_(0) = msg->pose.pose.position.x;
    odom_meas_(1) = msg->pose.pose.position.y;
    odom_meas_(2) = yaw;
    odom_meas_(3) = msg->twist.twist.linear.x;
    odom_meas_(4) = msg->twist.twist.linear.y;
    odom_meas_(5) = msg->twist.twist.angular.z;

    // wait to be modified
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i, j) = msg->pose.covariance[6*i+j];

    
    // activate odom
    if (!odom_active_) 
    {
      if (!odom_initializing_)
      {
        odom_initializing_ = true;
        odom_init_stamp_ = odom_stamp_;

        ROS_INFO("Initializing Odom sensor");      
      }
      if ( filter_stamp_ >= odom_init_stamp_)
      {
        odom_active_ = true;
        odom_initializing_ = false;
        ROS_INFO("Odom sensor activated");      
      }
      else 
        ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
        (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
}

void Ekf_Node::imuCallback(const ImuConstPtr& msg)
{
    // this callback 
    cout << "imu received!" << endl;

    // receive data 
    imu_stamp_ = msg->header.stamp;


    // Transforms imu data to base_footprint frame
    if (!robot_state.waitForTransform(base_footprint_frame_, msg->header.frame_id, imu_stamp_, ros::Duration(0.5))){
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_)
        ROS_ERROR("Could not transform imu message from %s to %s", msg->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else if (my_filter->is_Initialized)
        ROS_WARN("Could not transform imu message from %s to %s. Imu will not be activated yet.", msg->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else 
        ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", msg->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }

    if (!imu_transform_received) {
        robot_state.lookupTransform(base_footprint_frame_, "imu_link", imu_stamp_, base_imu_offset);
        imu_transform_received = true;
    }

    imu_time_  = ros::Time::now();
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
        imu_initializing_ = true;
        imu_init_stamp_ = imu_stamp_;
        ROS_INFO("Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
        imu_active_ = true;
        imu_initializing_ = false;
        ROS_INFO("Imu sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }
}

void Ekf_Node::spin(const ros::TimerEvent& e)
{
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    filter_stamp_ = ros::Time::now();
    if (my_filter->is_Initialized) {
        if (odom_active_) {
            // calculate the transformation measurement of odom
            Vector3d delta_pose, delta_hat_pose,
                     old_pose(last_odom_meas_(0),last_odom_meas_(1), last_odom_meas_(2));
            delta_pose(0) = odom_meas_(0) - last_odom_meas_(0);
            delta_pose(1) = odom_meas_(1) - last_odom_meas_(1);
            delta_pose(2) = odom_meas_(2) - last_odom_meas_(2);
            // delta_hat_pose = (rot1, trans, rot2)
            delta_hat_pose = odom_diff_model_delta(old_pose, delta_pose);

            last_odom_meas_ = noise_odom_ = odom_meas_;
            // Apply sampled update to noise odom
            noise_odom_(0) += delta_hat_pose(1) * 
                    cos(noise_odom_(2) + delta_hat_pose(0));
            noise_odom_(1) += delta_hat_pose(1) * 
                    sin(noise_odom_(2) + delta_hat_pose(0));
            noise_odom_(2) += delta_hat_pose(0) + delta_hat_pose(2);

            cout << noise_odom_ - odom_meas_ << endl;

            add_pose_to_path(odom_meas_, odom_path);
            add_pose_to_path(noise_odom_, noise_path);
            odom_path_pub_.publish(odom_path);
            noise_odom_path_pub_.publish(noise_path);
        }
    }

    if (odom_active_ && !my_filter->is_Initialized) {
        // initialize the state with first odom data 
        my_filter->state = odom_meas_;
        my_filter->last_filter_time = odom_stamp_;
        // init last_odom for caculate the delta pose
        last_odom_meas_ = noise_odom_ = odom_meas_;

        initialize_path();
        my_filter->is_Initialized = true;
    }

    if(imu_active_ && !my_filter->imu_Initialized) {        
        tf::transformTFToEigen(base_imu_offset, my_filter->base_imu_offset);
        my_filter->imu_Initialized = true;
        
        // cout << my_filter->base_imu_offset.matrix() << endl;        
        }


    if (my_filter->is_Initialized) {
        cout << "filter initialized ! " << endl;
    }

    // // check which sensors are still active
    // if ((odom_active_ || odom_initializing_) && 
    //     (Time::now() - odom_time_).toSec() > timeout_){
    //   odom_active_ = false; odom_initializing_ = false;
    //   ROS_INFO("Odom sensor not active any more");
    // }
    // if ((imu_active_ || imu_initializing_) && 
    //     (Time::now() - imu_time_).toSec() > timeout_){
    //   imu_active_ = false;  imu_initializing_ = false;
    //   ROS_INFO("Imu sensor not active any more");
    // }


    cout << "this is timer !!" << endl;

}

// !!! IMPORTANT renmember to add time stamp here!!!!
void Ekf_Node::add_pose_to_path(const Matrix<double, 6, 1>& meas, nav_msgs::Path& path) {
  path.header.seq += 1;
  // MODIFY!!!
  path.header.stamp = ros::Time::now();
  //

  geometry_msgs::PoseStamped p;
  p.header = path.header;
  p.pose.position.x = meas(0);
  p.pose.position.y = meas(1);
  p.pose.position.z = 0.0;

  tf::Quaternion q;
  q.setRPY(0, 0, meas(2));
  tf::quaternionTFToMsg(q , p.pose.orientation);
  if (path.poses.size() > max_path_plot) path.poses.erase(path.poses.begin());

  path.poses.push_back(p);
}



void Ekf_Node::initialize_path() {
  odom_path.header.seq = 0;
  odom_path.header.stamp = odom_stamp_;
  odom_path.header.frame_id = "odom";

  noise_path.header = odom_path.header;
  add_pose_to_path(odom_meas_, odom_path);
  add_pose_to_path(odom_meas_, noise_path);
}

Ekf_Node::~Ekf_Node() {
    ROS_INFO("Ekf_Node destructed.");

}


void sigintHandler(int sig)
{
    ROS_INFO("Shutting Down ...");
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_localization");
    Ekf_Node my_node;

    signal(SIGINT, sigintHandler);

    ros::spin();
    return (0);
}