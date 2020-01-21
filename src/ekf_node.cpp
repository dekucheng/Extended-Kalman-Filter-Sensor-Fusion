#include "my_ekf/ekf_node.h"

using namespace std;
using namespace Eigen;

Ekf_Node::Ekf_Node():
    private_nh_("~"),
    odom_active_(false),
    imu_active_(false),
    landmark_active_(false),
    odom_initializing_(false),
    imu_initializing_(false),
    odom_update_(false),
    imu_update_(false)
{
    double freq;
    private_nh_.param("spin_rate", freq, 30.0);
    private_nh_.param("time_out", timeout_, 1.0);
    private_nh_.param("odom_used", odom_used_, true);
    private_nh_.param("imu_used", imu_used_, true);
    private_nh_.param("landmark_used", landmark_used_, true);
    private_nh_.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    private_nh_.param("max_path_plot", max_path_plot, 10000);
    private_nh_.param("camera_optical_frame", camera_optical_frame_,std::string("camera_rgb_optical_frame"));

    // publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    noise_odom_path_pub_ = nh_.advertise<nav_msgs::Path>("noise_odom_path", 10);


    // subscriber
    if (odom_used_) {
      odom_sub_ = nh_.subscribe("odom", 10, &Ekf_Node::odomCallback, this);
      ROS_INFO("odom used, activate odom subscriber!");
    }
    if (imu_used_) {
      imu_sub_= nh_.subscribe("imu", 10, &Ekf_Node::imuCallback, this);
      ROS_INFO("imu used, activate imu subscriber!");
    }
    if (landmark_used_) {
      landmark_sub_ = nh_.subscribe("tag_detections", 5, &Ekf_Node::landmarkCallback, this);
      ROS_INFO("landmark used, activate apriltag subscriber!");
    }

    timer_ = nh_.createTimer(ros::Duration(1.0/max(freq, 1.0)), &Ekf_Node::spin, this);
    my_filter.reset(new Ekf_Estimation());
}

void Ekf_Node::odomCallback(const OdomConstPtr& msg) 
{
    // cout << "odom received!" << endl;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = msg->header.stamp;
    odom_time_  = ros::Time::now();

    // tf::Pose pose;
    // tf::poseMsgToTF(msg->pose.pose, pose);
    // double yaw = tf::getYaw(pose.getRotation());
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    // tf::poseMsgToTF(msg->pose.pose, pose);
    double yaw = tf::getYaw(q);

    odom_meas_(0) = msg->pose.pose.position.x;
    odom_meas_(1) = msg->pose.pose.position.y;
    odom_meas_(2) = yaw;
    odom_meas_(3) = msg->twist.twist.linear.x;
    odom_meas_(4) = msg->twist.twist.linear.y;
    odom_meas_(5) = msg->twist.twist.angular.z;

    // // wait to be modified
    // for (unsigned int i=0; i<6; i++)
    //   for (unsigned int j=0; j<6; j++)
    //     odom_covariance_(i, j) = msg->pose.covariance[6*i+j];

    
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
    // cout << "imu received!" << endl;

    // receive data 
    imu_stamp_ = msg->header.stamp;
    imu_time_  = ros::Time::now();

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    // tf::poseMsgToTF(msg->pose.pose, pose);
    imu_meas_ = tf::getYaw(q);

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
        robot_state.lookupTransform(base_footprint_frame_, msg->header.frame_id, ros::Time(0), base_imu_offset);
        imu_transform_received = true;
    }

    
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

void Ekf_Node::landmarkCallback(const LandmarkConstPtr& mark) {
  // check if this callback is useful
  if (mark->detections.size() == 0) {
    landmark_active_ = false;
  }

  // receive mark mesgs
  else {
    landmark_stamp_ = mark->header.stamp;
    landmark_time_ = ros::Time::now();

    // clear landmark_pose_set
    landmark_pose_set.clear();
    // push back tag detections
    for (int i=0; i<mark->detections.size(); i++) {
      landmark_pose p;
      Vector4d trans;

      p.id = mark->detections[i].id[0];
      trans(0) = mark->detections[i].pose.pose.pose.position.x;
      trans(1) = mark->detections[i].pose.pose.pose.position.y;
      trans(2) = mark->detections[i].pose.pose.pose.position.z;
      trans(3) = 1;

      Eigen::Quaternion<double> q(
        mark->detections[i].pose.pose.pose.orientation.w,
        mark->detections[i].pose.pose.pose.orientation.x,
        mark->detections[i].pose.pose.pose.orientation.y,
        mark->detections[i].pose.pose.pose.orientation.z
      );
      Matrix3d m = q.toRotationMatrix();
      p.pose.block<3,3>(0,0) = m;
      p.pose.block<4,1>(0,3) = trans;

      landmark_pose_set.push_back(p);

      // print transform
      // tf::StampedTransform st;
      // robot_state.lookupTransform("odom", "tag_0", ros::Time(0), st);
      // Affine3d transf;
      // tf::transformTFToEigen(st, transf);
      // cout << "tag position is: " << endl;
      // cout << transf.matrix() << endl;

      // cout << "tag position in camera frame is :" << endl;
      // cout << p.pose << endl;
    }


    if (!landmark_active_) {
      if (!landmark_initializing_){
        landmark_initializing_ = true;
        landmark_init_stamp_ = landmark_stamp_;
        ROS_INFO("Initializing landmark");      
      }
      if ( filter_stamp_ >= landmark_init_stamp_){
        landmark_active_ = true;
        landmark_initializing_ = false;
        ROS_INFO("Landmark activated");      
      }
      else ROS_DEBUG("Waiting to activate landmark, because landmark measurements are still %f sec in the future.", 
        (landmark_init_stamp_ - filter_stamp_).toSec());
    }
  }
}



void Ekf_Node::spin(const ros::TimerEvent& e)
{
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    filter_stamp_ = ros::Time::now();
    if (my_filter->is_Initialized) {
      ros::Time this_update_time = filter_stamp_;
        // check if update odom
        if (odom_active_ && ((odom_stamp_ - my_filter->last_filter_time).toSec() > 0.01)) {
            // calculate the transformation measurement of odom
            Vector3d delta_pose, delta_hat_pose,
                     old_pose(last_odom_meas_(0),last_odom_meas_(1), last_odom_meas_(2));
            delta_pose(0) = odom_meas_(0) - last_odom_meas_(0);
            delta_pose(1) = odom_meas_(1) - last_odom_meas_(1);
            delta_pose(2) = odom_meas_(2) - last_odom_meas_(2);
            // delta_hat_pose = (rot1, trans, rot2)
            delta_hat_pose = odom_diff_model_delta(old_pose, delta_pose);

            noise_odom_ = odom_meas_;
            // prepare the noise odom data (delta_hat_pose, dx, dy, dtheta)
            noise_odom_(0) = last_odom_meas_(0) + delta_hat_pose(1) * 
                    cos(last_odom_meas_(2) + delta_hat_pose(0));
            noise_odom_(1) = last_odom_meas_(1) + delta_hat_pose(1) * 
                    sin(last_odom_meas_(2) + delta_hat_pose(0));
            noise_odom_(2) = last_odom_meas_(2) + delta_hat_pose(0) + delta_hat_pose(2);

            // add measurement
            my_filter->addmeasurement(noise_odom_);  
            // reset last_odom
            last_odom_meas_ = odom_meas_;

            this_update_time = min(this_update_time, odom_stamp_);
            odom_update_ = true;
        }
        else {
          odom_update_ = false;
          ROS_INFO ("odom too old, not update with odom in this callback");
        }
        // check if update imu 
        if (imu_active_ && my_filter->imu_Initialized && ((imu_stamp_ = my_filter->last_filter_time).toSec() > 0.01)) {
          this_update_time = min(this_update_time, imu_stamp_);  
          my_filter->addmeasurement(imu_meas_);
          imu_update_ = true;
        }
        else {
          imu_update_ = false;
          ROS_INFO("imu data too old, not update with imu in this callback!");
        }
  
        if (my_filter -> update(odom_update_, imu_update_, this_update_time)) {
            add_pose_to_path(odom_meas_.block<3,1>(0,0), odom_path);
            Vector3d state = my_filter -> get_state();
            add_pose_to_path(state, noise_path);
            odom_path_pub_.publish(odom_path);
            noise_odom_path_pub_.publish(noise_path);
            // print out error
            // double err = sqrt(pow(odom_meas_(0)-state(0), 2) +
            //                   pow(odom_meas_(1)-state(1), 2) + 
            //                   pow(odom_meas_(2)-state(2), 2));
            // cout << "<==============the error is: " << err << endl;
            // cout << "true odom meas is: " << odom_meas_ << endl;
        }
        else {
          ROS_INFO("EkF filter not update, skip this spin");
        }
    }

    if (odom_active_ && !my_filter->is_Initialized) {
        // initialize the state with first odom data 
        last_odom_meas_ = odom_meas_;
        my_filter->Init(odom_meas_.block<3,1>(0,0), odom_stamp_);
        initialize_path();
    }

    if(imu_active_ && !my_filter->imu_Initialized) {
        my_filter->Init_imu(base_imu_offset);
        // cout << my_filter->base_imu_offset.matrix() << endl;        
    }

    if (landmark_active_ && !my_filter->landmark_Initialized) {
      tf::StampedTransform t;
      robot_state.lookupTransform(base_footprint_frame_, camera_optical_frame_, ros::Time(0), t);
      my_filter->Init_landmark(t);
    }

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


    // cout << "this is timer !!" << endl;

// !!! IMPORTANT renmember to add time stamp here!!!!
void Ekf_Node::add_pose_to_path(const Vector3d& meas, nav_msgs::Path& path) {
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
  add_pose_to_path(odom_meas_.block<3,1>(0,0), odom_path);
  add_pose_to_path(odom_meas_.block<3,1>(0,0), noise_path);
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