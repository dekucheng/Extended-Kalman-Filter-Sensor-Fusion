#ifndef _EKF_UTILS_
#define _EKF_UTILS_

#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <math.h>

#include "apriltag_ros/AprilTagDetectionArray.h"


using namespace std;
using namespace ros;
using namespace Eigen;

typedef struct {
  int id;
  Matrix4d pose;
} landmark_pose;

typedef struct {
  int id;
  Matrix<double, 6, 1> pose;
} landmark_pose_euler;




double normalize(double z);

double angle_diff(double a, double b);

double ran_gaussian(double sigma);

Vector3d odom_diff_model_delta(Vector3d& old_pose, Vector3d& delta_pose);

Affine3d create_rotation_matrix(double ax, double ay, double az);

#endif
