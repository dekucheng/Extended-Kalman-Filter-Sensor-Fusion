#include "my_ekf/utils.h"

using namespace std;
using namespace ros;
using namespace Eigen;

double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

double diff_angle(const double a)
{
  double d1, d2;
  d1 = fabs(a);
  d2 = M_PI * 2 - d1;
  if (d1 <= d2)
    return d1;
  else 
    return d2;
}

double ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}


Vector3d odom_diff_model_delta(Vector3d& old_pose, Vector3d& delta_pose)
{
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(delta_pose(1)*delta_pose(1) + 
            delta_pose(0)*delta_pose(0)) < 0.01)
        delta_rot1 = 0.0;
    else
        delta_rot1 = angle_diff(atan2(delta_pose(1), delta_pose(0)),
                                old_pose(2));
    delta_trans = sqrt(delta_pose(0)*delta_pose(0) +
                        delta_pose(1)*delta_pose(1));
    delta_rot2 = angle_diff(delta_pose(2), delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));


    double alpha1, alpha2, alpha3, alpha4;
    alpha1 = alpha2 = alpha3 = alpha4 = 0.2;
    // Sample pose differences
    delta_rot1_hat = angle_diff(delta_rot1,
                                ran_gaussian(alpha1*delta_rot1_noise*delta_rot1_noise +
                                                alpha2*delta_trans*delta_trans + 0.1));
    delta_trans_hat = delta_trans - 
            ran_gaussian(alpha3*delta_trans*delta_trans +
                            alpha4*delta_rot1_noise*delta_rot1_noise +
                            alpha4*delta_rot2_noise*delta_rot2_noise + 0.01);
    delta_rot2_hat = angle_diff(delta_rot2,
                                ran_gaussian(alpha1*delta_rot2_noise*delta_rot2_noise +
                                                alpha2*delta_trans*delta_trans + 0.1));
    Vector3d res(delta_rot1_hat, delta_trans_hat, delta_rot2_hat);
    return res;
}

Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rx * ry * rz;
}