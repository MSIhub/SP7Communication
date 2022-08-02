#include "mcaFilter.h"



/**
 *
 *
 */
Matrix3d McaFilter::angular_to_gimbal_matrix(double a1, double a2, double a3)
{
  Matrix3d gimbal;
  gimbal << 1, (sin(a1) * sin(a2)) / cos(a2), -(cos(a1) * sin(a2)) / cos(a2), 0,
      cos(a1), sin(a1), 0, -sin(a1) / cos(a2), cos(a1) / cos(a2);

  return gimbal;
}

/**
 *
 */
Matrix3d McaFilter::get_R_process(double roll, double pitch, double yaw)
{
  Matrix3d R_p;

  R_p << cos(pitch) * cos(roll),
      sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch),
      cos(roll) * sin(yaw) * sin(pitch) - cos(yaw) * sin(roll), -sin(pitch),
      cos(yaw) * cos(pitch), cos(pitch) * sin(yaw), cos(pitch) * sin(roll),
      cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw),
      cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);

  return R_p;
}

/**
 *
 */
Matrix3d McaFilter::get_T_process(double roll, double pitch, double yaw)
{
  Matrix3d T_p;
  T_p << 1, (cos(yaw) * sin(pitch)) / cos(pitch),
      (sin(yaw) * sin(pitch)) / cos(pitch), 0, -sin(yaw), cos(yaw), 0,
      cos(yaw) / cos(pitch), sin(yaw) / cos(pitch);

  return T_p;
}

/**
 * \param   f_L is low pass filtered trasnlation data
 *  \return is the acceleration of platform in platform frame
 */
Vector3d McaFilter::tilt_cord(Vector3d f_L)
{
  return {f_L[1] / g_acc, -f_L[0] /g_acc, 0.0};
}


