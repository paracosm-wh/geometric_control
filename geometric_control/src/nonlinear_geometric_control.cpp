#include "geometric_control/nonlinear_geometric_control.h"

NonlinearGeometricControl::NonlinearGeometricControl(double attctrl_tau) : Control() { attctrl_tau_ = attctrl_tau; }

NonlinearGeometricControl::~NonlinearGeometricControl() {}

void NonlinearGeometricControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                       const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
  // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
  // The original paper inputs moment commands, but for offboard control, angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);

  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  desired_rate_ = (2.0 / attctrl_tau_) * error_att;
  const Eigen::Vector3d zb = rotmat.col(2);
  desired_thrust_(0) = 0.0;
  desired_thrust_(1) = 0.0;
  desired_thrust_(2) = ref_acc.dot(zb);
}
