#ifndef NONLINEAR_GEOMETRIC_CONTROL_H
#define NONLINEAR_GEOMETRIC_CONTROL_H

#include "geometric_control/common.h"
#include "geometric_control/control.h"

class NonlinearGeometricControl : public Control {
 public:
  NonlinearGeometricControl(double attctrl_tau);
  virtual ~NonlinearGeometricControl();
  void Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk) override;

 private:
  double attctrl_tau_{1.0};
};

#endif
