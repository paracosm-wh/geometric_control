#ifndef NONLINEAR_ATTITUDE_CONTROL_H
#define NONLINEAR_ATTITUDE_CONTROL_H

#include "geometric_control/common.h"
#include "geometric_control/control.h"

class NonlinearAttitudeControl : public Control {
 public:
  NonlinearAttitudeControl(double attctrl_tau);
  virtual ~NonlinearAttitudeControl();
  void Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk) override;

 private:
  double attctrl_tau_{1.0};
};

#endif
