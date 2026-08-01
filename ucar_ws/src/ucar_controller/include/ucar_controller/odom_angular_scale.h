#ifndef UCAR_CONTROLLER_ODOM_ANGULAR_SCALE_H_
#define UCAR_CONTROLLER_ODOM_ANGULAR_SCALE_H_

namespace ucar_controller {

inline double scaleOdomAngularVelocity(double raw_vth,
                                       double ccw_scale,
                                       double cw_scale) {
  if (raw_vth > 0.0) {
    return raw_vth * ccw_scale;
  }
  if (raw_vth < 0.0) {
    return raw_vth * cw_scale;
  }
  return 0.0;
}

}  // namespace ucar_controller

#endif  // UCAR_CONTROLLER_ODOM_ANGULAR_SCALE_H_
