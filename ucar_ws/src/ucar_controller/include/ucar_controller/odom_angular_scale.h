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

inline double scaleOdomLateralVelocity(double raw_vy,
                                       double lateral_scale) {
  return raw_vy * lateral_scale;
}

}  // namespace ucar_controller

#endif  // UCAR_CONTROLLER_ODOM_ANGULAR_SCALE_H_
