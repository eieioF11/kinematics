#pragma once
#include <cmath>
#include <iostream>
#include <numeric>
#include <ratio>
#include <vector>

#include "move_base.hpp"

namespace kinematics {

  /**
   * @brief 対向二輪型ロボット逆運動学モデル
   *
   */
  template <typename FLOATING_TYPE>
  class TwoWheels;
  using TwoWheelsf = TwoWheels<float>;
  using TwoWheelsd = TwoWheels<double>;

  template <typename FLOATING_TYPE>
  class TwoWheels : public MoveBase {
    using value_type = FLOATING_TYPE;
  private:
    value_type inv_r_;
  public:
    Two_wheels(){}
    Two_wheels(value_type r,value_type l){// r:wheel radius[m], l:diameter[m]
      R = r;
      L = l;
      inv_r_ = 1./R;
      wheel_speeds.resize(2);
    }

    /**
     * @brief 3次元の速度入力
     *
     * @param vx [m/s]
     * @param vy [m/s]
     * @param vz [m/s]
     * @param angular_x [rad/s]
     * @param angular_y [rad/s]
     * @param angular_z [rad/s]
     */
    void move(value_type vx, value_type vy, value_type vz, value_type angular_x, value_type angular_y, value_type angular_z) {
      wheel_speeds[0] = (vx*inv_r_-(angular_z*L*inv_r_))*RADPS_TO_RPS;
      wheel_speeds[1] = (vx*inv_r_+(angular_z*L*inv_r_))*RADPS_TO_RPS;
    }
  };
/*---------------------
left wheel(l):0
right wheel(r):1
  |-L-|
l[0]--|--[1]r
 x
 |
_|____y
o|   
----------------------*/
} // namespace kinematics
