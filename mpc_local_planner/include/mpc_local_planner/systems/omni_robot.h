/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with thEis program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *  Adapted: Rodrigo Bernardo
 *********************************************************************/

#ifndef SYSTEMS_OMNI_ROBOT_H_
#define SYSTEMS_OMNI_ROBOT_H_

#include <mpc_local_planner/systems/base_robot_se2.h>

#include <cmath>

namespace mpc_local_planner {

/**
 * @brief Simple car model
 *
 * This class implements the dynamics for a simple car model
 * in which the rear wheels are actuated.
 * The front wheels are the steering wheels (for wheelbase > 0).
 * The state vector [x, y, theta] is defined at the center of the rear axle.
 * See [1,2] for a mathematical description and a figure.
 *
 * [1] S. M. LaValle, Planning Algorithms, Cambridge University Press, 2006.
 *     (Chapter 13, http://planning.cs.uiuc.edu/)
 * [2] A. De Luca et al., Feedback Control of a Nonholonomic Car-like Robot,
 *     in Robot Motion Planning and Control (Ed. J.-P. Laumond), Springer, 1998.
 *     (https://homepages.laas.fr/jpl/promotion/chap4.pdf)
 *
 * @see SimpleCarFrontWheelDrivingModel KinematicBicycleModelVelocityInput
 *      BaseRobotSE2 RobotDynamicsInterface corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class OmniRobot : public BaseRobotSE2
{
 public:
    //! Default constructor
    OmniRobot() = default;

    //! Constructs model with given wheelbase
    OmniRobot(double lx, double ly, double r) : _lx(lx), _ly(ly), _r(r) {}

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<OmniRobot>(); }

    // implements interface method
    int getInputDimension() const override { return 2; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() && "OmniRobot::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        f[0] = u[0] * std::cos(x[2]);
        f[1] = u[0] * std::sin(x[2]);
        f[2] = u[0] * std::tan(u[1]) / _lx;
    }

    // implements interface method
    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist) const override
    {
        assert(u.size() == getInputDimension());
        twist.linear.x = u[0];
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.z = u[1];  // warning, this is the angle and not the angular vel
        twist.angular.x = twist.angular.y = 0;

        return true;
    }

    //! Set Robotbase
    void setRobotbase(double lx, double ly, double r) {
        
        _lx = lx; 
        _ly = ly; 
        _r = r;
    
    }
    //! Get wheelbase
    double getlenght_x() const { return _lx; }

     //! Get wheelbase
    double getlenght_y() const { return _ly; }

     //! Get wheelradius
    double getWheelradius() const { return _r; }

 protected:
    double _lx = 1.0;
    double _ly = 1.0;
    double _r = 1.0;
};


}  // namespace mpc_local_planner

#endif  // SYSTEMS_OMNI_ROBOT_H_

/**

  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
**/