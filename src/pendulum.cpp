// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>

// #include "geometry_msgs/msg/PoseStamped.h"
using namespace std::chrono_literals;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("pendulum", rclcpp::NodeOptions())
  {
    declare_parameter("mass", 3.0);
    declare_parameter("initial_angle", 1.57);
    declare_parameter("initial_angular_speed", 0.0);
    declare_parameter("length", 3.0);
    declare_parameter("gravity", 9.8);
    declare_parameter("water_speed", 0.0);
    declare_parameter("water_density", 1000.0);
    declare_parameter("pendulum_radius", 0.05);
    declare_parameter("drag_coefficient", 1.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  bool first_time = true;
  auto load_initial_conditions() {
    double theta0 = get_parameter("initial_angle").as_double();
    double theta_dot0 = get_parameter("initial_angular_speed").as_double();
    struct initial_conditions {
        double theta0;
        double theta_dot0;

    };    

    return initial_conditions {theta0, theta_dot0};

  }

  auto pendulum( double theta, double theta_dot) {

    double m = get_parameter("mass").as_double();
    double L = get_parameter("length").as_double();
    double g = get_parameter("gravity").as_double();
    double Vc = get_parameter("water_speed").as_double();
    double pw = get_parameter("water_density").as_double();
    double r = get_parameter("pendulum_radius").as_double();
    double Kd = get_parameter("drag_coefficient").as_double();
    double theta_ddot;
    double tau_g;
    double tau_d;
    double tau_b;
    // double tau_f;
    double dt = 0.05;
    double V = (4/3)*M_PI*pow(r,3);
    double Ap = M_PI*pow(r,2);
    // double uf = 0.2;
    
    if (first_time){
    cout<<"---------------------------------------"<<endl;
    cout<<"pendulum mass         |          "<< m << endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"pendulum length       |          "<< L << endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"pendulum radius       |          "<< r << endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"water speed           |          "<< Vc << endl;
    cout<<"---------------------------------------"<<endl;
    cout<<"drag coeficient       |          "<< Kd << endl;
    cout<<"---------------------------------------"<<endl;
    first_time = false;
    }
    if (theta <= M_PI_2 && theta >= -M_PI_2 ){
      Vc = Vc;
    }
    else {
      Vc = 0;
    }
 
    tau_g = m*g*L*sin(theta);   //gravity
 
   
    // tau_d = Kd*abs(L*theta_dot-Vc)*(L*theta_dot-Vc)*L;  //drag
    tau_d = 0.5*pw*Ap*Kd*abs(L*theta_dot-Vc)*(L*theta_dot-Vc)*L;  //drag

    tau_b = -pw * V * g*L*sin(theta);     //bouyancy
    // tau_f = - uf *(m*g - pw * V * g) * L;     //friction

    theta_ddot = -(tau_g + tau_d + tau_b)/(m*pow(L,2));
 

    theta_dot = theta_dot + theta_ddot * dt;

    theta = theta + theta_dot * dt;
    struct states {
        double theta;
        double theta_dot;
        double theta_ddot;
    };    
    return states {theta, theta_dot, theta_ddot};
    }


private:
  void timer_callback()
  {

    auto message = sensor_msgs::msg::JointState();
    double theta0;
    double theta_dot0;
    message.header.stamp = rclcpp::Node::now	()	;
    if(first_time){
      auto initial_conditions = load_initial_conditions() ;
      theta0 = initial_conditions.theta0;
      theta_dot0 = initial_conditions.theta_dot0;
      cout<<"---------------------------------------"<<endl;
      cout<<"initial angle         |          "<< theta0 << endl;
      cout<<"---------------------------------------"<<endl;
      cout<<"initial angular speed |          "<< theta_dot0 << endl;

    }

    auto states_result = pendulum(theta0,theta_dot0) ;
    theta0 = states_result.theta;
    theta_dot0 = states_result.theta_dot;
    // cout << "  = "<< endl;

    // cout << "Theta_ddot = "<<states_result.theta_ddot<< endl;
    // cout << "Theta_dot = "<<states_result.theta_dot<< endl;
    // cout << "Theta = "<<states_result.theta<< endl;
    // cout << "  = "<< endl;

    message.name = {"pendulum_joint"};
    message.position = {states_result.theta};


    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;


};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
