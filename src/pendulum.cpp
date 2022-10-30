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
#include "geometry_msgs/msg/vector3.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
using std::placeholders::_1;

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
    declare_parameter("water_current_x", 0.0);
    declare_parameter("water_current_y", 0.0);
    declare_parameter("water_current_z", 0.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "water_current", 10, std::bind(&MinimalPublisher::water_current_callback, this, _1));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  bool first_time = true;
  bool water_current_publishing = false;
  double Vc_mag;
  double Vc_eff;
  double beta; //water current direction with respect to horizontal axis (x)
  double theta_now;
  double Vc;
  double Vc_x;
  double Vc_y;
  double Vc_z;
  int counter_check_topic_pub = 0;
  int counter_water_current_topic = 0;
  int counter_water_current_yaml = 0;

  auto load_initial_conditions() {
    double theta0 = get_parameter("initial_angle").as_double();
    double theta_dot0 = get_parameter("initial_angular_speed").as_double();
    struct initial_conditions {
        double theta0;
        double theta_dot0;

    };    

    return initial_conditions {theta0, theta_dot0};

  }
  void water_current_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) 
    {
      counter_check_topic_pub = 0;
      water_current_publishing = true;
      Vc_x = msg->x;
      Vc_y = msg->y;
      Vc_z = msg->z;
    }
  auto water_current_calculations(double Vc_x ,double Vc_z ) 
    {
      Vc_mag = sqrt(pow(Vc_x,2)+pow(Vc_z,2));
      beta = atan2 (Vc_z,Vc_x); 
      Vc = Vc_x*cos(theta_now) + Vc_z*sin(theta_now);
      struct water_current_result {
        double Vc_mag;
        double beta;
        double Vc;
    };    

    return water_current_result {Vc_mag, beta, Vc};

    }
  auto pendulum( double theta, double theta_dot) {
    counter_check_topic_pub += 1;
    double m = get_parameter("mass").as_double();
    double L = get_parameter("length").as_double();
    double g = get_parameter("gravity").as_double();
    double pw = get_parameter("water_density").as_double();
    double r = get_parameter("pendulum_radius").as_double();
    double Kd = get_parameter("drag_coefficient").as_double();

    double theta_ddot;
    double tau_g;
    double tau_d;
    double tau_b;
    // double tau_f;
    double dt = 0.01;
    double V = (4/3)*M_PI*pow(r,3);
    double Ap = M_PI*pow(r,2);
    // double uf = 0.2;
    if (counter_check_topic_pub >  10){   //if water current topic was publishing and then got interuppted
      water_current_publishing = false;
    }

    if (water_current_publishing){    //if water_current topic is being published
      auto water_current_data = water_current_calculations( Vc_x , Vc_z );
      Vc = water_current_data.Vc;
      Vc_mag = water_current_data.Vc_mag;
      beta = water_current_data.beta;
      counter_water_current_topic +=1;
      if (counter_water_current_topic ==2 || counter_water_current_topic%500 == 0){
        cout << "Water current from topic" << endl;
        cout << "Vc vector: ["<< Vc_x<< ","<< Vc_y<< ","<< Vc_z<< "] m/s , Beta: "<< beta <<" rad"<< endl;
        cout << "Vc effective: "<< Vc<<" m/s"<< endl;   
        cout<<"--------------------------------------------"<<endl;
     
        counter_water_current_yaml = 0;
      }
    }

    else{                           //if no topic is published, parameters will be from yaml file
      Vc_x = get_parameter("water_current_x").as_double();
      Vc_y = get_parameter("water_current_y").as_double();
      Vc_z = get_parameter("water_current_z").as_double();
      auto water_current_data = water_current_calculations( Vc_x , Vc_z );
      Vc = water_current_data.Vc;
      Vc_mag = water_current_data.Vc_mag;
      beta = water_current_data.beta;
      counter_water_current_yaml +=1;
      if (counter_water_current_yaml ==2 || counter_water_current_yaml%500 == 0){
        cout << "Water current from yaml file"<<endl;
        cout << "Vc vector: ["<< Vc_x<< ","<< Vc_y<< ","<< Vc_z<< "] m/s , Beta: "<< beta <<" rad"<< endl;
        cout << "Vc effective: "<< Vc<<" m/s"<< endl;
        cout<<"--------------------------------------------"<<endl;

        counter_water_current_topic = 0;
      }

    }
    
    if (first_time){
    cout<<"--------------------------------------------"<<endl;
    cout<<"pendulum mass         |               "<< m << " kg"<< endl;
    cout<<"--------------------------------------------"<<endl;
    cout<<"pendulum length       |               "<< L << " m"<<endl;
    cout<<"--------------------------------------------"<<endl;
    cout<<"pendulum radius       |               "<< r << " m"<<endl;
    cout<<"--------------------------------------------"<<endl;
    cout<<"water current vector  |               "<<  "["<< Vc_x<< ","<< Vc_y<< ","<< Vc_z<< "]" <<" m/s"<< endl;
    cout<<"--------------------------------------------"<<endl;
    cout<<"drag coeficient       |               "<< Kd <<endl;
    cout<<"---------------------------------------"<<endl;
    first_time = false;
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
      cout<<"--------------------------------------------"<<endl;
      cout<<"initial angle         |               "<< theta0 << " rad"<< endl;
      cout<<"--------------------------------------------"<<endl;
      cout<<"initial angular speed |               "<< theta_dot0 << " rad/s"<<endl;

    }

    auto states_result = pendulum(theta0,theta_dot0) ;
    theta0 = states_result.theta;
    theta_dot0 = states_result.theta_dot;
    theta_now = theta0;
    message.name = {"pendulum_joint"};
    message.position = {states_result.theta};


    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;

  size_t count_;


};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
