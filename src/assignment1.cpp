#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include "camMover.hpp"
#include "robotMover.hpp"
#include <iostream>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

using namespace std;

const int TIME_SLEEP = 10;

int main(int argc, char** argv)
{
  using namespace chrono_literals;
  rclcpp::init(argc, argv);
  char c;
  
  cout<<"Hi!"<<endl;
  cout<<"We are Amir and Sayna"<<endl;
  cout<<"Welcome to the experimental robotics laboratory!"<<endl;
  cout<<"Please Wait ..."<<endl;

  for (int i = 0; i<TIME_SLEEP; i++)
  {
    cout<<"Please Wait "<< TIME_SLEEP - i << "seceonds!"<<endl;
    // Wait 10 seconds for the system to be ready
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  system("clear");


  cout << "Running..." << std::endl;
  cout << "+------------------------------------------------------------------+\n"
          "|     Assignment 1 for Experimental robotics laboratory            |\n"
          "|                                                                  |\n"
          "|     > c detect the markers with the camera                       |\n"
          "|     > r detect the markers with the movement of the whole robot  |\n"
          "|                                                                  |\n"
          "+------------------------------------------------------------------+\n"
        << std::endl;

  cout << "Selection :";
  cin >> c;

  system("clear");

  if (c == 'c')
  {
    cout<<"Marker Detection by the camera"<<endl;
    rclcpp::spin(std::make_shared<CamMover>());
  }
  else
  {
    cout<<"Marker Detection by the whole robot"<<endl;
    rclcpp::spin(std::make_shared<RobotMover>());
  }

  rclcpp::shutdown();
  return 0;
}
