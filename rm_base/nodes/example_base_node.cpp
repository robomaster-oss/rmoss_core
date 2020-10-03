/****************************************************************************
 *  Copyright (C) 2019 RoboMasterOS.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : example_base_node.h
 *  brief : the node of robot_base.(example)
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-04-03
 ***************************************************************************/
#include "ros/ros.h"
#include "ros/package.h"
#include "robot_base/serialport_trans_dev.h"
#include "robot_base/robot_base_example.h"
#include <signal.h>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}


int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "robot_base",ros::init_options::NoSigintHandler);
  std::string conf_path = ros::package::getPath("robot_base") + "/res/config.yaml";
  string dev_path;
  FileStorage f(conf_path, FileStorage::READ);
  if (!f.isOpened()){
        cerr << "Failed to open " << conf_path << endl;
        return -1;
  }
  f["dev_path"] >> dev_path;
  f.release();
  //creat message transmit dev
  robot_base::SerialPortTransDev trans_dev;
  trans_dev.init(dev_path);
  //data process logic class
  robot_base::RobotBaseExample example;
  example.init(&trans_dev);
  //spin thread
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  //listen
  ros::Rate loop_rate(100);
  while(ros::ok()){
      example.listenDev();
      loop_rate.sleep();//周期休眠
  }
  //wait
  ros::waitForShutdown();
  return EXIT_SUCCESS;
}








