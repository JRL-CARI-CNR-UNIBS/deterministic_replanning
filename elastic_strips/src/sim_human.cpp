/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_human");
  ros::NodeHandle nh;

  ros::Publisher poses_pub=nh.advertise<geometry_msgs::PoseArray>("/poses",1);

  // leggo i nomi dei link di inizio e fine della catena cinematica che voglio usare
  std::string base_frame;
  if (!nh.getParam("base_frame",base_frame))
  {
    ROS_ERROR("Unable to find base_frame, do you load it?");
    return 0;
  }


  // se si sta usando Gazebo, il /clock è fornito da Gazebo, per dare tempo al nodo di riceverlo aspettiamo un secondo.
  ros::WallDuration(1).sleep();

  ros::Time t0=ros::Time::now();

  ros::Rate rate(30);
  while (ros::ok())
  {
    double t=(ros::Time::now()-t0).toSec();

    geometry_msgs::PoseArray human_poses;
    human_poses.header.frame_id=base_frame;

    geometry_msgs::Pose pose;
    pose.orientation.x=0;
    pose.orientation.y=0;
    pose.orientation.z=0;
    pose.orientation.w=1; // orientamento come world

    pose.position.x= -1.5+0.5*std::sin(t/10*2*M_PI);
    pose.position.y=  0+0.5*std::cos(t/10*2*M_PI);
    pose.position.z=  1;

    human_poses.poses.push_back(pose);
    human_poses.header.stamp=ros::Time::now();
    poses_pub.publish(human_poses);

    rate.sleep();
  }
  return 0;
}
