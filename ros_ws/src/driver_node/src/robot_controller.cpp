#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

#include "header/Navigation.h"

#include <sstream>

Navigation *nav;

char cmd[50];

ros::Publisher cmd_vel_pub;

ros::Publisher cmd_shoot_pub;

ros::Publisher cmd_laser_pub;

ros::Publisher occ_grid_pub;

ros::Publisher path_pub;

bool activePointCloud = false;

tf2_ros::Buffer tfBuffer;

Vector goal = Vector(0,0);

Vector movement = Vector(0,0,0);

void cmd_velocity()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist base_cmd;

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   

    if(cmd[0]=='a' && cmd[1]=='u' && cmd[2]=='t' && cmd[3]=='o') {
      base_cmd.linear.x = movement.x;
      base_cmd.linear.y = movement.y;
      base_cmd.angular.z = movement.z;
    }
    else {
   
      // move forward
      if(cmd[0]=='w'){
        base_cmd.linear.x = 0.1;
      } 
      // turn left (yaw)
      else if(cmd[0]=='a'){
        base_cmd.angular.z = 0.2;
      } 
      // turn right (yaw)
      else if(cmd[0]=='d'){
        base_cmd.angular.z = -0.2;
      }
      // move backward
      else if(cmd[0]=='s'){
        base_cmd.linear.x = -0.1;
      }
      // stop
      else if(cmd[0]=='0'){
        base_cmd.linear.x = 0.0;
      }

    }
    
    cmd_vel_pub.publish(base_cmd);

    ros::spinOnce();

    loop_rate.sleep();
  }

}

void cmd_shooting()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(cmd[0]=='k') {
      geometry_msgs::Twist base_cmd;
      cmd_shoot_pub.publish(base_cmd);
      ros::spinOnce();
    }
    else if(cmd[0]=='l') {
      geometry_msgs::Twist base_cmd;
      cmd_laser_pub.publish(base_cmd);
      ros::spinOnce();
    }

    loop_rate.sleep();
  }

}


void driveToGoal(geometry_msgs::Transform transform, std::vector<std::vector<int>> path) {
  ros::Rate loop_rate(10);

  if(cmd[0]=='a' && cmd[1]=='u' && cmd[2]=='t' && cmd[3]=='o') {
    
    movement = Vector(0,0,0);
    
    if(path.size() > 2) {
    
      double quatx= transform.rotation.x;
      double quaty= transform.rotation.y;
      double quatz= transform.rotation.z;
      double quatw= transform.rotation.w;
      
      // get orientation of robot
      /**< quaternion -> rotation Matrix */
      tf::Quaternion rotation(quatx, quaty, quatz, quatw);
      tf::Matrix3x3 m(rotation);
      
      /**< rotation Matrix -> rpy */
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      Vector position = Vector(transform.translation.x, transform.translation.y);
      Vector nextGoal = Vector(path[1][0]/100.0f, path[1][1]/100.0f);
      yaw = Transformations::degrees(yaw) + 90.0f;
      
      // compute angle to next goal align to
      Vector difference = nextGoal.sub(position).normalize();
      float angle = Transformations::SignedAngleBetweenVectorAndDirection(difference, yaw);
      std::cout << "Rotation: " << yaw << " Angle: " << angle << std::endl;
      std::cout << "Position: " << position.x << " " << position.y << endl;
      std::cout << "Next Goal: " << nextGoal.x << " " << nextGoal.y << endl;
      std::cout << "Goal: " << goal.x/100.0f << " " << goal.y/100.0f << endl;
      std::cout << std::endl;
      
      // set movement for /cmd_vel topic
      movement.z = std::min((angle)/200.0f, 0.2f);
      movement.z = std::max(movement.z, -0.2);
      
      movement.x = std::max(std::min(0.2f / std::abs(angle/50.0f), 0.2f), 0.0f);
      if(std::abs(angle) > 20.0f)
        movement.x = 0;
      
    }
    else {
      std::cout << "No path" << std::endl;
    }
  }
}

void publishPath(std::vector<std::vector<int>> pathVector) {
  nav_msgs::Path path;
  
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  for(int i = 0; i < pathVector.size(); i++) {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = pathVector[i][0]/100.0f;
    pose.pose.position.y = pathVector[i][1]/100.0f;
    pose.pose.position.z = 0.0f;

    path.poses.push_back(pose);
  }
  
  path_pub.publish(path);  
  
  ros::spinOnce();
}

void publishOccupancyGrid(std::vector<std::vector<int>> wallPoints) {
    ros::Rate loop_rate(10);

    nav_msgs::OccupancyGrid newGrid;
    newGrid.info.resolution = 0.1f;         // float32
    newGrid.info.width      = wallPoints.size();           // uint32
    newGrid.info.height     = wallPoints[0].size();           // uint32
    
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "map";
    transformStamped.transform.translation.x = nav->map.WallPointsOffset.x/10.0f;
    transformStamped.transform.translation.y = nav->map.WallPointsOffset.y/10.0f;;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    br.sendTransform(transformStamped);

    std::vector<signed char> a(0);
    for(int y = 0; y < wallPoints[0].size(); y++) {
        for(int x = 0; x < wallPoints.size(); x++) {
            a.push_back(wallPoints[x][y]);
        }
    }

    newGrid.data = a;

    occ_grid_pub.publish(newGrid);
    
    ros::spinOnce();

    loop_rate.sleep();
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if(msg->width > 3 && !activePointCloud) {
    activePointCloud = true;  
  
    sensor_msgs::PointCloud2 input_pointcloud = *msg;
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(input_pointcloud, out_pointcloud);

    // saving points of point cloud
    std::vector<Vector> points = std::vector<Vector>(0);
    for(int i = 0 ; i < out_pointcloud.points.size(); ++i){
        points.push_back(Vector(out_pointcloud.points[i].x*100.0f, out_pointcloud.points[i].y*100.0f, out_pointcloud.points[i].z*100.0f));
    }
  
    // creating map out of points
    nav->setWallPositions(points);
    nav->map.filterMap(10.0f, -1.0f);
    
    // navigation
    try{
      geometry_msgs::TransformStamped transformRobot = tfBuffer.lookupTransform("world", "camera", ros::Time(0));
      
      Vector position = Vector(transformRobot.transform.translation.x*100.0, transformRobot.transform.translation.y*100.0, 0.0f);
      std::vector<std::vector<int>> path = nav->findRoute(position, goal, 0.1f);
      
      publishPath(path);
      
      // saving map as image
      nav->createImageOfMap("map.ppm", Vector(0,0), { 50, 200, 90 }, 700, 700);
      std::vector<std::vector<int>> wallPoints = nav->getWallPoints();
      
      // publishing occupancy grid
      publishOccupancyGrid(wallPoints);
      
      // driving to goal
      driveToGoal(transformRobot.transform, path);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    activePointCloud = false;
  }
}

void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("Received pose: " << msg);
    goal.x = msg->pose.position.x*100.0f;
    goal.y = msg->pose.position.y*100.0f;
}

void visualizationMarkerCallBack(const visualization_msgs::Marker::ConstPtr& msg) {
    if(msg->text == "PERSON" && cmd[0]=='a' && cmd[1]=='u' && cmd[2]=='t' && cmd[3]=='o') {
      cmd[0] = '0';
      std::cout << "Found person!" << std::endl;
    }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  nav = new Navigation("map.map");

  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "cmd_vel");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   
	ros::NodeHandle n;
  
  ros::Subscriber sub_points = n.subscribe("/orb_slam3/all_points", 1000, pointCloudCallback);
  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1000, poseCallBack);
  ros::Subscriber sub_marker = n.subscribe("/visualization_marker", 1000, visualizationMarkerCallBack);

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  cmd_laser_pub = n.advertise<geometry_msgs::Twist>("laser", 1000);
  cmd_shoot_pub = n.advertise<geometry_msgs::Twist>("shoot", 1000);
  occ_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out", 1000);
  path_pub = n.advertise<nav_msgs::Path>("path_out", 1000);
  
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  boost::thread cmd_vel{cmd_velocity};
  
  boost::thread cmd_shoot{cmd_shooting};

  while (ros::ok())
  {
    std::cin.getline(cmd, 50);
    if(cmd[0]!='w' && cmd[0]!='s' && cmd[0]!='d' && cmd[0]!='a' && cmd[0]!='0' && cmd[0]!='.')
    {
      std::cout << "unknown command:" << cmd << "\n";
      continue;
    }
    
    // quit
    if(cmd[0]=='.'){
      break;
    }
  }


  return 0;
}
