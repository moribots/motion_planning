/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include <ros/ros.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>

#include "map/map.hpp"
#include "nuslam/TurtleMap.h"

#include <functional>  // To use std::bind
#include <visualization_msgs/Marker.h>

// Used to convert YAML list of lists to std::vector
#include <xmlrpcpp/XmlRpcValue.h> // catkin component


int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: draw_map");
  // Vars
  double frequency = 60.0;
  XmlRpc::XmlRpcValue xml_obstacles;

  // store Obstacle(s) here to create Map
  std::vector<map::Obstacle> obstacles_v;

  ros::init(argc, argv, "draw_map_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);
  nh_.getParam("obstacles", xml_obstacles);

  // 'obstacles' is a triple-nested list.
  // 1st level: obstacle (Obstacle), 2nd level: vertices (std::vector), 3rd level: coordinates (Vector2D)

  // std::vector<Obstacle>
  if(xml_obstacles.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("There is no list of obstacles");
  } else {
    for(int i = 0; i < xml_obstacles.size(); ++i)
    {
      // Obstacle contains std::vector<Vector2D>
      // create Obstacle with empty vertex vector
      map::Obstacle obs;
      if(xml_obstacles[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
          ROS_ERROR("obstacles[%d] has no vertices", i);
      } else {
          for(int j = 0; j < xml_obstacles.size(); ++j)
          {
            // Vector2D contains x,y coords
            if(xml_obstacles[i][j].size() != 2)
            {
               ROS_ERROR("Vertex[%d] of obstacles[%d] is not a pair of coordinates", j, i);
            } else if(
             xml_obstacles[i][j][0].getType() != XmlRpc::XmlRpcValue::TypeDouble or
             xml_obstacles[i][j][1].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
               ROS_ERROR("The coordinates of vertex[%d] of obstacles[%d] are not doubles", i);
            } else {
              // PASSED ALL THE TESTS: push Vector2D to vertices list in Obstacle object
              rigid2d::Vector2D vertex(xml_obstacles[i][j][0], xml_obstacles[i][j][1]);
              obs.vertices.push_back(vertex);
            }
          }
      }
      // push Obstacle object to vector of Object(s) in Map
      obstacles_v.push_back(obs);
    }
  }

  // Create Map
  map::Map map(obstacles_v);

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    rate.sleep();
  }

  return 0;
}