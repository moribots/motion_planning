/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/GridCells.h>
#include <geometry_msgs/Pose.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "map/grid.hpp"

using rigid2d::Vector2D;

int main(int argc, char** argv)
{
  ROS_INFO("STARTING NODE: grid_map");

  // Vars
  double frequency = 10.0;
  // resolution by which to transform marker poses (10 cm/cell so we use 10)
  double resolution = 0.01;
  std::string frame_id = "base_footprint";
  XmlRpc::XmlRpcValue xml_obstacles;

  // Grid Parameters
  double inflate = 0.1;
  double SCALE = 10.0;
  std::string map_type = "grid_map";

  // store Obstacle(s) here to create Map
  std::vector<map::Obstacle> obstacles_v;

  ros::init(argc, argv, "grid_map_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);
  nh_.getParam("obstacles", xml_obstacles);
  nh_.getParam("map_frame_id", frame_id);
  nh_.getParam("inflate", inflate);
  nh_.getParam("resolution", resolution);
  nh_.getParam("scale", SCALE);

  ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);

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
          for(int j = 0; j < xml_obstacles[i].size(); ++j)
          {
            // Vector2D contains x,y coords
            if(xml_obstacles[i][j].size() != 2)
            {
               ROS_ERROR("Vertex[%d] of obstacles[%d] is not a pair of coordinates", j, i);
            } else if(
              xml_obstacles[i][j][0].getType() != XmlRpc::XmlRpcValue::TypeDouble or
              xml_obstacles[i][j][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
              ROS_ERROR("The coordinates of vertex[%d] of obstacles[%d] are not doubles", j, i);
            } else {
              // PASSED ALL THE TESTS: push Vector2D to vertices list in Obstacle object
              rigid2d::Vector2D vertex(xml_obstacles[i][j][0], xml_obstacles[i][j][1]);
              // NOTE: SCALE DOWN
              vertex.x /= SCALE;
              vertex.y /= SCALE;
              obs.vertices.push_back(vertex);
            }
          }
      }
      // push Obstacle object to vector of Object(s) in Map
      obstacles_v.push_back(obs);
    }
  }


  // Initialize Grid
  map::Grid grid(obstacles_v, inflate);

  // Build Map
  grid.build_map(resolution);

  ROS_INFO("Grid Built!");

  // Get map bounds
  auto bounds = grid.return_map_bounds();
  auto gridsize = grid.return_grid_dimensions();

  // rviz representation of the grid
  std::vector<int8_t> map;
  grid.occupancy_grid(map);

  // Grid Pose
  geometry_msgs::Pose map_pose;
  map_pose.position.x = bounds.at(0).x;
  map_pose.position.y = bounds.at(0).y;
  map_pose.position.z = 0.0;
  map_pose.orientation.x = 0.0;
  map_pose.orientation.y = 0.0;
  map_pose.orientation.z = 0.0;
  map_pose.orientation.w = 1.0;

  // Occupancy grid for visualization
  nav_msgs::OccupancyGrid grid_map;
  grid_map.header.frame_id = frame_id;
  grid_map.info.resolution = resolution;
  grid_map.info.width = gridsize.at(0);
  grid_map.info.height = gridsize.at(1);

  grid_map.info.origin = map_pose;

  while(ros::ok())
  {
    ros::spinOnce();
    grid_map.header.stamp = ros::Time::now();
    grid_map.info.map_load_time = ros::Time::now();
    grid_map.data = map;
    grid_pub.publish(grid_map);
  }

  return 0;
}
