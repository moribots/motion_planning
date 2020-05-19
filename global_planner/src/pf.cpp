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

#include "map/map.hpp"
#include "map/prm.hpp"
#include "map/grid.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include "global_planner/potential_field.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <functional>  // To use std::bind

// Used to deal with YAML list of lists
#include <xmlrpcpp/XmlRpcValue.h> // catkin component


int main(int argc, char** argv)
/// The Main Function ///
{
    ROS_INFO("STARTING NODE: pf");

    // Vars
    double frequency = 10.0;
    // Scale by which to transform marker poses (10 cm/cell so we use 10)
    double SCALE = 10.0;
    std::string frame_id = "base_footprint";
    XmlRpc::XmlRpcValue xml_obstacles;

    // Map Parameters
    double thresh = 0.01;
    double inflate = 0.1;

    std::vector<double> start_vec{7.0, 3.0};
    std::vector<double> goal_vec{7.0, 26.0};

    // PF Parameters
    double eta = 0.1;
    double ada = 1000.0;
    double zeta = 0.1;
    double d_thresh = 2.0;
    double Q_thresh = 0.1;

    // store Obstacle(s) here to create Map
    std::vector<map::Obstacle> obstacles_v;

    ros::init(argc, argv, "pf_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    ros::NodeHandle nh_("~"); // get a handle to ROS
    // Parameters
    nh_.getParam("frequency", frequency);
    nh_.getParam("obstacles", xml_obstacles);
    nh_.getParam("map_frame_id", frame_id);
    nh_.getParam("thresh", thresh);
    nh_.getParam("inflate", inflate);
    nh_.getParam("scale", SCALE);
    nh_.getParam("start", start_vec);
    nh_.getParam("goal", goal_vec);

    // START and GOAL
    rigid2d::Vector2D start(start_vec.at(0)/SCALE, start_vec.at(1)/SCALE);
    rigid2d::Vector2D goal(goal_vec.at(0)/SCALE, goal_vec.at(1)/SCALE);

    // Path Publishers
    ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("robot_path", 1);

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
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = start.x;
    ps.pose.position.y = start.y;
    std::vector<geometry_msgs::PoseStamped> robot_poses{ps};
    nav_msgs::Path path;
    path.header.frame_id = frame_id;

    global::PotentialField PF(obstacles_v, eta, ada, zeta, d_thresh, Q_thresh);
    bool terminate = PF.return_terminate();

    ros::Rate rate(frequency);

    // Main While
    while (ros::ok())
    {
        ros::spinOnce();

        path.header.stamp = ros::Time::now();

        if (!terminate)
        {
          start = PF.OneStepGD(start, goal, obstacles_v);
          ps.pose.position.x = start.x;
          ps.pose.position.y = start.y;
          robot_poses.push_back(ps);
          terminate = PF.return_terminate();
          ROS_DEBUG("NEW POS: [%.2f, %.2f]", start.x, start.y);
        }

        path.poses = robot_poses;
        path_pub.publish(path);

        
        rate.sleep();
    }

    return 0;
}