#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <f1tenth_simulator/PixyData.h>
#include <f1tenth_simulator/PixyResolution.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "math.h"

struct Coor {
    double x;
    double y;
};

class Search {
    
private:
  ros::NodeHandle n;
  ros::Publisher nav_goal;
  ros::Publisher sum_cm;
  ros::Subscriber time_cm;
  ros::Subscriber ease_cm;
  ros::Subscriber object_cm;
  ros::Subscriber unsearched_area_cm;
  ros::Subscriber frontier_cm;
  ros::Subscriber bucket;
 
  nav_msgs::OccupancyGrid sum_costmap;
  nav_msgs::OccupancyGrid time_costmap;
  nav_msgs::OccupancyGrid ease_costmap;
  nav_msgs::OccupancyGrid object_costmap;
  nav_msgs::OccupancyGrid unsearched_area_costmap;
  nav_msgs::OccupancyGrid frontier_costmap;
    
  ros::Timer timer;
  double pub_time;
  
  
  double time_cm_weight;
  double ease_cm_weight;
  double object_cm_weight;
  double unsearched_area_cm_weight;
  double frontier_cm_weight;
  
  bool environment_mapped;
  bool environment_searched;
  bool frontier_cm_recieved;
  bool ua_cm_recieved;
  bool ease_cm_recieved;
  bool time_cm_recieved;
  bool obstacle_cm_recieved;
    
  bool found;
  double a;
  double b;
  double c;
  double stopping_distance;
    
  double map_origin_y;
  double map_origin_x;
    
public:
  Search() {
    n = ros::NodeHandle("~");
    
    std::string sum_costmap, time_costmap, ease_costmap, object_costmap, unsearched_area_costmap, frontier_costmap;
    n.getParam("sum_costmap_topic", sum_costmap);
    n.getParam("time_costmap_topic", time_costmap);
    n.getParam("ease_costmap_topic", ease_costmap);
    n.getParam("object_costmap_topic", object_costmap);
    n.getParam("unsearched_area_costmap_topic", unsearched_area_costmap);
    n.getParam("frontier_costmap_topic", frontier_costmap);
      
    nav_goal = n.advertise<geometry_msgs::PoseStamped>("/search_goal", 10);
    sum_cm = n.advertise<nav_msgs::OccupancyGrid>(sum_costmap, 10);
    unsearched_area_cm = n.subscribe(unsearched_area_costmap, 10, &Search::unsearched_area_cm_callback, this);
    time_cm = n.subscribe(time_costmap, 10, &Search::time_cm_callback, this);
    ease_cm = n.subscribe(ease_costmap, 10, &Search::ease_cm_callback, this);
    object_cm = n.subscribe(object_costmap, 10, &Search::object_cm_callback, this);
    frontier_cm = n.subscribe(frontier_costmap, 10, &Search::frontier_cm_callback, this);
    bucket = n.subscribe("/bucket_visual", 10, &Search::bucket_callback, this);
    
    n.getParam("search_goal_update_time", pub_time);
    timer = n.createTimer(ros::Duration(pub_time), &Search::timer_callback, this);
    
    n.getParam("time_cm_weight", time_cm_weight);
    n.getParam("ease_cm_weight", ease_cm_weight);
    n.getParam("object_cm_weight", object_cm_weight);
    n.getParam("unsearched_area_cm_weight", unsearched_area_cm_weight);
    n.getParam("frontier_cm_weight", frontier_cm_weight);
      
    n.getParam("pixy_dist_coef", a);
    n.getParam("pixy_dist_exp", b);
    n.getParam("pixy_angle_coef", c);
    n.getParam("stopping_distance", stopping_distance);
     
    environment_mapped = false;  
    environment_searched = false;
      
    frontier_cm_recieved = false;
    ua_cm_recieved = false;
    ease_cm_recieved = false;
    obstacle_cm_recieved = false;
    time_cm_recieved = false;
      
    found = false;
      
    map_origin_x = 0.0;
    map_origin_y = 0.0;
  }
  
  void tranform_and_publish(int height, int width, int x_offset, int y_offset) {
      double block_area = height*width;
      double pixy_distance = a*pow(block_area, b);
      double pixy_angle = (-x_offset+159)*(c*M_PI/180);
      
      //double pixy_x = (pixy_distance*cos(pixy_angle))/100;
      //double pixy_y = (pixy_distance*sin(pixy_angle))/100;
      //ROS_INFO_STREAM("dist: " << pixy_distance << ". ang: " << pixy_angle);
      //ROS_INFO_STREAM("pixy_x: " << pixy_x << ". pixy_y: " << pixy_y);
      tf::TransformListener listener;
      tf::StampedTransform pixy_to_map;
      ros::Time t = ros::Time(0);
      listener.waitForTransform("map", "base_footprint", t, ros::Duration(30));
      listener.lookupTransform("map", "base_footprint", t, pixy_to_map);
      double th = (tf::getYaw(pixy_to_map.getRotation()));
      double car_x = pixy_to_map.getOrigin().getX(); // - 2*0.4318*cos(th);
      double car_y = pixy_to_map.getOrigin().getY(); // - 2*0.4318*sin(th);
      //ROS_INFO_STREAM("car_x: " << car_x << ". car_y: " << car_y);
      
      //ROS_INFO_STREAM("car_x: " << car_x << ". car_x: " << car_y);
      
      double cell_x = car_x + ((pixy_distance+43-stopping_distance)/100)*(cos(th + pixy_angle));// - map_origin_x;
      double cell_y = car_y + ((pixy_distance+43-stopping_distance)/100)*(sin(th + pixy_angle));// - map_origin_y;
      //target_cell = int( (int(cell_y/map_resolution)-1)*map_width + int(cell_x/map_resolution) - 1);
      //ROS_INFO_STREAM("origin_x: " << map_origin_x << ". origin_y: " << map_origin_y);
      
      geometry_msgs::PoseStamped goal;
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = "map";
      goal.pose.position.x = cell_x;
      goal.pose.position.y = cell_y;
      goal.pose.orientation.w = 1.0;
      nav_goal.publish(goal);
  }
    
  void bucket_callback(const f1tenth_simulator::PixyData & msg) {
      if (!msg.blocks.empty() && int(msg.blocks[0].roi.height*msg.blocks[0].roi.width) != 0) {
          found = true;
          tranform_and_publish((int)msg.blocks[0].roi.height, (int)msg.blocks[0].roi.width, (int)msg.blocks[0].roi.x_offset, (int)msg.blocks[0].roi.y_offset);
      }
      //else found = false;
  }
  
  void timer_callback(const ros::TimerEvent&) {
      if (ease_cm_recieved && !found) {
        geometry_msgs::PoseStamped goal;
        sum_costmap.header = ease_costmap.header;
        sum_costmap.info = ease_costmap.info;

        apply_weights();
        Coor goal_point = find_best_goal();
          
        goal.header = sum_costmap.header;
        goal.pose.position.x = goal_point.x;
        goal.pose.position.y = goal_point.y;
        goal.pose.orientation.w = 1.0;

        nav_goal.publish(goal);
        
        ua_cm_recieved = false;
        ease_cm_recieved = false;
        obstacle_cm_recieved = false;
        time_cm_recieved = false;
        frontier_cm_recieved = false;
      }
  }
  
  void apply_weights() {
    std::vector<double> weights;
    double highest_weight = 0.0;

    for (int i = 0; i < int(frontier_costmap.data.size()); i++) {
        double weight = frontier_cm_weight*double(frontier_costmap.data[i]);
        if (ua_cm_recieved) weight += unsearched_area_cm_weight*double(unsearched_area_costmap.data[i]); 
        if (environment_mapped && environment_searched && time_cm_recieved) weight += time_cm_weight*double(time_costmap.data[i]);
        if (ease_cm_recieved) weight += ease_cm_weight*double(ease_costmap.data[i]);
        if (obstacle_cm_recieved) weight *= double(object_costmap.data[i]); // obstacle costmap
            
        if (weight > highest_weight) highest_weight = weight;
        weights.push_back(weight);
    }
  
    // normalize
    std::vector<signed char, std::allocator<signed char>> normalized_weights;
    for (int i = 0; i < int(weights.size()); i++) {
        normalized_weights.push_back( (signed char)(100 * ( weights[i] / highest_weight )) );
    }
    sum_costmap.data = normalized_weights;
    sum_cm.publish(sum_costmap);
  }
    
  Coor find_best_goal() {
      std::vector<int> best_cells;
      int highest_weight = 0;
      for (int i = 0; i < int(sum_costmap.data.size()); i++) {
          int cell_weight = int(sum_costmap.data[i]);
          if (cell_weight > highest_weight) {
              best_cells.clear();
              best_cells.push_back(i);
              highest_weight = cell_weight;
          }
          else if (cell_weight == highest_weight) best_cells.push_back(i);
      }
      
      int rand_cell = rand() % int(best_cells.size());
      int best_cell = best_cells[rand_cell];
      
      std::vector<double> coor = cell_to_world(best_cell);
      Coor best_goal = {coor[0], coor[1]};
      
      return best_goal;
  }
              
  std::vector<double> cell_to_world(int idx) {
      int width = sum_costmap.info.width;
      int y = int(idx/width);
      int x = idx % width;
      std::vector<double> coor = {x * sum_costmap.info.resolution + sum_costmap.info.origin.position.x,
                                  y * sum_costmap.info.resolution + sum_costmap.info.origin.position.y};
      return coor;
  }
    
  bool is_environment_mapped(nav_msgs::OccupancyGrid & cm) {
    for (int i = 0; i < int(cm.data.size()); i++) {
        if (cm.data[i] != 0) return false;
    }
    return true;
  }
    
  bool is_environment_searched(nav_msgs::OccupancyGrid & cm) {
    for (int i = 0; i < int(cm.data.size()); i++) {
        if (cm.data[i] == 100) return false;
    }
    return true;
  }
              
  void unsearched_area_cm_callback(const nav_msgs::OccupancyGrid & cm) {
    unsearched_area_costmap = cm;
    ua_cm_recieved = true;
    environment_searched = is_environment_searched(unsearched_area_costmap);
  }
  void time_cm_callback(const nav_msgs::OccupancyGrid & cm) {
    time_costmap = cm;
    time_cm_recieved = true;
  }
  void ease_cm_callback(const nav_msgs::OccupancyGrid & cm) {
    ease_costmap = cm;
    ease_cm_recieved = true;
  }
  void object_cm_callback(const nav_msgs::OccupancyGrid & cm) {
    object_costmap = cm;
    obstacle_cm_recieved = true;
  }
  void frontier_cm_callback(const nav_msgs::OccupancyGrid & cm) {
    map_origin_x = cm.info.origin.position.x;
    map_origin_y = cm.info.origin.position.y;
    frontier_costmap = cm;
    frontier_cm_recieved = true;
    environment_mapped = is_environment_mapped(frontier_costmap);
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "searcher");
    Search bucketfinder;
    ros::spin();
    return 0;
}
