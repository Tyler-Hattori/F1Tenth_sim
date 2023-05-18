#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <f1tenth_simulator/seenPoints.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "math.h"

struct Memory {
    int idx;
    int weight;
};

class UnsearchedAreaCostmap {
    
private:
  ros::NodeHandle n;
  ros::Publisher ua_cm_pub;
  ros::Publisher ua_temp_cm_pub;
  ros::Subscriber map;
  ros::Subscriber view;
 
  nav_msgs::OccupancyGrid ua_cm;
  nav_msgs::OccupancyGrid binary_cm;
  nav_msgs::OccupancyGrid temp_cm;
  std::vector<Memory> history;
  
  double kernel_size;
  double attentiveness;
   
public:
  UnsearchedAreaCostmap() {
    n = ros::NodeHandle("~");
      
    std::string costmap_topic, global_costmap_topic, seen_points_topic;
    n.getParam("unsearched_area_costmap_topic", costmap_topic);
    //n.getParam("global_costmap_topic", global_costmap_topic);
    n.getParam("seen_points_topic", seen_points_topic);
      
    ua_cm_pub = n.advertise<nav_msgs::OccupancyGrid>(costmap_topic, 10);
    ua_temp_cm_pub = n.advertise<nav_msgs::OccupancyGrid>("ua_temp_cm", 10);
    map = n.subscribe("/map", 10, &UnsearchedAreaCostmap::map_callback, this);
    view = n.subscribe(seen_points_topic, 10, &UnsearchedAreaCostmap::view_callback, this);
      
    n.getParam("ua_kernel_size", kernel_size);
    n.getParam("ua_cm_attentiveness", attentiveness);
    attentiveness = int(100/attentiveness);
      
    std::vector<signed char, std::allocator<signed char>> weights;
    weights.reserve(1);
    binary_cm.data = weights;
  }
  
  signed char calculate_priority(int idx, nav_msgs::OccupancyGrid temp_cm) {
      int weight = 0;
      int width = temp_cm.info.width;
      int height = temp_cm.info.height;
      int kernel_size_in_cells = int(kernel_size/temp_cm.info.resolution);
      int max_temp_weight = kernel_size_in_cells * kernel_size_in_cells * 100;
      
      for (int row = int(idx/width) - int(kernel_size_in_cells/2); row <= int(idx/width) + int(std::ceil(kernel_size_in_cells/2)); row++) {
          for (int col = int(idx % width) - int(kernel_size_in_cells/2); col <= int(idx % width) + int(std::ceil(kernel_size_in_cells/2)); col++) {
              if (row >= 0 && row < height && col >= 0 && col < width) weight += (int)temp_cm.data[row*width + col]; 
          }
      }
      
      if (weight < 0) weight = 0;
      
      return (signed char)int(100*weight/max_temp_weight);
  }
    
  /*std::vector<double> cell_to_world(int index) {
      int width = ua_cm.info.width;
      int x = int(index/width);
      int y = index % width;
      std::vector<double> coor = {x * ua_cm.info.resolution + ua_cm.info.origin.position.x,
                                  y * ua_cm.info.resolution + ua_cm.info.origin.position.y};
      return coor;
  }
  
  int world_to_cell(double x, double y) {
      int width = ua_cm.info.width;
      int u  = int((x - ua_cm.info.origin.position.x) / ua_cm.info.resolution);
      int v  = int((y - ua_cm.info.origin.position.y) / ua_cm.info.resolution);
      return u*width + v; 
  }*/
    
  int history_index_of(int cell) {
      for (int i = 0; i < int(history.size()); i++) {
          if (history[i].idx == cell) return i;
          // if (world_to_cell(history[i].x, history[i].y) == cell) return i;
      }
      return -1;
  }
   
  void map_callback(const nav_msgs::OccupancyGrid & cm) {
      binary_cm.header = cm.header;
      binary_cm.info = cm.info;
      ua_cm.header = cm.header;
      ua_cm.info = cm.info;
      temp_cm.header = cm.header;
      temp_cm.info = cm.info;
      
      std::vector<signed char, std::allocator<signed char>> weights;
      
      for (int i = 0; i < int(cm.data.size()); i++) {
          if (cm.data[i] == 0) weights.push_back(100);
          else weights.push_back(-100);
      }
      
      binary_cm.data = weights;
  }
    
  void view_callback(const pathing::seenPoints & seen_points) {
    if (int(binary_cm.data.size()) > 0) {
      std::vector<signed char, std::allocator<signed char>> weights;
      weights = binary_cm.data;
      
      for (int i = 0; i < int(seen_points.confidences.size()); i++) {
        int weight = 100 - int(seen_points.confidences[i]);
        //std::vector<double> coor = cell_to_world(int(seen_points.indices[i]));
        int cell = int(seen_points.indices[i]);
        Memory new_memory = {cell, weight}; //{coor[0], coor[1], weight};
        history.push_back(new_memory);
      }
      
      ROS_INFO_STREAM("number of seen points: " << int(history.size()));
      for (int i = int(history.size()) - 1; i >= 0; i--) {
        int cell = history[i].idx; //world_to_cell(history[i].x, history[i].y);
        if (weights[cell] == 100 || weights[cell] == -100) {
            weights[cell] = history[i].weight;
        }
        else {
            if (weights[cell] <= history[i].weight) history.erase(history.begin() + i);
            else {
                weights[cell] = history[i].weight;
                history.erase(history.begin() + history_index_of(cell));
            }
        }
      }
      
      temp_cm.data = weights;
      ua_temp_cm_pub.publish(temp_cm);
      
      std::vector<signed char, std::allocator<signed char>> priorities;
      
      for (int i = 0; i < int(temp_cm.data.size()); i = i + attentiveness) {
          priorities.push_back(calculate_priority(i,temp_cm));
          for (int j = 0; j < attentiveness - 1; j++) { priorities.push_back(0); }
      }
      
      int discrepancy = int(temp_cm.data.size()) - int(priorities.size());
      if (discrepancy > 0) {
         for (int i = 0; i < discrepancy; i++) {
            priorities.push_back(0);
         }
      }
      else if (discrepancy < 0) {
         for (int i = 0; i < -discrepancy; i++) {
            priorities.pop_back();
         }
      }
      
      ua_cm.data = priorities;
      ua_cm_pub.publish(ua_cm);
    }
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "unsearched_area_costmapper");
    UnsearchedAreaCostmap area;
    ros::spin();
    return 0;
}
