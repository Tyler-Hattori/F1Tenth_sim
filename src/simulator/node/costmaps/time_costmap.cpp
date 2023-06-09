#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <f1tenth_simulator/seenPoints.h>

#include "math.h"

struct Memory {
    int idx;
    double stamp;
};

class TimeCostmap {
    
private:
  ros::NodeHandle n;
  ros::Publisher time_cm_pub;
  ros::Subscriber view;
  ros::Subscriber global_cm;
 
  nav_msgs::OccupancyGrid time_cm;
  std::vector<Memory> history;
  
    double time_threshold;
    double time_priority_slope;
    int costmap_size;
   
public:
  TimeCostmap() {
    n = ros::NodeHandle("~");
      
    std::string costmap_topic, seen_points_topic;
    n.getParam("time_costmap_topic", costmap_topic);
    n.getParam("seen_points_topic", seen_points_topic);
      
    n.getParam("time_threshold", time_threshold);
    n.getParam("time_priority_slope", time_priority_slope);
      
    time_cm_pub = n.advertise<nav_msgs::OccupancyGrid>(costmap_topic, 10);
    view = n.subscribe(seen_points_topic, 10, &TimeCostmap::view_callback, this);
    global_cm = n.subscribe("/map", 10, &TimeCostmap::map_callback, this);
    costmap_size = 0;
  }
  
  signed char calculate_priority(double stamp, double now) {
      if (now - stamp <= time_threshold) return 0;
      else return (signed char)std::min(int((now - stamp - time_threshold) * time_priority_slope), 100);
  }
    
  /*std::vector<double> cell_to_world(int index) {
      int width = time_cm.info.width;
      int x = int(index/width);
      int y = index % width;
      std::vector<double> coor = {x * time_cm.info.resolution + time_cm.info.origin.position.x,
                                  y * time_cm.info.resolution + time_cm.info.origin.position.y};
      return coor;
  }
  
  int world_to_cell(double x, double y) {
      int width = time_cm.info.width;
      int u  = int((x - time_cm.info.origin.position.x) / time_cm.info.resolution);
      int v  = int((y - time_cm.info.origin.position.y) / time_cm.info.resolution);
      return u*width + v; 
  }*/
   
  void map_callback(const nav_msgs::OccupancyGrid & cm) {
      time_cm.header = cm.header;
      time_cm.info = cm.info;
      costmap_size = int(cm.data.size());
  }
    
  void view_callback(const f1tenth_simulator::seenPoints & seen_points) {
    if (costmap_size > int(seen_points.confidences.size())) {
      double now = ros::Time::now().toSec();
      
      std::vector<signed char, std::allocator<signed char>> blanks;
      for (int i = 0; i < costmap_size; i++) {
          blanks.push_back((signed char)100);
      }
      
      for (int i = 0; i < int(seen_points.confidences.size()); i++) {
        int idx = int(seen_points.indices[i]);
        blanks[idx] = 0;
        //std::vector<double> coor = cell_to_world(int(seen_points.indices[i]));
        Memory new_memory = {idx, now}; //{coor[0], coor[1], now};
        history.push_back(new_memory);
      }
      
      for (int i = int(history.size()) - 1; i >= 0; i--) {
        if (history[i].stamp != now) {
            int cell = history[i].idx;//world_to_cell(history[i].x, history[i].y);
            if (blanks[cell] == 0) history.erase(history.begin() + i);
            else blanks[cell] = calculate_priority(history[i].stamp, now);
        }
      }
        
      time_cm.data = blanks;
      time_cm_pub.publish(time_cm);
    }
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "remember_old_locations");
    TimeCostmap timer;
    ros::spin();
    return 0;
}
