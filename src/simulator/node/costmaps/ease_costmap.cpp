#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "math.h"

class EaseCostmap {
    
private:
  ros::NodeHandle n;
  ros::Publisher ease_cm;
  ros::Subscriber map;
  
   double min_ease_dist;
   double max_ease_dist;
   double berth;
   double max_ease_ideal_dist;
   int non_ideal_attenuator;
   double attentiveness;
    
public:
  EaseCostmap() {
    n = ros::NodeHandle("~");
      
    std::string costmap_topic;
    n.getParam("ease_costmap_topic", costmap_topic);
    n.getParam("min_ease_dist", min_ease_dist);
    n.getParam("max_ease_dist", max_ease_dist);
      
    ease_cm = n.advertise<nav_msgs::OccupancyGrid>(costmap_topic, 10);
    map = n.subscribe("/map", 10, &EaseCostmap::map_callback, this);
      
    n.getParam("max_ease_ideal_dist", max_ease_ideal_dist);
    n.getParam("ease_berth", berth);
    n.getParam("ease_non_ideal_attenuator", non_ideal_attenuator);
    n.getParam("ease_cm_attentiveness", attentiveness);
    attentiveness = int(100/attentiveness);
  }
    
  signed char weight(double dist, double ang) {
      int weight = 0;
     
      if (dist <= min_ease_dist) return 0;
      else if (dist >= max_ease_dist) return 0;
      else if (dist <= min_ease_dist + max_ease_ideal_dist) weight = 100;
      else weight = int((1/(dist-max_ease_ideal_dist))*100);
      
      if (std::abs(ang) >= berth) weight = int(weight/non_ideal_attenuator);
      
      return (signed char)weight;
  }
  
  void map_callback(const nav_msgs::OccupancyGrid & cm) {
      nav_msgs::OccupancyGrid costmap;
      costmap.header = cm.header;
      costmap.info = cm.info;
      
      std::vector<signed char, std::allocator<signed char>> weights;
      
      tf::TransformListener listener;
      tf::StampedTransform cam_to_map;
      ros::Time t = ros::Time(0);
      listener.waitForTransform("map", "base_link", t, ros::Duration(30));
      listener.lookupTransform("map", "base_link", t, cam_to_map);
      double th = tf::getYaw(cam_to_map.getRotation());
      double car_x = cam_to_map.getOrigin().getX();// + cm.info.origin.position.x; //- 0.4318*cos(th);
      double car_y = cam_to_map.getOrigin().getY();// + cm.info.origin.position.y; //- 0.4318*sin(th);
      //ROS_INFO_STREAM("th: " << th << " car_x: " << car_x << " car_y: " << car_y);
      
      for (int i = 0; i < int(cm.data.size()); i = i + attentiveness) {
          double cell_x = (cm.info.resolution * ((i+1) % cm.info.width)) + cm.info.origin.position.x;
          double cell_y = (cm.info.resolution * (1 + int(i/cm.info.width))) + cm.info.origin.position.y;
          double y = cell_y - car_y; //cell_x*sin(th) + cell_y*cos(th) + car_y;
          double x = cell_x - car_x; //cell_x*cos(th) - cell_y*sin(th) + car_x;
          double distance_from_car = sqrt(pow(x,2) + pow(y,2));
          double angle_from_car = atan(y / x) - th;
          if (y < 0 && x < 0) angle_from_car -= 3.141593;
          else if (y > 0 && x < 0) angle_from_car += 3.141593;
          signed char val = weight(distance_from_car, angle_from_car);
          weights.push_back(val);
          for (int j = 0; j < attentiveness - 1; j++) weights.push_back(0);
      }
      
      int discrepancy = int(cm.data.size()) - int(weights.size());
      if (discrepancy > 0) {
         for (int i = 0; i < discrepancy; i++) {
            weights.push_back(0);
         }
      }
      else if (discrepancy < 0) {
         for (int i = 0; i < -discrepancy; i++) {
            weights.pop_back();
         }
      }
      
      costmap.data = weights;
      ease_cm.publish(costmap);
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ease_costmapper");
    EaseCostmap easer;
    ros::spin();
    return 0;
}
