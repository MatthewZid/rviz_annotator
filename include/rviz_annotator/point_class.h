#ifndef POINT_CLASS_H
#define POINT_CLASS_H

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <regex>
#include <fstream>

namespace rviz_annotator
{

typedef struct pointClass
{
  std::string name;
  ros::Time stamp;
  ros::Time segment_stamp;
  std::string topic;
  std::string type;
  std::vector<geometry_msgs::Point32> points;
  std::vector<long> cluster_pos;
  std::vector<long> point_pos;
}PointClass;

std::vector<PointClass> readcsv();
size_t binarySearch(const std::vector<PointClass>& pcvec, size_t l, size_t r, double x);
size_t cluster_search(const std::vector<PointClass>& pcvec, const ros::Time x);

}

#endif