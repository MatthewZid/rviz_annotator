#ifndef RVIZ_ANNOTATION_PANEL_H
#define RVIZ_ANNOTATION_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <rviz_annotator/PointSelection.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

# include <rviz/panel.h>

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QComboBox>
#include <rviz_annotator/edit_widget.h>

#include <rviz_annotator/point_class.h>
#include <rviz/yaml_config_reader.h>
#endif

class QLineEdit;

void insertMarker(const std::vector<geometry_msgs::Point32>& points_vec, const std::string& cl_name);
void publishDeleteMarker();
void updateMarkers();

namespace rviz_annotator
{

class AnnotationPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  AnnotationPanel( QWidget* parent = 0 );

  virtual void onInitialize();

   virtual void load( const rviz::Config& config );
   virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  void handleTxtChanged();

  void nameClusterButton();
  void joinButton();
  void divideButton();
  void cancelButton();

  void topicSelect(const QString& txt);

  void refreshAction();

private:
  void createTopicList();
  void createNameList();
  void divideAction();

  //params
  QString sel_topic;
  QString marker_array_topic;
  QString csv_name;
  QString csv_dir;
  QString pc2_topic;

  // Then we finish up with protected member variables.
protected:
  QPushButton *cluster_name_btn;
  QPushButton *cluster_join_btn;
  EditWidget* cluster_name_edit;
  EditWidget* cluster_topic_list;
  QPushButton *refresh_btn;
  QPushButton *divide_btn;
  QPushButton *cancel_btn;

  // The ROS node handle.
  ros::NodeHandle nh;
  ros::Subscriber selection_sub;
  ros::Subscriber viz_sub;

  int button_id;
};

} // end namespace

#endif // TELEOP_PANEL_H