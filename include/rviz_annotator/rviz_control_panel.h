#ifndef RVIZ_CONTROL_PANEL_H
#define RVIZ_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
#define HSPACING 5
#define VSPACING 15
#define BAGPATH "/home/mzidianakis/Ros_WS/bagfiles/"

# include <ros/ros.h>
#include <ros/package.h>

# include <rviz/panel.h>

#include <stdio.h>

#include <QPainter>
#include <QString>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <QComboBox>
#include <QPushButton>
#include <QGroupBox>
#include <QApplication>
#include <QCheckBox>
#include <QFileDialog>

#include <mutex>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <rviz_annotator/rviz_rosbag_player.h>

#include <boost/filesystem.hpp>
#include <rviz/yaml_config_reader.h>
#endif

namespace fs = boost::filesystem;

namespace rviz_rosbag
{
class Player;
class PlayerOptions;
}

class QLineEdit;
class QPushButton;
class QApplication;

void runPlayer();

namespace rviz_annotator
{

class RvizCntrlPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  RvizCntrlPanel( QWidget* parent = 0 );
  ~RvizCntrlPanel();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  //virtual void load( const rviz::Config& config );
  //virtual void save( rviz::Config config ) const;

private Q_SLOTS:
  void bagSelect(const int index);

  void startButton();
  void pauseButton();
  void stepButton();
  void backstepButton();
  void folderButton();
  void terminateButton();

  void enableStartBtn();

  void loopCheckbox();
  void quietCheckbox();
  void syncCheckbox();
  void clockCheckbox();

private:
  int findBagfiles();

  QString bagfiles_dir;

  // Then we finish up with protected member variables.
protected:
  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* current_bagpath;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  std::vector<std::string> bag_files_;
  int bagpath_file_count;
  QString bag_path;

  rviz_rosbag::PlayerOptions* options;
  bool paused_;
};

} // end namespace

#endif