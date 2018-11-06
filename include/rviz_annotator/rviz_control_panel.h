#ifndef RVIZ_CONTROL_PANEL_H
#define RVIZ_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
#define HSPACING 5
#define VSPACING 15
#define BAGPATH "/home/mzidianakis/Ros_WS/bagfiles/"

# include <ros/ros.h>

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

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class RvizCntrlPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
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