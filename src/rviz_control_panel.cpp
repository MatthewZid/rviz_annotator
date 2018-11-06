#include <rviz_annotator/rviz_control_panel.h>

QComboBox* bagmenu;

QPushButton* start_button;
QPushButton* pause_button;
QPushButton* step_button;
QPushButton* backstep_button;
QPushButton* filediag_button;

QPushButton* cancel_loop_button;

QCheckBox* loop_checkbox;
QCheckBox* quiet_checkbox;
QCheckBox* sync_topics_checkbox;
QCheckBox* clock_checkbox;

rviz_rosbag::Player* rosbag_player;

void runPlayer()
{
  try{
      rosbag_player->publish();
      Q_EMIT start_button->pressed();
      QApplication::processEvents();
  }
  catch(std::runtime_error& e){
      ROS_FATAL("%s\n", e.what());
      ros::shutdown();
  }
}

namespace rviz_annotator
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
RvizCntrlPanel::RvizCntrlPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  //default bagfile directory
  std::string homepath = std::getenv("HOME");
  bag_path = QString::fromStdString(homepath) + "/Ros_WS/bagfiles";

  bagpath_file_count = findBagfiles();

  //initialize rosbag player
  options = new rviz_rosbag::PlayerOptions;
  std::string bagfile = bag_path.toStdString() + "/" + bag_files_[0];
  (options->bags).push_back(bagfile);

  //default player options
  options->queue_size = 100;
  options->bag_time_frequency = 100.0f;

  rosbag_player = new rviz_rosbag::Player(*options);
  paused_ = false;

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QGroupBox* rosbag_group = new QGroupBox("Rosbag player");

  QHBoxLayout* filediag_layout = new QHBoxLayout;
  QLabel* path_lbl = new QLabel("Bag path:");
  filediag_button = new QPushButton("...");
  current_bagpath = new QLineEdit;
  current_bagpath->setText(bag_path);
  current_bagpath->setReadOnly(true);
  filediag_layout->addWidget(path_lbl);
  filediag_layout->addWidget(current_bagpath);
  filediag_layout->addWidget(filediag_button);

  QHBoxLayout* bag_layout = new QHBoxLayout;
  QLabel* label = new QLabel;
  bagmenu = new QComboBox;
  label->setText("Bag file:");
  label->setAlignment(Qt::AlignLeft);

  for(int i = 0; i < bagpath_file_count; i++)
    bagmenu->insertItem(i, QString(bag_files_[i].c_str()));

  bagmenu->setEditable(false);
  bag_layout->addWidget(label);
  bag_layout->addWidget(bagmenu);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->setSpacing(0);
  start_button = new QPushButton("Start");
  pause_button = new QPushButton("Pause");
  pause_button->setEnabled(false);
  buttons_layout->addWidget(start_button);
  buttons_layout->addWidget(pause_button);

  QHBoxLayout* player_layout = new QHBoxLayout;
  player_layout->setSpacing(0);
  step_button = new QPushButton(">>");
  backstep_button = new QPushButton("<<");
  step_button->setAutoRepeat(true); //repeat action while holding down
  backstep_button->setAutoRepeat(true); //repeat action while holding down
  step_button->setEnabled(false);
  backstep_button->setEnabled(false);
  player_layout->addWidget(backstep_button);
  player_layout->addWidget(step_button);

  QHBoxLayout* cancel_layout = new QHBoxLayout;
  cancel_loop_button = new QPushButton("Terminate");
  cancel_loop_button->setEnabled(false);
  cancel_layout->addWidget(cancel_loop_button);

  QHBoxLayout* check_boxes = new QHBoxLayout;
  loop_checkbox = new QCheckBox("Loop");
  quiet_checkbox = new QCheckBox("Quiet");
  sync_topics_checkbox = new QCheckBox("Sync Topics");
  clock_checkbox = new QCheckBox("Clock");
  check_boxes->addWidget(loop_checkbox);
  check_boxes->addWidget(quiet_checkbox);
  check_boxes->addWidget(sync_topics_checkbox);
  check_boxes->addWidget(clock_checkbox);

  // Then create the control widget.
  //q_widget_ = new RvizQWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* rosbag_layout = new QVBoxLayout;
  rosbag_layout->setSpacing(VSPACING);
  rosbag_layout->addLayout(filediag_layout);
  rosbag_layout->addLayout(bag_layout);
  rosbag_layout->addLayout( buttons_layout );
  rosbag_layout->addLayout(player_layout);
  rosbag_layout->addLayout(cancel_layout);
  rosbag_layout->addLayout(check_boxes);

  rosbag_group->setLayout(rosbag_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->setSpacing(VSPACING);
  layout->addWidget(rosbag_group);
  //rosbag_layout->addWidget( q_widget_ );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this RvizCntrlPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  //QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect(bagmenu, QOverload<int>::of(&QComboBox::activated), [=](int index){bagSelect(index);});
  qRegisterMetaType< QVector<int> >("QVector<int>");  //fixes QVector<int> error in conncet
  connect(bagmenu, SIGNAL(activated(int)), this, SLOT(bagSelect(const int)));
  connect(filediag_button, SIGNAL(released()), this, SLOT(folderButton()));

  connect(start_button, SIGNAL(released()), this, SLOT(startButton()));
  connect(start_button, SIGNAL(pressed()), this, SLOT(enableStartBtn()));
  connect(pause_button, SIGNAL(released()), this, SLOT(pauseButton()));
  connect(step_button, SIGNAL(pressed()), this, SLOT(stepButton()));
  connect(backstep_button, SIGNAL(pressed()), this, SLOT(backstepButton()));

  connect(cancel_loop_button, SIGNAL(released()), this, SLOT(terminateButton()));

  connect(loop_checkbox, SIGNAL(stateChanged(int)), this, SLOT(loopCheckbox()));
  connect(quiet_checkbox, SIGNAL(stateChanged(int)), this, SLOT(quietCheckbox()));
  connect(sync_topics_checkbox, SIGNAL(stateChanged(int)), this, SLOT(syncCheckbox()));
  connect(clock_checkbox, SIGNAL(stateChanged(int)), this, SLOT(clockCheckbox()));
  //QObject::connect( q_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( rosbag_player_input_, SIGNAL( editingFinished() ), this, SLOT( updateChoice() ));
  //QObject::connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  //output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  //q_widget_->setEnabled( false );
}

RvizCntrlPanel::~RvizCntrlPanel()
{
  delete options;
  delete rosbag_player;
}

void RvizCntrlPanel::bagSelect(const int index)
{
  std::string bagfile;
  options->bags.clear();

  bagfile = bag_path.toStdString() + "/" + bag_files_[index];

  options->bags.push_back(bagfile);
  rosbag_player->changeOptions(*options);
}

void RvizCntrlPanel::enableStartBtn()
{
  start_button->setEnabled(true);

  loop_checkbox->setEnabled(true);
  quiet_checkbox->setEnabled(true);
  clock_checkbox->setEnabled(true);
  bagmenu->setEnabled(true);
  step_button->setEnabled(false);
  backstep_button->setEnabled(false);
  cancel_loop_button->setEnabled(false);
  pause_button->setText("Pause");
  pause_button->setEnabled(false);

  current_bagpath->setEnabled(true);
  filediag_button->setEnabled(true);
}

void RvizCntrlPanel::startButton()
{
  start_button->setEnabled(false);
  bagmenu->setEnabled(false);
  pause_button->setEnabled(true);
  step_button->setEnabled(true);
  backstep_button->setEnabled(true);
  cancel_loop_button->setEnabled(true);

  loop_checkbox->setEnabled(false);
  quiet_checkbox->setEnabled(false);
  clock_checkbox->setEnabled(false);

  current_bagpath->setEnabled(false);
  filediag_button->setEnabled(false);

  if(loop_checkbox->isChecked()){
    options->loop = true;
    rosbag_player->changeOptions(*options);
  }

  QApplication::processEvents();

  std::thread buttonStart(runPlayer);
  buttonStart.detach();
}

void RvizCntrlPanel::pauseButton()
{
  //Stop button actions
  (rosbag_player->lock_choice_).lock();
  rosbag_player->setChoice(' ');
  (rosbag_player->lock_choice_).unlock();

  paused_ = !paused_;

  if(paused_)
    pause_button->setText("Resume");
  else
    pause_button->setText("Pause");
}

void RvizCntrlPanel::stepButton()
{
  (rosbag_player->lock_choice_).lock();
  rosbag_player->setChoice('s');
  (rosbag_player->lock_choice_).unlock();
}

void RvizCntrlPanel::backstepButton()
{
  (rosbag_player->lock_choice_).lock();
  rosbag_player->setChoice('b');
  (rosbag_player->lock_choice_).unlock();
}

void RvizCntrlPanel::folderButton()
{
  std::string homepath = std::getenv("HOME");
  QString qHomepath = QString::fromStdString(homepath);
  bag_path = QFileDialog::getExistingDirectory(this, "Open bagfile directory", qHomepath, QFileDialog::ShowDirsOnly
                                                                                              | QFileDialog::DontResolveSymlinks
                                                                                              | QFileDialog::DontUseNativeDialog);
  if(bag_path == "")
    bag_path = current_bagpath->text();

  if(bag_path != current_bagpath->text())
  {
    bag_files_.clear();
    bagmenu->clear();

    bagpath_file_count = findBagfiles();

    for(int i = 0; i < bagpath_file_count; i++)
      bagmenu->insertItem(i, QString(bag_files_[i].c_str()));

    current_bagpath->setText(bag_path);

    options->bags.clear();
    std::string bagfile = bag_path.toStdString() + "/" + bag_files_[0];

    options->bags.push_back(bagfile);
    rosbag_player->changeOptions(*options);
  }
}

void RvizCntrlPanel::terminateButton()
{
  pause_button->setEnabled(false);
  step_button->setEnabled(false);
  backstep_button->setEnabled(false);
  cancel_loop_button->setEnabled(false);
  
  options->loop = false;
  rosbag_player->changeOptions(*options);
  rosbag_player->setTerminate(true);
}

void RvizCntrlPanel::loopCheckbox()
{
  if(loop_checkbox->checkState() == Qt::Checked)
    options->loop = true;
  else
    options->loop = false;

  rosbag_player->changeOptions(*options);
}

void RvizCntrlPanel::quietCheckbox()
{
  if(quiet_checkbox->checkState() == Qt::Checked)
    options->quiet = true;
  else
    options->quiet = false;

  rosbag_player->changeOptions(*options);
}

void RvizCntrlPanel::syncCheckbox()
{
  if(sync_topics_checkbox->checkState() == Qt::Checked)
    options->sync_topics = true;
  else
    options->sync_topics = false;

  rosbag_player->changeOptions(*options);
}

void RvizCntrlPanel::clockCheckbox()
{
  if(clock_checkbox->checkState() == Qt::Checked)
    options->bag_time = true;
  else
    options->bag_time = false;

  rosbag_player->changeOptions(*options);
}

int RvizCntrlPanel::findBagfiles()
{
  //locate bagfiles from directory
  fs::path bp(bag_path.toStdString());
  int dcount = 0;

  try
  {
    ROS_ASSERT(fs::exists(bp));
    ROS_ASSERT(fs::is_directory(bp));

    for(fs::directory_iterator dit(bp); dit != fs::directory_iterator(); dit++)
      if(is_regular_file(*dit) and dit->path().has_extension())
        if(dit->path().extension().string() == ".bag"){
          bag_files_.push_back(dit->path().filename().string());
          dcount++;
        }
  }
  catch(const fs::filesystem_error& ex){
    std::cout << ex.what() << std::endl;
  }

  if(dcount == 0)
  {
    ROS_FATAL("No bag files found!\n");
    ros::shutdown();
  }

  ROS_INFO("\nFound %d bag files\n", dcount);

  return dcount;
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
/*void RvizCntrlPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void RvizCntrlPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //rosbag_player_input_->setText( topic );
    //updateChoice();
  }
}*/

} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_annotator::RvizCntrlPanel, rviz::Panel )