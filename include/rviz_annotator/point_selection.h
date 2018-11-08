#ifndef POINT_SELECTION_H
#define POINT_SELECTION_H

//distance defines (DEPRECATED)
/*#define CHECK_DIST_X 0.1
#define CHECK_DIST_NEG_X 0.2
#define CHECK_DIST_Y 0.1
#define CHECK_DIST_NEG_Y 0.2
#define CHECK_DIST_Z 0.5
#define CHECK_DIST_NEG_Z 1*/
#ifndef Q_MOC_RUN
#include <rviz/tool.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderOperation.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include <rviz/properties/vector_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>

#include <pointcloud_msgs/PointCloud2_Segments.h>

#include <cstdlib>
#include <rviz/yaml_config_reader.h>
#endif

namespace Ogre
{
class SceneNode;
class Vector3;
class ColourValue;
}

namespace rviz
{
class VectorProperty;
class ColorProperty;
class FloatProperty;
class VisualizationManager;
class ViewportMouseEvent;
class SelectionManager;
class PropertyTreeModel;
class Property;
}

namespace rviz_annotator
{

class PointSelector: public rviz::Tool
{
Q_OBJECT
public:
  PointSelector();
  ~PointSelector();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

private:
  bool selecting_;
  int sel_start_x_;
  int sel_start_y_;

  //params
  QString sel_topic;
  QString pc_segments_topic;
};

} // end namespace

#endif