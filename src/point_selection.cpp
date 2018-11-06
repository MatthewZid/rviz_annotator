#include <rviz_annotator/point_selection.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point32.h>
#include <rviz_annotator/PointSelection.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

ros::NodeHandle nh;
ros::Subscriber pointsub;
ros::Publisher selection_pub;
pointcloud_msgs::PointCloud2_Segments cluster_msg;

void pointsCallback(const pointcloud_msgs::PointCloud2_Segments& msg)
{
    ROS_WARN("Point selector: Message received\n");

    cluster_msg = msg;
}

namespace rviz_annotator
{

PointSelector::PointSelector()
  : /*moving_flag_node_( NULL )
  , current_flag_property_( NULL )
  ,*/ selecting_(false)
  , sel_start_x_(0)
  , sel_start_y_(0)
{
  shortcut_key_ = 's';
}

PointSelector::~PointSelector()
{
  /*for( unsigned i = 0; i < flag_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( flag_nodes_[ i ]);
  }*/

  /*for( unsigned i = 0; i < point_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( point_nodes_[ i ]);
  }*/
}

void PointSelector::onInitialize()
{
  /*flag_resource_ = "package://rviz_annotator/media/flag.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "PointSelector: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  moving_flag_node_->attachObject( entity );
  moving_flag_node_->setVisible( false );*/

  //create manual object (NOT NEEDED, only for test purposes)
  /*Ogre::Vector3 point_pos[2];
  point_pos[0].x=-1.0f;
  point_pos[0].y=0.0f;
  point_pos[0].z=0.5f;

  point_pos[1].x=1.0f;
  point_pos[1].y=0.0f;
  point_pos[1].z=0.5f;

  for(int i=0; i<2; i++){
  	Ogre::ManualObject* manual = scene_manager_->createManualObject("manual_"+std::to_string(i));
	manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
 
	manual->position(0.0, 0.0, 0.0);
 
	manual->end();
	Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode("manual"+std::to_string(i));

	node->attachObject(manual);
	node->setVisible(true);
	node->setPosition(point_pos[i]);

	point_nodes_.push_back(node);
	manual_objects_.push_back(manual);
  }*/
    std::string input_topic = "pointcloud2_cluster_tracking/clusters";

    pointsub = nh.subscribe(input_topic, 1, pointsCallback);
    selection_pub = nh.advertise<rviz_annotator::PointSelection>("rviz_annotator/selection_topic", 1);
}

void PointSelector::activate()
{
  /*if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( true );

    current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
    current_flag_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_flag_property_ );
  }*/
  

  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;

  //start rosbag player in parallel
  //boost::thread player_thread(runPlayer);
  //player_thread.join();
}

void PointSelector::deactivate()
{
  /*if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( false );
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }*/

  context_->getSelectionManager()->removeHighlight();
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int PointSelector::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  	int flags = 0;

	/*if( !moving_flag_node_ )
	{
	return Render;
	}*/

	//Ogre::Vector3 intersection;
	//Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

	//plant flag
	/*if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
	                                     ground_plane,
	                                     event.x, event.y, intersection ))
	{
	moving_flag_node_->setVisible( true );
	moving_flag_node_->setPosition( intersection );
	current_flag_property_->setVector( intersection );

	if( event.leftDown() )
	{
	  makeFlag( intersection );
	  current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
	  return Render | Finished;
	}
	}
	else
	{
	moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
	}*/

	//check if mouse moved
	/*if(event.x==event.last_x and event.y==event.last_y)
		return flags;*/

	//selection rectangle
	rviz::SelectionManager* sel_manager = context_->getSelectionManager();

	if(event.leftDown()){
		selecting_ = true;

		sel_start_x_ = event.x;
		sel_start_y_ = event.y;
	}

	if(selecting_){
		sel_manager->highlight(event.viewport, sel_start_x_, sel_start_y_, event.x, event.y);

		if(event.leftUp()){
			///////////////////////////////////////// TESTING AREA ////////////////////////////////////////////////////////////

			/***************** Way No1 ******************
			 *                                          *
			 * Given the instances of the objects, find *
			 * their 2D positions from their world 3D	*
			 * positions. (DEPRECATED)					*
			 *											*
			 ********************************************/

			/*ROS_INFO("\nStart x,y: %d, %d\nEnd x,y: %d, %d\n-------------------------------------\n", sel_start_x_, sel_start_y_, event.x, event.y);

			for(int i=0; i < manual_objects_.size(); i++){
				Ogre::Vector3 point_pos = manual_objects_[i]->getParentNode()->getPosition();
				Ogre::Vector2 point_coords = rviz::project3DPointToViewportXY(event.viewport, point_pos);

				if( (point_coords.x <= sel_start_x_ or point_coords.x <= event.x) and
					(point_coords.x >= sel_start_x_ or point_coords.x >= event.x) and
					(point_coords.y <= sel_start_y_ or point_coords.y <= event.y) and
					(point_coords.y >= sel_start_y_ or point_coords.y >= event.y) )
				{
					ROS_INFO("\nPoint %d in!!!\n", i);

					//change color of every point in selection rectangle
			  		manual_objects_[i]->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
			  		manual_objects_[i]->position(0.0,0.0,0.0);
					manual_objects_[i]->colour(1.0,0.0,0.0,1.0);
					manual_objects_[i]->end();
					manual_objects_[i]->getParentNode()->setPosition(point_pos);
					manual_objects_[i]->getParentNode()->_update(true,true);
				}
			}
			ROS_INFO("\n--------------------------------------\n");*/

			/******************* Way No2 ********************
			 *                                              *
			 * Given the derived 2D positions of selection, *
			 * find approximation of world 3D positions and *
			 * their relative object instances.             *
			 * (DEPRECATED)									*
			 *											    *
			 ************************************************/

			//actual selection (defined by type)
			rviz::SelectionManager::SelectType type = rviz::SelectionManager::Replace;
			//rviz::M_Picked selection;

			if(event.shift())
				type = rviz::SelectionManager::Add;
			else if(event.control())
				type = rviz::SelectionManager::Remove;

			sel_manager->select(event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type);

			//get 3D coords from selection
			/*std::vector<Ogre::Vector3> points_pos;
			int width = std::abs(sel_start_x_ - event.x);
			int height = std::abs(sel_start_y_ - event.y);

			if(sel_start_x_ < event.x and sel_start_y_ < event.y)
				sel_manager->get3DPatch(event.viewport, sel_start_x_, sel_start_y_, width, height, true, points_pos);
			else if(sel_start_x_ > event.x and sel_start_y_ < event.y)
				sel_manager->get3DPatch(event.viewport, event.x, sel_start_y_, width, height, true, points_pos);
			else if(sel_start_x_ < event.x and sel_start_y_ > event.y)
				sel_manager->get3DPatch(event.viewport, sel_start_x_, event.y, width, height, true, points_pos);
			else if(sel_start_x_ > event.x and sel_start_y_ > event.y)
				sel_manager->get3DPatch(event.viewport, event.x, event.y, width, height, true, points_pos);*/

			//handle selected point cloud properties
			rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
			int num_points = model->rowCount();

			std::vector<int> found_clusters;
			int found_point_cntr = 0;
			rviz_annotator::PointSelection found_points;

			for(int i=0; i < num_points; i++)
			{
				QModelIndex child_index = model->index(i, 0);
				rviz::Property *child = model->getProp(child_index);
				//ROS_WARN("Num ch: %d\n", child->numChildren());
				//ROS_INFO("Name %d: %s\n", i, (child->getNameStd()).c_str());

				rviz::VectorProperty *vec_child = (rviz::VectorProperty*) child->childAt(0);
				Ogre::Vector3 pc_vec = vec_child->getVector();

				rviz::ColorProperty* color_child = (rviz::ColorProperty*) child->childAt(1);
				rviz::FloatProperty* alpha_child = (rviz::FloatProperty*) child->childAt(2);
				Ogre::ColourValue pc_colour = color_child->getOgreColor();

				//ROS_INFO("Color of %d: %f, %f, %f\n", i, pc_colour.r, pc_colour.g, pc_colour.b);
				//ROS_INFO("\nPos: %f, %f, %f\n", pc_vec.x, pc_vec.y, pc_vec.z);

				//get point id from name
				std::string point_name;
				int point_id;
				std::istringstream iss(child->getNameStd());

				iss >> point_name >> point_id;
				
				//find cluster id of the selected point
				bool found_point = false;

				for(size_t j=0; j < cluster_msg.clusters.size(); j++){
					pcl::PCLPointCloud2 cloud2;
        			pcl_conversions::toPCL( cluster_msg.clusters[j] , cloud2);

        			pcl::PointCloud<pcl::PointXYZ> cloud;
        			pcl::fromPCLPointCloud2(cloud2, cloud);

					for(size_t k=0; k < cloud.points.size(); k++)
						if(pc_vec.x == cloud.points[k].x and pc_vec.y == cloud.points[k].y and pc_vec.z == cloud.points[k].z)
						{
							geometry_msgs::Point32 pt;

							found_point = true;

							pt.x = cloud.points[k].x;
							pt.y = cloud.points[k].y;
							pt.z = cloud.points[k].z;
							found_points.points.push_back(pt);
							found_points.point_pos.push_back(k);
							break;
						}

					if(found_point){
						found_clusters.push_back(cluster_msg.cluster_id[j]);
						found_points.cluster_pos.push_back(j);
						found_point_cntr++;
						break;
					}
				}
			}

			if(found_point_cntr < num_points)
				ROS_WARN("Point selector: Missing cloud points!\n");

			//find different id's
			std::string state_msg;

			if(found_point_cntr != 0)
			{
				auto it_begin = found_clusters.begin();
				int current_id = *it_begin;
				it_begin++;
				bool isDifferent = false;

				for(auto it = it_begin; it != found_clusters.end(); it++)
				{
					if(*it != current_id){
						isDifferent = true;
						break;
					}
				}

				//publish state - all id's identical: clean cluster, different id's: dirty cluster
				if(isDifferent)
					state_msg = "Dirty cluster!";
				else
					state_msg = "Clean cluster";
			}
			else{
				ROS_WARN("No selected points found!\n");

				state_msg = "";
			}

			found_points.state_msg.data = state_msg;
			found_points.segment_stamp = cluster_msg.header.stamp;

			selection_pub.publish(found_points);
			ros::spinOnce();

			///////////////////////////////////////// TESTING AREA ////////////////////////////////////////////////////////////

			selecting_ = false;
		}

		flags |= Render;
	}
	else
		sel_manager->highlight(event.viewport, event.x, event.y, event.x, event.y);

	return flags;
}

/*// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void PointSelector::makeFlag( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  flag_nodes_.push_back( node );

  ROS_INFO("Flag at: %f, %f, %f\n", position.x,position.y,position.z);
}*/

/*// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void PointSelector::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( flag_config );
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void PointSelector::load( const rviz::Config& config )
{
  // Here we get the "Flags" sub-config from the tool config and loop over its entries:
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Flag " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( flag_config );
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
  }
}*/

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_annotator::PointSelector,rviz::Tool )