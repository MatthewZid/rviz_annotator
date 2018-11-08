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
  : selecting_(false)
  , sel_start_x_(0)
  , sel_start_y_(0)
{
  shortcut_key_ = 's';
}

PointSelector::~PointSelector()
{
}

void PointSelector::onInitialize()
{
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

	/*YAML::Node conf = YAML::LoadFile("package://rviz_annotator/config/plugin_params.yaml");

	sel_topic = conf["sel_topic"].as<std::string>();
	pc_segments_topic = conf["pc_segments_topic"].as<std::string>();*/

	rviz::Config config;
	rviz::YamlConfigReader conf_reader;

	std::string pkg_path = ros::package::getPath("rviz_annotator");

	if(pkg_path == "")
	{
		ROS_FATAL("Could not find package!\nPackage must be named rviz_annotator");
		ros::shutdown();
	}

	std::string param_path = pkg_path + "/config/plugin_params.yaml";

	conf_reader.readFile(config, QString(param_path.c_str()));
	config.mapGetString("sel_topic", &sel_topic);
	config.mapGetString("pc_segments_topic", &pc_segments_topic);

    //std::string input_topic = "pointcloud2_cluster_tracking/clusters";
    std::string input_topic = pc_segments_topic.toStdString();

    pointsub = nh.subscribe(input_topic, 1, pointsCallback);
    //selection_pub = nh.advertise<rviz_annotator::PointSelection>("rviz_annotator/selection_topic", 1);
    selection_pub = nh.advertise<rviz_annotator::PointSelection>(sel_topic.toStdString(), 1);
}

void PointSelector::activate()
{
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
}

void PointSelector::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
}

int PointSelector::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  	int flags = 0;

	//Ogre::Vector3 intersection;
	//Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

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

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_annotator::PointSelector,rviz::Tool )