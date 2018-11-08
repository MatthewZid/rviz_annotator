#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <rviz_annotator/point_class.h>
#include <fstream>

#include <pcl_ros/point_cloud.h>
#include <ros/package.h>

namespace rosbag
{
class MessageInstance;
class View;
}

bool sortWay(rviz_annotator::PointClass a, rviz_annotator::PointClass b)
{
	return(a.segment_stamp.toSec() < b.segment_stamp.toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "exportBag");

	ros::NodeHandle nh;

	//initialize files
	rosbag::Bag input_bag;
	rosbag::Bag output_bag;

	std::string homepath = std::getenv("HOME");
	std::string input_bagname = argv[1];
	std::string output_bagname = "annotated_" + input_bagname;

	QString bagfiles_dir;
	QString pc_segments_topic;
	QString pc2_topic;

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
	config.mapGetString("bagfiles_dir", &bagfiles_dir);
	config.mapGetString("pc_segments_topic", &pc_segments_topic);
	config.mapGetString("pc2_topic", &pc2_topic);

	//read csv file
	std::vector<rviz_annotator::PointClass> cluster = rviz_annotator::readcsv();

    //open input bagfile
	try
	{
		//std::string bagpath = homepath + "/Ros_WS/bagfiles/" + input_bagname;
		std::string bagpath = homepath + "/" + bagfiles_dir + "/" + input_bagname;
		input_bag.open(bagpath);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << input_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

    //open output bagfile
    try
	{
		std::string bagpath = homepath + "/" + bagfiles_dir + "/" + output_bagname;
		output_bag.open(bagpath, rosbag::bagmode::Write);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << output_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

	if(!ros::ok()){
		input_bag.close();
		output_bag.close();
		return -1;
	}

	rosbag::View full_bag_view;
	full_bag_view.addQuery(input_bag);

	ros::Time initial_time = full_bag_view.getBeginTime();
	ros::Time finish_time = full_bag_view.getEndTime();

	rosbag::View bag_view;
	bag_view.addQuery(input_bag, initial_time, finish_time);

	//sort clusters according to stamp
	std::sort(cluster.begin(), cluster.end(), sortWay);

	//create timestamp map
	std::map< double, std::vector<size_t> > stamp_map;

	for(rosbag::MessageInstance m : bag_view)
	{
		if(!m.isType<pointcloud_msgs::PointCloud2_Segments>() or m.getTopic() != ("/" + pc_segments_topic))
			continue;

		boost::shared_ptr<pointcloud_msgs::PointCloud2_Segments> pcs_msg = m.instantiate<pointcloud_msgs::PointCloud2_Segments>();

		std::vector<size_t> posvec;

		for(size_t i=0; i < cluster.size(); i++)
			if(pcs_msg->header.stamp.toSec() == cluster[i].segment_stamp.toSec())
				posvec.push_back(i);

		if(!posvec.empty())
			stamp_map.insert(std::make_pair(pcs_msg->header.stamp.toSec(), posvec));
	}

	//class count
	std::set<std::string> clusters_set;

	for(size_t i=0; i < cluster.size(); i++)
		clusters_set.insert(cluster[i].name);

	size_t class_num = clusters_set.size();

	//class-color map
	std::map<std::string, pcl::PointXYZRGB> color_map;

	for(auto it = clusters_set.begin(); it != clusters_set.end(); it++)
	{
		size_t pos = std::distance(clusters_set.begin(), it);
		int color_id = pos % 3;
		std::string class_name = *it;
		pcl::PointXYZRGB color_val;

		if(color_id == 0){
			color_val.r = 255;
			color_val.g = 255 * pos % 255;
			color_val.b = 255 * pos % 255;
		}
		else if(color_id == 1){
			color_val.r = 255 * pos % 255;
			color_val.g = 255;
			color_val.b = 255 * pos % 255;
		}
		else if(color_id == 2){
			color_val.r = 255 * pos % 255;
			color_val.g = 255 * pos % 255;
			color_val.b = 255;
		}

		color_map.insert(std::make_pair(class_name, color_val));
	}

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > pc_msg_vec;
	bool found = false;

	std::cout << "Please wait..." << std::endl <<std::endl;

	//write messages to output bagfile, consulting csv annotations
	for(rosbag::MessageInstance m : bag_view)
	{
		if(!m.isType<pointcloud_msgs::PointCloud2_Segments>() or m.getTopic() != ("/" + pc_segments_topic)){
			if(m.isType<sensor_msgs::PointCloud2>() and m.getTopic() == ("/" + pc2_topic))
			{
				if(!found)
				{
					output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
					continue;
				}

				boost::shared_ptr<sensor_msgs::PointCloud2> pc2_msg = m.instantiate<sensor_msgs::PointCloud2>();
				boost::shared_ptr<sensor_msgs::PointCloud2> accumulator(boost::make_shared<sensor_msgs::PointCloud2>());

    			for(size_t i=0; i < pc_msg_vec.size(); i++)
    			{
    				pcl::PCLPointCloud2 clouds;
    				sensor_msgs::PointCloud2 cluster_msgs;

			        pcl::toPCLPointCloud2(pc_msg_vec[i], clouds);
			        pcl_conversions::fromPCL(clouds, cluster_msgs);

			        cluster_msgs.header.stamp = ros::Time::now();
			        cluster_msgs.header.frame_id = pc2_msg->header.frame_id;

			        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(*accumulator);
			        pcl::concatenatePointCloud(cluster_msgs, tmp, *accumulator);
    			}

    			accumulator->header.stamp = ros::Time::now();
    			accumulator->header.frame_id = pc2_msg->header.frame_id;

    			output_bag.write(m.getTopic(), m.getTime(), accumulator, m.getConnectionHeader());
			}
			else
				output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());

			continue;
		}

		boost::shared_ptr<pointcloud_msgs::PointCloud2_Segments> pcs_msg = m.instantiate<pointcloud_msgs::PointCloud2_Segments>();

		auto it = stamp_map.find(pcs_msg->header.stamp.toSec());

        if(it == stamp_map.end()){
        	found = false;
        	output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
        	continue;
        }

        std::vector< pcl::PointCloud<pcl::PointXYZRGB> > cloud_vec;

		//if pointcloud2_segments, consult csv and then write
        for(size_t i=0; i < pcs_msg->clusters.size(); i++)
        {
        	pcl::PCLPointCloud2 cloud2;
	        pcl_conversions::toPCL(pcs_msg->clusters[i], cloud2);

	        pcl::PointCloud<pcl::PointXYZ> cloudtemp;
	        pcl::PointCloud<pcl::PointXYZRGB> cloud;
	        pcl::fromPCLPointCloud2(cloud2, cloudtemp);
	        pcl::copyPointCloud(cloudtemp, cloud);

	        for(size_t j=0; j < cloud.points.size(); j++)
	        {
	        	cloud.points[j].r = 80;
				cloud.points[j].g = 80;
				cloud.points[j].b = 80;
	        }

	        cloud_vec.push_back(cloud);
        }

        for(size_t i=0; i < it->second.size(); i++)
        {
        	size_t pos = (it->second)[i];

        	for(size_t j=0; j < cluster[pos].points.size(); j++)
        	{
        		auto colit = color_map.find(cluster[pos].name);
        		size_t clpos = cluster[pos].cluster_pos[j];
        		size_t ppos = cluster[pos].point_pos[j];

        		cloud_vec[clpos].points[ppos].r = colit->second.r;
				cloud_vec[clpos].points[ppos].g = colit->second.g;
				cloud_vec[clpos].points[ppos].b = colit->second.b;
        	}
        }

        pc_msg_vec = cloud_vec;
        found = true;

        std::vector<sensor_msgs::PointCloud2> cloud2_vec;

        for(size_t i=0; i < cloud_vec.size(); i++)
        {
        	pcl::PCLPointCloud2 clouds;
			sensor_msgs::PointCloud2 cluster_msgs;

	        pcl::toPCLPointCloud2(cloud_vec[i], clouds);
	        pcl_conversions::fromPCL(clouds, cluster_msgs);

	        std::string frame_id = pcs_msg->clusters[i].header.frame_id;
	        cluster_msgs.header.stamp = ros::Time::now();
	        cluster_msgs.header.frame_id = frame_id;

	        pcs_msg->clusters[i] = cluster_msgs;
        }

        pcs_msg->header.stamp = ros::Time::now();

        output_bag.write(m.getTopic(), m.getTime(), pcs_msg, m.getConnectionHeader());
	}

	input_bag.close();
	output_bag.close();

	ROS_INFO("Exported annotated bag\n");

	return 0;
}