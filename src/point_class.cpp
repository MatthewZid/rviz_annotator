#include <rviz_annotator/point_class.h>

namespace rviz_annotator
{

std::vector<PointClass> readcsv()
{
	std::vector<PointClass> cc;

	//read YAML
	std::string pkg_path = ros::package::getPath("rviz_annotator");

	if(pkg_path == "")
	{
		ROS_FATAL("Could not find package!\nPackage must be named rviz_annotator");
		ros::shutdown();
	}

	std::string param_path = pkg_path + "/config/plugin_params.yaml";

	YAML::Node config = YAML::LoadFile(param_path);

	const std::string csv_dir = config["csv_dir"].as<std::string>();

	//std::string homepath = std::getenv("HOME");
	std::string filename = "annotation.csv";
	std::string csv_path = csv_dir + "/" + filename;

	std::ifstream csvfile(csv_path);

	if(!csvfile.is_open())
	{
	ROS_WARN("%s could not be opened!\n", filename.c_str());
	return cc;
	}

	ROS_INFO("Loading csv...\n");

	//read annotation fields
	std::string line;

	while(std::getline(csvfile, line))
	{
		PointClass pc;
		std::istringstream lss(line);

		std::getline(lss, line, ',');
		pc.name = line;

		double tm;
		std::getline(lss, line, ',');
		std::istringstream(line) >> tm;
		pc.stamp.fromSec(tm);

		std::getline(lss, line, ',');
		std::istringstream(line) >> tm;
		pc.segment_stamp.fromSec(tm);

		std::getline(lss, line, ',');
		pc.topic = line;

		std::getline(lss, line, ',');
		pc.type = line;

		std::getline(lss, line, '[');
		std::getline(lss, line);

		//find x,y,z,v,w in list [((x,y,z),v,w),...]
		std::smatch m;
		std::regex reg("^\\(\\(([-\\.[:digit:]]+,[-\\.[:digit:]]+,[-\\.[:digit:]]+)\\)(,[-\\.[:digit:]]+,[-\\.[:digit:]]+)\\),?(.*)$");
		std::string rem = line;
		std::vector<geometry_msgs::Point32> pvec;
		std::vector<long> clpos;
		std::vector<long> ppos;

		while(rem != "]"){
			std::string temp;
			std::string result;
			std::regex_replace (std::back_inserter(result), rem.begin(), rem.end(), reg, "$1$2");
			std::regex_replace (std::back_inserter(temp), rem.begin(), rem.end(), reg, "$3");

			//point
			std::istringstream pss(result);
			std::string num;
			geometry_msgs::Point32 point;

			double pt;
			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.x = pt;

			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.y = pt;

			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.z = pt;

			pvec.push_back(point);

			//cluster pos
			long pos;
			std::getline(pss, num, ',');
			std::istringstream(num) >> pos;
			clpos.push_back(pos);

			//point pos
			std::getline(pss, num, ',');
			std::istringstream(num) >> pos;
			ppos.push_back(pos);

			rem = temp;
		}

		pc.points = pvec;
		pc.cluster_pos = clpos;
		pc.point_pos = ppos;

		cc.push_back(pc);
	}

	csvfile.close();

	ROS_INFO("%s loaded successfully\n", filename.c_str());

	return cc;
}

/*size_t binarySearch(const std::vector<PointClass>& pcvec, size_t l, size_t r, const ros::Time x)
{
	if(l <= r)
	{
		size_t mid = l + (r - l) / 2;

		if(pcvec[mid].stamp.toSec() == x.toSec())
			return mid;

		if(pcvec[mid].stamp.toSec() > x.toSec())
			return binarySearch(pcvec, l, mid -1, x);

		return binarySearch(pcvec, mid + 1, r, x);
	}

	return -1;
}

size_t cluster_search(const std::vector<PointClass>& pcvec, const ros::Time x)
{
	return binarySearch(pcvec, 0, pcvec.size()-1, x);
}*/

} // end namespace