#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <m3d_msgs/calibration.h>
typedef pcl::PointXYZ PointType;




class aggregator_node;
aggregator_node * m_aggregator_node;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);

class scanSegment
{
public:
	pcl::PointCloud<PointType> segment;
	Eigen::Affine3f original_Transform;

};

class pointCloudAggregator
{


public:
	pointCloudAggregator()
	{

		angularDistance = 2.0*M_PI;
		firstScan = false;
	}
	void addSegment (pcl::PointCloud<PointType> &p, tf::Transform &transform)
	{
		if (!creatingPointCloud) return;
		scanSegment segment;
		segment.segment = p;
		Eigen::Affine3d Af3d;
		tf::transformTFToEigen(transform, Af3d);

		segment.original_Transform = Af3d.cast<float>();
		// compute addition of traveled angular distance
		tf::Quaternion q = transform.getRotation();

		segments.push_back(segment);

		if (firstScan)
		{
			begin = q;
			firstScan = false;
			actual = q;
		}
		else
		{
			double d_distance = q.angleShortestPath(actual);
			if (!isnan(d_distance))currentAngularDistance=currentAngularDistance + d_distance ;
			actual = q;
		}
	}

	bool createPointCloud()
	{
		creatingPointCloud = true;
		firstScan = true;
	}
	bool isPointcloudReady()
	{

		if ( currentAngularDistance > angularDistance)
		{
			return true;
		}
		return false;
	}

	double getProgress()
	{
		if (!creatingPointCloud) return -1;
		return 0.1*(floor(currentAngularDistance*1000.0/angularDistance));
		return 0;
	}
	std::vector <scanSegment> segments;
private:

	tf::Quaternion begin;
	tf::Quaternion actual;
	double currentAngularDistance;
	double angularDistance;
	bool creatingPointCloud;
	bool firstScan;
};


class aggregator_node
{
public:

	void getTranslationRotationFromParams(float x,float y, float z, float yaw, float pitch, float roll, Eigen::Vector3f &t_m, Eigen::Quaternionf &t_q)
	{
		Eigen::Vector3f t_rot_euler;

		t_m.x() = x;
		t_m.y() = y;
		t_m.z() = z;

		t_rot_euler.x() =  yaw;
		t_rot_euler.y() =  pitch;
		t_rot_euler.z() =  roll;

		t_q = Eigen::AngleAxisf(t_rot_euler[0], Eigen::Vector3f::UnitX())*
				Eigen::AngleAxisf(t_rot_euler[1], Eigen::Vector3f::UnitY())*
				Eigen::AngleAxisf(t_rot_euler[2], Eigen::Vector3f::UnitZ());

	}

	void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void * cookie)
	{
	  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	  if ((event.getKeySym () == "a" || event.getKeySym () == "A") && event.keyDown ())
	  {
		 std::cout << "accepted calibration, sending to driver \n";
		 m3d_msgs::calibration calib;
		 Eigen::Vector3f optimal_t;
		 Eigen::Quaternionf optimal_r;
		 getTranslationRotationFromParams(0,p[0],p[1],p[2],p[3],p[4],optimal_t,optimal_r);
		 calib.request.offsetMatrix.translation.x = optimal_t.x();
		 calib.request.offsetMatrix.translation.y = optimal_t.y();
		 calib.request.offsetMatrix.translation.z = optimal_t.z();
		 calib.request.offsetMatrix.rotation.x = optimal_r.x();
		 calib.request.offsetMatrix.rotation.y = optimal_r.y();
		 calib.request.offsetMatrix.rotation.z = optimal_r.z();
		 calib.request.offsetMatrix.rotation.w = optimal_r.w();

		 if (ros::service::call( "/"+unitName+"/m3d_calibration", calib))
		 {
			 std::cout << "called\n";
       std::cout << calib.response.result << "\n";
		 }
	  }
	}

	aggregator_node ():
	n("~"),tfListener(tfBuffer)
	{

		laserOffsetMatrix = Eigen::Affine3f::Identity();
		unitName = "m3d_test";
		n.param<std::string>("unit_name", unitName, "m3d_test");
		n.param<std::string>("pointCloudFrame", rootTransform, unitName+"/m3d_link");
		n.param<std::string>("rotLaserScan", rotLaserScan, "/"+unitName+"/rot_scan");
		n.param<std::string>("rotLaserPointCloud", rotLaserPointCloud, "/"+unitName+"/rot_cloud");
		std::cout << "will work with "<< unitName << "\n";


		progress_pub = n.advertise<std_msgs::Float32>("progress", 1);
		done_pub =  n.advertise<std_msgs::Bool>("done", 1);


		if (rotLaserScan.size()!=0)
			laserScan_sub  = n.subscribe(rotLaserScan, 1, &aggregator_node::rotLaserScanCallback, this);
		if (rotLaserPointCloud.size()!=0)
			pointCloud_sub = n.subscribe(rotLaserPointCloud, 1, &aggregator_node::rotLaserPointCloudCallback, this);
		mPointCloudAggregator.createPointCloud();
		ros::spin();

	}
	Eigen::Affine3f laserOffsetMatrix;
	pcl::PointCloud<PointType>::Ptr firstPcFilter;
	pcl::PointCloud<PointType>::Ptr secondPcFilter;

	int testData(float x,float y, float z, float yaw, float pitch, float roll)
	{

		Eigen::Vector3f t_m, t_rot_euler;

		t_m.x() = x;
		t_m.y() = y;
		t_m.z() = z;

		t_rot_euler.x() =  yaw;
		t_rot_euler.y() =  pitch;
		t_rot_euler.z() =  roll;

		Eigen::Quaternionf mat = Eigen::AngleAxisf(t_rot_euler[0], Eigen::Vector3f::UnitX())*
				Eigen::AngleAxisf(t_rot_euler[1], Eigen::Vector3f::UnitY())*
				Eigen::AngleAxisf(t_rot_euler[2], Eigen::Vector3f::UnitZ());


		Eigen::Affine3f m_addition = Eigen::Affine3f::Identity();
		m_addition.rotate(mat);
		m_addition.translate(t_m);
		laserOffsetMatrix = m_addition;
		pcl::PointCloud<PointType> firstPc;
		pcl::PointCloud<PointType> secondPc;
		//std::cout <<"Matrix offset : \n" << laserOffsetMatrix.matrix() << "\n";
		for (size_t i = 0; i< mPointCloudAggregator.segments.size(); i++)
		{
			pcl::PointCloud<PointType> mPointCloud;
			//std:: cout << "segment size " << mPointCloudAggregator.segments[i].segment.size()  << "\n";

			Eigen::Affine3f mm =   mPointCloudAggregator.segments[i].original_Transform * laserOffsetMatrix;
			pcl::transformPointCloud (mPointCloudAggregator.segments[i].segment, mPointCloud,   mm );
			for (size_t j =0; j < mPointCloudAggregator.segments[i].segment.size(); j++ )
			{
				if (mPointCloudAggregator.segments[i].segment[j].y>0)
				{
					firstPc.push_back(mPointCloud[j]);
				}
				else
				{
					secondPc.push_back(mPointCloud[j]);
				}
			}
		}

		//std:: cout << "first cloud size" << firstPc.size() << "\n";
		//std:: cout << "second cloud size" << secondPc.size() << "\n";


		pcl::PointCloud<PointType>::Ptr firstPcFilter(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr secondPcFilter(new pcl::PointCloud<PointType>) ;
		pcl::PointCloud<PointType>::Ptr extraPoints(new pcl::PointCloud<PointType>) ;

		this->firstPcFilter = firstPcFilter;
		this->secondPcFilter = secondPcFilter;


		pcl::VoxelGrid<PointType> grid;
		grid.setInputCloud (firstPc.makeShared());
		grid.setLeafSize (0.1, 0.1, 0.1);
		grid.filter (*firstPcFilter);

		grid.setInputCloud (secondPc.makeShared());
		grid.setLeafSize (0.1, 0.1, 0.1);
		grid.filter (*secondPcFilter);

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (firstPcFilter);
		kdtree.setSortedResults(false);

		int c =0;
		for(size_t i=0;i<secondPcFilter->size();++i)
		{

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			if ( kdtree.radiusSearch ((*secondPcFilter)[i], 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
			{
				c++;
				//extraPoints->push_back((*secondPcFilter)[i]);
			}
		}
		//std::cout << "ouliners :" << extraPoints->size() << "\n";
		return c;

	}



	float m_rand()
	{
		return -1.0f + 2 *((float) rand()) / ((float)RAND_MAX);
	}
	void publishPointcloud()
	{

		srand(time(0));

		if (!mPointCloudAggregator.isPointcloudReady()) return;



		std::cout <<" finished getting data\n";

    std::ofstream output;
	    output.open ("/tmp/out.txt");
		testData(0,0,0,0,0,0);


		int dims = 5;



		p.push_back(0.0);
		p.push_back(0.0);
		p.push_back(0.0);

		p.push_back(0.05);
		p.push_back(0.05);

    std::vector<float> start = p;
		float best_error =  testData(0,p[0],p[1],p[2],p[3],p[4]);
		int n =0;
		float incr  = 100;
		float temperature = 1.0f;
		float alpha = 0.999;
		while (temperature > 0.00001f)
		{
			float c_p[5];
			c_p[0] = start[0] + 0.003 *m_rand();
			c_p[1] = start[1] + 0.003 *m_rand();
			c_p[2] = start[2] + 0.01 *m_rand();
			c_p[3] = start[3] + 0.2 *m_rand();
			c_p[4] = start[4] + 0.2 *m_rand();

			float c_error = testData(0,c_p[0],c_p[1],c_p[2],c_p[3],c_p[4]);


			float ac_p = exp((best_error-c_error) / temperature);
			if (ac_p > ((float) rand()) / ((float)RAND_MAX) )
			{
				best_error = c_error;
				p[0] = c_p[0];
				p[1] = c_p[1];
				p[2] = c_p[2];
				p[3] = c_p[3];
				p[4] = c_p[4];
			}

			temperature = alpha * temperature;

			std::cout << "system temperature : " <<  temperature << " best_error : " << best_error << "\n";
      output << 0 << "\t"<<p[0]<< "\t"<<p[1]<< "\t"<<p[2]<< "\t"<<p[3]<< "\t"<<p[4]<<"\n";
      output.flush();

      viewer.removePointCloud("first_cloud");
			viewer.removePointCloud("second_cloud");


			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color_handler (firstPcFilter, 230, 20, 20); // Red
			 viewer.addPointCloud (firstPcFilter, first_color_handler, "first_cloud");

			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> second_color_handler (secondPcFilter, 20, 230, 20); // Red
			 viewer.addPointCloud (secondPcFilter, second_color_handler, "second_cloud");

			 viewer.spinOnce();

		}
		std::cout << "ANEALLING ENDED\n";
		std::cout << "Found params:\n";
		std::cout << "p[0]:" << p[0] << "\n";
		std::cout << "p[1]:" << p[1] << "\n";
		std::cout << "p[2]:" << p[2] << "\n";
		std::cout << "p[3]:" << p[3] << "\n";
		std::cout << "p[4]:" << p[4] << "\n";
		std::cout << "p[5]:" << p[5] << "\n";
		std::cout << "press A to accept, q to quit" << "\n";



		viewer.registerKeyboardCallback (&aggregator_node::keyboardEventOccurred, *this, (void*)&viewer);


		 viewer.removePointCloud("first_cloud");
		 viewer.removePointCloud("second_cloud");


		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> first_color_handler (firstPcFilter, 230, 20, 20); // Red
          viewer.addPointCloud (firstPcFilter, first_color_handler, "first_cloud");

	      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> second_color_handler (secondPcFilter, 20, 230, 20); // Red
		  viewer.addPointCloud (secondPcFilter, second_color_handler, "second_cloud");

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		   viewer.spinOnce ();
		}
		std::exit(0);

	}


	void rotLaserPointCloudCallback(const sensor_msgs::PointCloud2Ptr& scan )
	{
		ROS_DEBUG("Recieved scan, frame : %s", scan->header.frame_id.c_str());
		geometry_msgs::TransformStamped transform;
	    try{
	    	tfBuffer.canTransform(rootTransform, scan->header.frame_id, scan->header.stamp, ros::Duration(0.2));
	    	transform = tfBuffer.lookupTransform(rootTransform, scan->header.frame_id, scan->header.stamp);
	        pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(*scan,pcl_pc2);
			pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
			pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
			tf::Transform tf_transform;
			tf::transformMsgToTF (transform.transform, tf_transform);
			//mPointCloudAggregator.addPoints((*temp_cloud)[i], tf_transform);
			mPointCloudAggregator.addSegment(*temp_cloud,tf_transform);
			publishPointcloud();
	    }
	    catch (tf2::TransformException &ex) {
	      ROS_WARN("%s",ex.what());
	    }


	}

	void rotLaserScanCallback( const sensor_msgs::LaserScanPtr& scan)
	{
		ROS_DEBUG("Recieved pointcloud, frame : %s", scan->header.frame_id.c_str());
		geometry_msgs::TransformStamped transform;
		try{
			tfBuffer.canTransform(rootTransform, scan->header.frame_id, scan->header.stamp, ros::Duration(0.2));
			transform = tfBuffer.lookupTransform(rootTransform, scan->header.frame_id, scan->header.stamp);
			tf::Transform tf_transform;
			tf::transformMsgToTF (transform.transform, tf_transform);
			pcl::PointCloud<PointType> segmentPc;

			for (size_t i=0; i < scan->ranges.size(); i++)
			{
				tf::Vector3 pointOut;
				float ang = scan->angle_min+i*scan->angle_increment;
				float dist = scan->ranges[i];
				float intensity = 0;
				if (i <scan->intensities.size())
				{
					intensity = 0.01* scan->intensities[i];
				}

				PointType point;
				point.x = cos(ang)*dist;
				point.y = sin(ang)*dist;
				point.z = 0;
				if (dist > 1 and dist  < (scan->range_max - 0.25) ) segmentPc.push_back(point);
			}
			//std::cout << "adding segment with size " << segmentPc.size() << "\n";
			mPointCloudAggregator.addSegment(segmentPc,tf_transform);
			publishPointcloud();
		}
		catch (tf2::TransformException &ex) {
		  ROS_WARN("%s",ex.what());
		}


	}


private:
	ros::NodeHandle n;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	std::string rootTransform;
	std::string rotLaserScan;
	std::string rotLaserPointCloud;

	ros::Publisher cloud_pub;
	ros::Publisher progress_pub;
	ros::Publisher done_pub;

	ros::Subscriber rqt_sub;
	ros::Subscriber laserScan_sub;
	ros::Subscriber pointCloud_sub;
	pointCloudAggregator mPointCloudAggregator;
	pcl::visualization::PCLVisualizer viewer;
	std::string unitName;
	/// optimized params
	std::vector<float> p;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_aggregator");
	m_aggregator_node= new aggregator_node();
}
