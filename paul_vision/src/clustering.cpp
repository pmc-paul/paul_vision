
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
 

ros::Publisher pub_cloud;
ros::Publisher pub_cluster;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_msg);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (* cloudFilteredPtr);
  
  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

   // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);
  pass.setInputCloud (xyzCloudPtrFiltered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.06, 0.06);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.01); // 1cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrFiltered);
  ec.extract (cluster_indices);
  std::cout << "number of clusters found: " << cluster_indices.size() << std::endl;
  
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

  // create a pcl object to hold the extracted cluster
  pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &index : it->indices){
      cloud_cluster->push_back ((*xyzCloudPtrFiltered)[index]); 
      clusterPtr->push_back ((*xyzCloudPtrFiltered)[index]); }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "temp cluster size: " << cloud_cluster->size() << std::endl;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    for (int i = 0; i < cloud_cluster->points.size(); i++){
        x += cloud_cluster->points[i].x;
        y += cloud_cluster->points[i].y;
        z += cloud_cluster->points[i].z;
    }
    x = x / cloud_cluster->points.size();
    y = y / cloud_cluster->points.size();
    z = z / cloud_cluster->points.size();
    std::cout << "cluster location x: " << x << " y: " << y << " z: " << z << std::endl;
    pcl::toPCLPointCloud2( *cloud_cluster ,outputPCL);
    pcl_conversions::fromPCL(outputPCL, output);
    pcl_conversions::fromPCL(cloud_msg->header, output.header);
    pub_cluster.publish(output);
  }

  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  pcl_conversions::fromPCL(cloud_msg->header, output.header);
  pub_cloud.publish(output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_clustering");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud_output", 10);
  pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("cluster_output", 10);

  // Spin
  ros::spin ();
}
