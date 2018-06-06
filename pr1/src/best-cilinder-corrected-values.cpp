#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include <sound_play/sound_play.h>

#include <vector>
#include <boost/algorithm/string.hpp>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher pustr;
ros::Publisher pub_marker_array;
ros::Subscriber goal_sub;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZ PointT;

using namespace std;
// kakšna je oddaljenost
float napaka = 0.4;
int count_marker_ids = 0;

// should pcl run?
bool run_cloud = false;

// kordinati ki prispejo na goal_sub od robota
float Xpos = 0, Ypos = 0;

float calculateDistance(float x_cylinder, float y_cylinder, float x_robot, float y_robot);

class Cylinder {
    public:
      float x;   // x point
      float y;   // y point
      float z;   // z point
      int found; // how many times cylinder was found

      Cylinder(float x1, float y1, float z1)
      {
        found = 1;
        x = x1;
        y = y1;
        z = z1;
        //std::cerr << "NEW CYLINDER: " + to_string(x) + " " + to_string(y) + " " + to_string(z) << endl;
      }

      bool sameCylinders(float x1, float y1, float z1)
      {
        //float diff_x = fabs(x - x1);
        //float diff_y = fabs(y - y1);
        //float diff_z = fabs(z - z1);
        float distance = calculateDistance(x, y, x1, y1);
        //std::cerr << "[sameCylinders] distance is: " + to_string(distance)<< endl;

        if(distance < napaka){
          return true;
        }

        //if((diff_x > 0.5) && (diff_y > 0.5)){
        //  return true;
        //}

        return false;
      }

      string printData()
      {
        return "[" + to_string(x) + "," + to_string(y) + "]";
      }
};


// vector for cylinders
vector<Cylinder> cylinder_array;

// possible good cylinders
vector<Cylinder> possible_cylinders;

float calculateDistance(float x_cylinder, float y_cylinder, float x_robot, float y_robot)
{
  return sqrt(pow((x_cylinder - x_robot),2) + pow((y_cylinder - y_robot), 2));
}





float calculateAngle(float x_cylinder, float y_cylinder, float x_robot, float y_robot)
{
  std::cerr << "CYL ROBOT :" << x_cylinder << " " << y_cylinder << " " << x_robot << " " << y_robot << endl;
  string abc = "CYL ROBOT :" + to_string(x_cylinder) + " " + to_string(y_cylinder) + " " + to_string(x_robot) + " " + to_string(y_robot);
  std_msgs::String msg_coordinates_to_go;
  std::stringstream ss_coordinates_to_go;
  ss_coordinates_to_go << abc;
  msg_coordinates_to_go.data = ss_coordinates_to_go.str();
  pustr.publish(msg_coordinates_to_go);
  return atan2(y_cylinder - y_robot, x_cylinder - x_robot);
}

// insert distance = distane - 0.5m
// distance = 0.5
string calculatePoint(float distance, float angle, float x_cyl, float y_cyl)
{
  float X = x_cyl + (distance * cos(angle));
  float Y = y_cyl + (distance * sin(angle));
  std::cerr << "GOTO: " << X << " " << Y << endl;
  string abc = "GOTO: " + to_string(X) + " " + to_string(Y);
  std_msgs::String msg_coordinates_to_go;
  std::stringstream ss_coordinates_to_go;
  ss_coordinates_to_go << abc;
  msg_coordinates_to_go.data = ss_coordinates_to_go.str();
  pustr.publish(msg_coordinates_to_go);

  return "FOUND_CYLINDER " + to_string(X) + " " + to_string(Y)+  " " + to_string(angle-1.57) + " " + to_string(cylinder_array.size() - 1);
}





bool checkCylindersSimilarity(Cylinder cilinder)
{
  if(isnan(cilinder.x) || isnan(cilinder.y) || isnan(cilinder.z))
    return true;

  //std::cerr << "[CHECKING FOCKING CILINDERS] started" << endl;
  for (unsigned i=0; i < possible_cylinders.size(); i++) {
    if(possible_cylinders[i].sameCylinders(cilinder.x, cilinder.y, cilinder.z)){
      //std::cerr << "[CHECKING FOCKING CILINDERS] cilinders similar" << endl;
      possible_cylinders[i].found++;
      std::cerr << "cylinder found " + to_string(possible_cylinders[i].found) + " times" << endl;
      if(possible_cylinders[i].found == 3)
        return false;
    }
  }
  //std::cerr << "[CHECKING FOCKING CILINDERS] cilindars not similar" << endl;
  return true;
}

void printCylinderArray()
{
  //std::cerr << "[PRINTING FOCKING CILINDERS] started printing" << endl;
  for (unsigned i=0; i < cylinder_array.size(); i++) {
      std::cerr << cylinder_array[i].printData() << endl;
  }
  //std::cerr << "[PRINTED FOCKING CILINDERS] stopped printing" << endl;
}

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{

  if(!run_cloud)
    return;
  std::cerr << "IZVAJANJE!! " + to_string(Xpos) + " " + to_string(Ypos)  << endl;
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();
  
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  
  // Read in the cloud data
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.025, 1);//(0, 1.5)
  pass.filter (*cloud_filtered);
  //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  //std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  
  pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2 (*cloud_plane, outcloud_plane);
  pubx.publish (outcloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.5);//(0.1)
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.06, 0.2);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()){ 
    //std::cerr << "Can't find the cylindrical component." << std::endl;
  }
  else
  {
    //std::cerr << "I found something." << std::endl;
    // std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
          
          pcl::compute3DCentroid (*cloud_cylinder, centroid);
          //std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << std::endl;

    //Create a point in the "camera_rgb_optical_frame"
          geometry_msgs::PointStamped point_camera;
          geometry_msgs::PointStamped point_map;
        visualization_msgs::Marker marker;
          geometry_msgs::TransformStamped tss;
          
          point_camera.header.frame_id = "camera_rgb_optical_frame";
          point_camera.header.stamp = ros::Time::now();

        point_map.header.frame_id = "map";
          point_map.header.stamp = ros::Time::now();

      point_camera.point.x = centroid[0];
      point_camera.point.y = centroid[1];
      point_camera.point.z = centroid[2];

    try{
      time_test = ros::Time::now();

      //std::cerr << time_rec << std::endl;
      //std::cerr << time_test << std::endl;
          tss = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec);
          //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
    }
          catch (tf2::TransformException &ex)
    {
         ROS_WARN("Transform warning: %s\n", ex.what());
    }

          //std::cerr << tss ;

          tf2::doTransform(point_camera, point_map, tss);

        //std::cerr << "point_camera: " << point_camera.point.x << " " <<  point_camera.point.y << " " <<  point_camera.point.z << std::endl;

        //std::cerr << "point_map: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;

        marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();

          marker.ns = "cylinder";
          marker.id = count_marker_ids;

          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = point_map.point.x;
          marker.pose.position.y = point_map.point.y;
          marker.pose.position.z = point_map.point.z;
          marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

          marker.color.r=0.0f;
          marker.color.g=1.0f;
          marker.color.b=0.0f;
          marker.color.a=1.0f;

        marker.lifetime = ros::Duration();

        Cylinder cilinder (point_map.point.x, point_map.point.y, point_map.point.z);
        possible_cylinders.push_back(cilinder);

        // nadaljuj ce tega cilindra ni v arrayi, torej se tega cilindra nisem našel
        if(!checkCylindersSimilarity(cilinder)){
          //play sound
          //sc.playWaveFromPkg("sound_play", "/home/team_beta/ROS/src/pr1/music/up.ogg");

          // dodaj cilinder v array
          cylinder_array.push_back(cilinder);
          // izracunaj kot in točke kam se mora robot premaknit
          float angle = calculateAngle(point_map.point.x, point_map.point.y, Xpos, Ypos);
          string coordinates_to_go = calculatePoint(napaka, angle, point_map.point.x, point_map.point.y);
          // povecaj id naslednjega markerja
          count_marker_ids++;
          

          std::cerr << "********** [PUBLISHED] NEW CILINDER CORRECT! **********" << endl;
          printCylinderArray();
          visualization_msgs::MarkerArray ma; 
          ma.markers.push_back(marker);
          //pubm.publish (marker);

          // publish marker array(markers)
          pub_marker_array.publish (ma);
          //std::cerr << "[PUBLISHED] marker array" << endl;


          // publish string msg object
          std_msgs::String msg_coordinates_to_go;
          std::stringstream ss_coordinates_to_go;
          ss_coordinates_to_go << coordinates_to_go;
          msg_coordinates_to_go.data = ss_coordinates_to_go.str();
          pustr.publish(msg_coordinates_to_go);
          //std::cerr << "[PUBLISHED] string msg" << endl;

          // ### WORKING FROM TUTORIAL 
          // std_msgs::String msg;
          // std:stringstream ss;
          // ss << "hello" << endl;
          // msg.data = ss.str();
          // pustr.publish(msg);
          // ### WORKING FROM TUTORIAL 

          // publish point clouds
          pcl::PCLPointCloud2 outcloud_cylinder;
          pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud_cylinder);
          puby.publish (outcloud_cylinder);
          //std::cerr << "[PUBLISHED] point cloud" << endl;

        }

  }
  
}

void parse_goal(const std_msgs::String::ConstPtr& message){

    std::vector<std::string> content;
    boost::split(content, message->data, boost::is_any_of("\t "));

    if(content[0] == "going_to"){
        std:cerr << "CYLINDER going" << endl;
        run_cloud = false;
    }

    if(content[0] == "arrived_to"){
        Xpos = std::stof(content[1]);
        Ypos = std::stof(content[2]);
        std::cerr << "CYLINDER: " << "robot arrived to: "  << "X: "<< Xpos << " Y: " << Ypos << endl;
        run_cloud = true;
    }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  cout << "ros init" << endl;
  ros::init(argc, argv, "cylinder_detect");
  cout << "ros NodeHandle  " << endl;
  ros::NodeHandle nh;
  //sound_play::SoundClient sc;


  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // subscriber za pozicijo robota
  goal_sub = nh.subscribe("goal_goal", 1000, parse_goal);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2> ("cylinder", 1);

  //pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder",1);
  pustr = nh.advertise<std_msgs::String>("notifications",1000);
  pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>("normals_marker_array", 1000);

  // Spin
  ros::spin ();
}

