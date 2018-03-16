#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }

}

void mouseCallback(int event, int x, int y, int, void* data) {

    if( event != EVENT_LBUTTONDOWN || cv_map.empty())
        return;

    int v = (int)cv_map.at<unsigned char>(y, x);

	if (v != 255) {
		ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
		return;
	}

    ROS_INFO("Moving to (x: %d, y: %d)", x, y);

	tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
	tf::Point transformed = map_transform * pt;

	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "map";
	goal.pose.orientation.w = 1;
	goal.pose.position.x = transformed.x();
	goal.pose.position.y = -transformed.y();
	goal.header.stamp = ros::Time::now();

	goal_pub.publish(goal);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);

    namedWindow("Map");

    setMouseCallback("Map", mouseCallback, NULL);

    while(ros::ok()) {

        if (!cv_map.empty()) imshow("Map", cv_map);

        waitKey(30);

        ros::spinOnce();
    }
    return 0;

}
