#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>

#include <sound_play/sound_play.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0.05;
int size_y;
int size_x;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;


void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map) {
    cout << "mouse call back was called" << endl;
    size_x = msg_map->info.width;
    size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3)) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }


    // resize cv image if it doesn't have the same dimensions as the map
    if ((cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    cout << map_resolution << endl;
    tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector <int8_t> &map_msg_data(msg_map->data);

    unsigned char *cv_map_data = (unsigned char *) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y - 1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y - y);
        for (int y = 0; y < size_y; ++y) {

            int idx_map_y = size_x * (size_y - y);
            //int idx_map_y = size_x * y;
            int idx_img_y = size_x * y;

            for (int x = 0; x < size_x; ++x) {

                int idx = idx_img_y + x;

                switch (map_msg_data[idx_map_y + x]) {
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
}

float transfx(int point){
    return point * map_resolution;
}

float transfy(int point){
    return point * -map_resolution;
}



void walkPath(float points[][2], int len, MoveBaseClient &ac){
    float quatw[6] = {0.866, 0.500, 0.000,-0.500, 0.500, 0.866};
    float quatz[6] = {0.500, 0.866, 1.000, 0.866,-0.866,-0.500};
    float pointx, pointy;
    for(int i = 0; i < len; i++){        
        pointx = points[i][0];
        pointy = points[i][1];
        //cout << "transformed point: " << transformed.x() << " " << transformed.y() << endl;
        cout << "point: " << pointx << " " << pointy << endl;
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        //goal.target_pose.pose.position.x = transformed.x();
        //goal.target_pose.pose.position.y = transformed.y();
        goal.target_pose.pose.position.x = pointx;
        goal.target_pose.pose.position.y = pointy;
        goal.target_pose.pose.orientation.w = 1.0;
        //ROS_INFO("Sending goal");
        cout << "point: " << pointx << " " << pointy << endl;
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            //ROS_INFO("moved to next point");
        }
        else{
            //ROS_INFO("The base failed to move");
        }

        for(int j = 0 ; j < 6 ; j++){
            //ROS_INFO("Sending rotation");
            move_base_msgs::MoveBaseGoal goal1;
            goal1.target_pose.header.frame_id = "map";
            goal1.target_pose.header.stamp = ros::Time::now();
            goal1.target_pose.pose.position.x = pointx;
            goal1.target_pose.pose.position.y = pointy;
            goal1.target_pose.pose.orientation.w = quatw[j];
            goal1.target_pose.pose.orientation.z = quatz[j];
            ac.sendGoal(goal1);
            ac.waitForResult();
            system("python /home/team_beta/ROS/src/pr1/src/original_detect_markers.py");
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                //ROS_INFO("rotated");
                cout <<"point: " << pointx << " " << pointy << "     rotated: " << j << endl;
            }
            else{
                ROS_INFO("failed to rotate");
            }
        }
    }
}


void say(String text){

    sound_play::SoundClient sc;

    sc.say(text);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "navi");

    ros::NodeHandle n;
    map_sub = n.subscribe("map", 10, &mapCallback);
    //say("Hello testing");
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    int len = 12;
    float points[len][2] = {
        {0, 0},
        {0.44, 0.41},
        {0.26, 1.13},
        {0.72, 1.80},
        {0.35, 2.82},
        {1.19, 4.28},
        {1.34, 3.60},
        {1.37, 2.83},
        {1.44, 1.89},
        {1.55, 0.85},
        {1.46, 0.23},
        {0.85, 0.27}
    };
    while(ros::ok()){
        ros::spinOnce();
        walkPath(points, len, ac);
    }

    walkPath(points, len, ac);
    return 0;
}

