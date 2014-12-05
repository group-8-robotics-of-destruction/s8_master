#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_msgs/Classification.h>
#include <s8_master/master_node.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_object_aligner/ObjectAlignAction.h>
#include <s8_explorer/ExploreAction.h>
#include <s8_msgs/DistPose.h>
#include <std_msgs/String.h>
#include <ras_msgs/RAS_Evidence.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <s8_mapper/PlaceNode.h>
#include <s8_mapper/mapper_node.h>


// OTHER
#include <vector>
#include <signal.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <algorithm>    // std::min_element, std::max_element

#define TOPO_NODE_FREE              1 << 1
#define TOPO_NODE_OBJECT            1 << 3
#define TOPO_NODE_OBJECT_VIEWER     1 << 5


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

using namespace std;
using namespace s8;
using namespace s8::master_node;
using s8::mapper_node::SERVICE_PLACE_NODE;
using s8::explorer_node::ExploreFinishedReason;

class NodeMaster: public Node
{
    const int hz;

    actionlib::SimpleActionClient<s8_object_aligner::ObjectAlignAction> object_align_action;
    actionlib::SimpleActionClient<s8_explorer::ExploreAction> explore_action;

    ros::Subscriber object_type_subscriber;
    ros::Subscriber object_dist_pose_subscriber;
    ros::Subscriber rgb_image_subscriber;
    ros::Publisher point_cloud_publisher;
    ros::Publisher espeak_publisher;
    ros::Publisher evidence_publisher;
    s8_msgs::Classification classType;
    ros::Time classify_time;
    ros::ServiceClient place_node_client;


    bool isClassTypeInitialized;
    int count[11];
    int j;

    float circle_red_low_H;

    bool exploring;
    bool classify;
    bool aligned;
    bool navigating;

    int doing_nothing_count;
    int object_detected_in_row_count;

    double object_distance;
    double object_angle;

    sensor_msgs::ImageConstPtr rgb_image;
public:
    NodeMaster(int hz) : hz(hz), object_align_action(ACTION_OBJECT_ALIGN, true), explore_action(ACTION_EXPLORE, true), navigating(false), exploring(false), classify(false), aligned(false), doing_nothing_count(0), object_detected_in_row_count(0)
    {
        add_params();
        //printParams();
        object_type_subscriber = nh.subscribe(TOPIC_OBJECT_TYPE, BUFFER_SIZE, &NodeMaster::object_type_callback, this);
        object_dist_pose_subscriber = nh.subscribe(TOPIC_OBJECT_DIST_POSE, 1, &NodeMaster::object_dist_pose_callback, this);
        rgb_image_subscriber   = nh.subscribe(TOPIC_RGB_IMAGE, 1, &NodeMaster::rgb_image_callback, this);
        espeak_publisher  = nh.advertise<std_msgs::String>(TOPIC_ESPEAK, 1);
        evidence_publisher  = nh.advertise<ras_msgs::RAS_Evidence>(TOPIC_EVIDENCE, 1);
        classify_time = ros::Time::now();

        isClassTypeInitialized = false;
        std::fill(count,count+11,0);
        j = 0;

        place_node_client = nh.serviceClient<s8_mapper::PlaceNode>(SERVICE_PLACE_NODE, true);

        ROS_INFO("Waiting for explorer action server...");
        explore_action.waitForServer();
        ROS_INFO("Connected to explorer server!");

        ROS_INFO("Waiting for object align action server...");
        object_align_action.waitForServer();
        ROS_INFO("Connected to object align server!");

        start_explore();
    }

    void place_node(double x, double y, double dist, double theta, int value) {
        s8_mapper::PlaceNode pn;
        pn.request.x = x;
        pn.request.y = y;
        pn.request.dist = dist;
        pn.request.theta = theta;
        pn.request.value = value;
        pn.request.isWallForward = false;
        pn.request.isWallLeft = false;
        pn.request.isWallRight = false;
        if(!place_node_client.call(pn)) {
            ROS_FATAL("Failed to call place node.");
        }
    }

    void updateClass()
    {
        static double x_sum;
        static double y_sum;
        static double dist_sum;
        static double theta_sum;
        static int cnt;

        if(!classify) {
            if(!exploring) {
                if(!navigating) {
                    doing_nothing_count++;
                    ROS_INFO("Doing nothing");

                    if(doing_nothing_count > 10) {
                        ROS_INFO("Enough doing nothing. Exploring!");
                        start_explore();
                    }
                } else {
                    doing_nothing_count = 0;
                }
            }

            x_sum = 0;
            y_sum = 0;
            dist_sum = 0;
            theta_sum = 0;
            cnt = 0;
            return;
        }

        if(object_distance > 0) {
            ROS_INFO("distance: %lf, angle: %lf", object_distance, object_angle);
            double y = object_distance * std::cos(object_angle) - 0.06;
            double x = object_distance * std::sin(object_angle);

            x_sum += x;
            y_sum += y;
            dist_sum += object_distance;
            theta_sum += object_angle;
            cnt++;
        }

        doing_nothing_count = 0;

        //ROS_INFO("Classifying...");

        if (!isClassTypeInitialized)
        {
            //ROS_INFO("Not Initalized!");
            return;
        }
        if (j == 30)
        {
            int idx = 0;
            string name;
            if (count[0] < 0.8*30){
                idx = idxOfMax(count);
                name = typeFromInt(idx);
                ROS_INFO("number: %d, name: %s", idx, name.c_str());
                ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",count[0], count[1],count[2],count[3],count[4],count[5],count[6],count[7],count[8],count[9],count[10]);
            }
            j = 0;
            std::fill(count,count+11,0);

            if(idx != 0)
            {
                espeakPublish(name.c_str());
                evidencePublish(name.c_str());
                evidencePublish(name.c_str());
                evidencePublish(name.c_str());
                evidencePublish(name.c_str());
                ROS_INFO("Publishing");

                if(cnt > 0) {
                    double x = x_sum / cnt;
                    double y = y_sum / cnt;
                    double theta = theta_sum / cnt;
                    double dist = dist_sum / cnt;
                    
                    ROS_INFO("Object at (%lf, %lf) value: %d", x, y, TOPO_NODE_OBJECT);
                    place_node(0, 0, 0, 0, TOPO_NODE_OBJECT_VIEWER);
                    place_node(x, y, dist, theta, TOPO_NODE_OBJECT);
                }
            } else {
                ROS_WARN("Failed to classify");
            }
            //Done classifying! Continue exploring. TODO: Make sure it doesnt hit the object.
            classify_time = ros::Time::now();
            start_explore();
            classify = false;
            return;
        }

        int type = classType.type;
        count[type]++;

        ROS_INFO("%d, %s", classType.type, classType.name.c_str());
        ROS_INFO("number of objects seen:%d", j);
        //isClassTypeInitialized = false;
        j++;
    }

private:
    void align() {
        ROS_INFO("Object aligning....");
        s8_object_aligner::ObjectAlignGoal goal;
        goal.align = true;
        object_align_action.sendGoal(goal);

        bool finised_before_timeout = object_align_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = object_align_action.getState();
            ROS_INFO("Object align action finished. %s", state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                classify = true;
                aligned = true;
            } else {
                classify = false;
                aligned = false;
            }

        } else {
            ROS_WARN("Object align action timed out.");
        }
    }

    void stop_exploring() {
        if(exploring) {
            ROS_INFO("STOPPING: Exploring is stopping...");
            explore_action.cancelGoal();
        }
    }

    void start_explore() {
        ROS_INFO("START: Starting exploring");
        exploring = true;
        aligned = false;
        classify = false;
        navigating = false;
        s8_explorer::ExploreGoal goal;
        goal.explore = true;
        explore_action.sendGoal(goal, boost::bind(&NodeMaster::explore_done_callback, this, _1, _2), actionlib::SimpleActionClient<s8_explorer::ExploreAction>::SimpleActiveCallback(), actionlib::SimpleActionClient<s8_explorer::ExploreAction>::SimpleFeedbackCallback());
    }

    void explore_done_callback(const actionlib::SimpleClientGoalState& state, const s8_explorer::ExploreResultConstPtr & result) {
        ROS_INFO("STOPPED: Exploring stopped");
        exploring = false;

        if(result->reason == ExploreFinishedReason::REVISITED) {
            ROS_INFO("Explorer revisited node. Need to check with map.");
            navigating = true;
            ROS_INFO("Navigating");
            //TODO: Do me.
        }
    }

    void object_dist_pose_callback(const s8_msgs::DistPose::ConstPtr & dist_pose) {
        object_distance = dist_pose->dist;
        object_angle = dist_pose->pose;

        if(dist_pose->dist > 0) {
            ros::Time current_time = ros::Time::now();
            object_detected_in_row_count++;

            if(object_detected_in_row_count > 1 && (current_time-classify_time).toSec() > 5) {
                if(exploring) {
                    //There is an object. Time to stop exploring and do object aligning.
                    stop_exploring();
                    ROS_INFO("Object detected!");
                    classify = true;
                    aligned = false;
                    object_detected_in_row_count = 0;
                } else {
                    if(!aligned) {
                        classify = true;
                        aligned = true;
                        //align();
                        object_detected_in_row_count = 0;
                    }
                }
            }
            else if((current_time - classify_time).toSec() < 5)
                object_detected_in_row_count = 0;
        }
    }

    int idxOfMax(int count[])
    {
        int maxSize = 0;
        int idx = 0;
        for (int k = 1; k < 11; k++)
        {
            if (count[k] > maxSize)
            {
                maxSize = count[k];
                idx = k;
            }
        }
        return idx;
    }

    string typeFromInt(int idx)
    {
        switch(idx)
        {
            case(0):
                return "No Object";
            case(1):
                return "Red Ball";
            case(2):
                return "Yellow Ball";
            case(3):
                return "Red Cube";
            case(4):
                return "Yellow Cube";
            case(5):
                return "Green Cube";
            case(6):
                return "Blue Cube";
            case(7):
                return "Patric";
            case(8):
                return "Purple Cross";
            case(9):
                return "Green Cylinder";
            case(10):
                return "Blue triangle";
        }
    }

    void object_type_callback(const s8_msgs::Classification classType_msg)
    {
        classType = classType_msg;
        isClassTypeInitialized = true;
    }


    void espeakPublish(string espeak)
    {
        std_msgs::String text;
        text.data = espeak;
        espeak_publisher.publish(text);
    }

    void evidencePublish(string type)
    {
        ras_msgs::RAS_Evidence evidence;
        evidence.stamp = ros::Time::now();
        evidence.group_number = 8;
        evidence.object_id = type;
        evidence.image_evidence = *rgb_image;
        evidence_publisher.publish(evidence);
    }

    void rgb_image_callback(const sensor_msgs::ImageConstPtr& msgColor)
    {
        rgb_image = msgColor;
    }
  
    void add_params()
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(CONFIG_DOC, pt);
        // CIRCLE
        circle_red_low_H = pt.get<float>("circle_red_low_H");
    }
};

int main(int argc, char **argv) {

    std::cout << argv[0] << std::endl;
    ros::init(argc, argv, NODE_NAME);

    NodeMaster master(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        ros::spinOnce();
        master.updateClass();
        loop_rate.sleep();
    }

    return 0;
}
