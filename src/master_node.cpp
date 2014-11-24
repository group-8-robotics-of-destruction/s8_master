#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_msgs/Classification.h>
#include <s8_master/master_node.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_object_aligner/ObjectAlignAction.h>
#include <s8_explorer/ExploreAction.h>
#include <s8_msgs/DistPose.h>


// OTHER
#include <vector>
#include <signal.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <algorithm>    // std::min_element, std::max_element


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

using namespace std;
using namespace s8;
using namespace s8::master_node;

class NodeMaster: public Node
{
    const int hz;

    actionlib::SimpleActionClient<s8_object_aligner::ObjectAlignAction> object_align_action;
    actionlib::SimpleActionClient<s8_explorer::ExploreAction> explore_action;

    ros::Subscriber object_type_subscriber;
    ros::Subscriber object_dist_pose_subscriber;
    ros::Publisher point_cloud_publisher;
    s8_msgs::Classification classType;

    bool isClassTypeInitialized;
    int count[11];
    int j;

    float circle_red_low_H;

    bool exploring;
    bool classify;
    bool aligned;

    int doing_nothing_count;
    int object_detected_in_row_count;
public:
    NodeMaster(int hz) : hz(hz), object_align_action(ACTION_OBJECT_ALIGN, true), explore_action(ACTION_EXPLORE, true), exploring(false), classify(false), aligned(false), doing_nothing_count(0), object_detected_in_row_count(0)
    {
        add_params();
        //printParams();
        object_type_subscriber = nh.subscribe(TOPIC_OBJECT_TYPE, BUFFER_SIZE, &NodeMaster::object_type_callback, this);
        object_dist_pose_subscriber = nh.subscribe(TOPIC_OBJECT_DIST_POSE, 1, &NodeMaster::object_dist_pose_callback, this);

        isClassTypeInitialized = false;
        std::fill(count,count+11,0);
        j = 0;

        ROS_INFO("Waiting for explorer action server...");
        explore_action.waitForServer();
        ROS_INFO("Connected to explorer server!");

        ROS_INFO("Waiting for object align action server...");
        object_align_action.waitForServer();
        ROS_INFO("Connected to object align server!");

        start_explore();
    }

    void updateClass()
    {
        if(!classify) {
            if(!exploring) {
                doing_nothing_count++;
                ROS_INFO("Doing nothing");

                if(doing_nothing_count > 10) {
                    ROS_INFO("Enough doing nothing. Exploring!");
                    start_explore();
                }
            } else {
                doing_nothing_count = 0;
            }

            return;
        }

        doing_nothing_count = 0;

        ROS_INFO("Classifying...");

        if (!isClassTypeInitialized)
        {
            //ROS_INFO("Not Initalized!");
            return;
        }
        if (j == 50)
        {
            int idx = idxOfMax(count);
            string name = typeFromInt(idx);
            ROS_INFO("number: %d, name: %s", idx, name.c_str());
            ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",count[0], count[1],count[2],count[3],count[4],count[5],count[6],count[7],count[8],count[9],count[10]);
            std::fill(count,count+11,0);
            j = 0;

            //Done classifying! Continue exploring. TODO: Make sure it doesnt hit the object.
            start_explore();
            return;
        }

        int type = classType.type;
        count[type]++;

        //ROS_INFO("%d, %s", classType.type, classType.name.c_str());
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
        s8_explorer::ExploreGoal goal;
        goal.explore = true;
        explore_action.sendGoal(goal, boost::bind(&NodeMaster::explore_done_callback, this, _1, _2), actionlib::SimpleActionClient<s8_explorer::ExploreAction>::SimpleActiveCallback(), actionlib::SimpleActionClient<s8_explorer::ExploreAction>::SimpleFeedbackCallback());
    }

    void explore_done_callback(const actionlib::SimpleClientGoalState& state, const s8_explorer::ExploreResultConstPtr & result) {
        ROS_INFO("STOPPED: Exploring stopped");
        exploring = false;
    }

    void object_dist_pose_callback(const s8_msgs::DistPose::ConstPtr & dist_pose) {
        if(dist_pose->dist > 0) {
            object_detected_in_row_count++;

            if(object_detected_in_row_count > 5) {
                if(exploring) {
                    //There is an object. Time to stop exploring and do object aligning.
                    ROS_INFO("Object detected!");
                    stop_exploring();
                } else {
                    if(!aligned) {
                        align();
                        object_detected_in_row_count = 0;
                    }
                }
            }
        }
    }

    int idxOfMax(int count[])
    {
        int maxSize = 0;
        int idx = 0;
        for (int k = 0; k < 11; k++)
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
                return "Red Circle";
            case(2):
                return "Yellow Circle";
            case(3):
                return "Red Cube";
            case(4):
                return "Yellow Cube";
            case(5):
                return "Green Cube";
            case(6):
                return "Blue Cube";
            case(7):
                return "Orange Star";
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
