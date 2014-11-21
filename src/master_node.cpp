#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_msgs/Classification.h>
#include <s8_master/master_node.h>

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

    ros::Subscriber object_type_subscriber;
    ros::Publisher point_cloud_publisher;
    s8_msgs::Classification classType;

    bool isClassTypeInitialized;
    int count[11];
    int j;

    float circle_red_low_H;

public:
    NodeMaster(int hz) : hz(hz)
    {
        add_params();
        //printParams();
        object_type_subscriber = nh.subscribe(TOPIC_OBJECT_TYPE, BUFFER_SIZE, &NodeMaster::object_type_callback, this);

        isClassTypeInitialized = false;
        std::fill(count,count+11,0);
        j = 0;
    }

    void updateClass()
    {
        if (!isClassTypeInitialized)
        {
            ROS_INFO("Not Initalized!");
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
            return;
        }

        int type = classType.type;
        count[type]++;

        //ROS_INFO("%d, %s", classType.type, classType.name.c_str());
        //isClassTypeInitialized = false;
        j++;
    }

private:

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
