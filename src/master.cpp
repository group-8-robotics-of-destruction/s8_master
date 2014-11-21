#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_msgs/Classification.h>

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

#define NODE_NAME           		  "s8_master"
#define TOPIC_OBJECT_TYPE   		  "/s8/Classification/type"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/modifiedObject"
#define CONFIG_DOC                "catkin_ws/src/s8_master/parameters/parameters.json"


using namespace std;

class NodeMaster: public s8::Node
{
    const int hz;

    ros::Subscriber object_type_subscriber;
    ros::Publisher point_cloud_publisher;
    s8_msgs::Classification classType;

    bool isClassTypeInitialized;
    int count[11];
    int j;

    // PARAMETERS
    int number_of_checks;
    float no_object_percentage;

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
    ~NodeMaster()
    {
    }

    void updateClass()
    {
        if (!isClassTypeInitialized)
        {
            ROS_INFO("Not Initalized!");
            return;
        }
        if (j == number_of_checks)
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

        // Check to see if there really is an object
        if (count[0]> no_object_percentage * number_of_checks)
            return 0;

        // If there is an object, loop through them to find the one
        // that occurs the most often
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
        // CLASSIFICATION
        no_object_percentage    = pt.get<float>("no_object_percentage");
        number_of_checks        = pt.get<int>("number_of_checks");
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
