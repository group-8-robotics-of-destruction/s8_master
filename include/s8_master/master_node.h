#ifndef __MASTER_NODE_H
#define __MASTER_NODE_H

#include <s8_object_aligner/object_aligner_node.h>
#include <s8_explorer/explorer_node.h>

namespace s8 {
    namespace master_node {
        const std::string NODE_NAME                 =   "s8_master_node";
        const std::string TOPIC_OBJECT_TYPE         =   "/s8/Classification/type";
        const std::string TOPIC_EXTRACTED_OBJECTS   =   "/s8/modifiedObject";
        const std::string TOPIC_OBJECT_DIST_POSE    =   "/s8/ip/classification/distPose";
        const std::string TOPIC_ESPEAK	            =   "/espeak/string";
        const std::string TOPIC_EVIDENCE            =   "/evidence";
        const std::string TOPIC_RGB_IMAGE           =   "/camera/rgb/image_rect_color";
        const std::string CONFIG_DOC                =   "/home/ras/catkin_ws/src/s8_master/parameters/parameters.json";

        const std::string ACTION_OBJECT_ALIGN =         s8::object_aligner_node::ACTION_OBJECT_ALIGN;
        const std::string ACTION_EXPLORE =              s8::explorer_node::ACTION_EXPLORE;

    }
}

#endif
