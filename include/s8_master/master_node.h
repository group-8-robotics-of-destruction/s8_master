#ifndef __MASTER_NODE_H
#define __MASTER_NODE_H

namespace s8 {
    namespace master_node {
        const std::string NODE_NAME =                   "s8_master_node";

        const std::string TOPIC_OBJECT_TYPE =           "/s8/Classification/type";
        const std::string TOPIC_EXTRACTED_OBJECTS =     "/s8/modifiedObject";
        const std::string CONFIG_DOC =                  "catkin_ws/src/s8_master/parameters/parameters.json";
    }
}

#endif
