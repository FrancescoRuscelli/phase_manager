#ifndef ROS_SERVER_CLASS_H
#define ROS_SERVER_CLASS_H

#include <phase_manager/phase_manager.h>
#include <ros/ros.h>

namespace HorizonPhases {

    class RosServerClass
    {
        public:

            RosServerClass(PhaseManager::Ptr pm);
            void run();


        private:

            void init_publishers();

            ros::NodeHandle _nh;
            PhaseManager::Ptr _pm;

            std::unordered_map<std::string, SinglePhaseManager::Ptr> _timelines;

            std::unordered_map<std::string, ros::Publisher> _timelines_pub;

    };

}
#endif // ROS_SERVER_CLASS_H
