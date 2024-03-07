#ifndef ROS_SERVER_CLASS_H
#define ROS_SERVER_CLASS_H

#include <phase_manager/phase_manager.h>
#include <ros/ros.h>

namespace HorizonPhases {

    class RosServerClass
    {
        public:

            typedef std::shared_ptr<RosServerClass> Ptr;
            RosServerClass(PhaseManager::Ptr pm);
            void run();


        private:

            void init_publishers();

            std::unique_ptr<ros::NodeHandle> _nh;
            PhaseManager::Ptr _pm;

            std::unordered_map<std::string, SinglePhaseManager::Ptr> _timelines;

            ros::Publisher _timelines_pub;

    };

}
#endif // ROS_SERVER_CLASS_H
