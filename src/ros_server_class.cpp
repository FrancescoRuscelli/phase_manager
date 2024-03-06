#include <phase_manager/ros_server_class.h>
#include <phase_manager/Timeline.h>


using namespace HorizonPhases;


RosServerClass::RosServerClass(PhaseManager::Ptr pm):
    _pm(pm)

{
    _timelines = _pm->getTimelines();

    init_publishers();
}


void RosServerClass::init_publishers()
{

    // open publisher
    for (auto pair : _timelines)
    {
        _timelines_pub[pair.first] = _nh.advertise<phase_manager::Timeline>("phasemanager/" + pair.first, 10);
    }
}

void RosServerClass::run()
{

    for (auto pair : _timelines)
    {
        phase_manager::Timeline timeline_msg;

        timeline_msg.name = pair.first;
        auto phases = pair.second->getActivePhases();

        std::vector<std::string> phase_names;
        std::vector<int> phase_initial_nodes;
        std::vector<int> phase_durations;

        for (auto phase : phases)
        {
            phase_names.push_back(phase->getName());
            phase_initial_nodes.push_back(phase->getPosition());
            phase_durations.push_back(phase->getNNodes());
        }

        timeline_msg.phases = phase_names;
        timeline_msg.initial_nodes = phase_initial_nodes;
        timeline_msg.durations = phase_durations;

        _timelines_pub[pair.first].publish(timeline_msg);

    }

}
