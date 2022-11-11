#include <phase_manager/horizon_manager.h>

bool HorizonManager::addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint)
{
    // should push pointer
    _constraints.insert(constraint);
}

bool HorizonManager::addVariable(ItemWithBoundsBase::ItemWithBoundsBasePtr variable)
{
//    std::cout << "name:" << variable->getName() << std::endl;
    std::cout << "before size" << _variables.size() << std::endl;
    _variables.insert(variable);
    std::cout << "after size" << _variables.size() << std::endl;
}

bool HorizonManager::flush()
{
    for (auto constraint : _constraints)
    {
        constraint->flush();
    }

    for (auto variables : _variables)
    {
        variables->flush();
    }

}

bool HorizonManager::reset()
{
    for (auto constraint : _constraints)
    {
        constraint->clearNodes();
    }

    for (auto variable : _variables)
    {
        variable->clearNodes();
    }

}
