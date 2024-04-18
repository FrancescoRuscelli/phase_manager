#ifndef HORIZON_INTERFACE_H
#define HORIZON_INTERFACE_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <set>
#include <iostream>

// synthesis of what I need from the items in Horizon
class ItemBase
{
    /*
     * ItemBase is an interface used in PhaseManager that can host any item that has a 'setNodes()' method
     */
public:

    typedef std::shared_ptr<ItemBase> Ptr;
    ItemBase(): _is_changed(false) {}

    virtual bool setNodesInternal(std::vector<int> nodes, bool erasing) = 0;
    virtual std::string getName() = 0;
    virtual int getDim() = 0;
    virtual std::vector<int> getNodes() = 0;

    virtual bool reset() = 0;
    virtual bool update(std::vector<int> local_nodes, std::vector<int> absolute_nodes) = 0;

    bool isChanged()
    {
        return _is_changed;
    }

    bool setNodes(std::vector<int> nodes, bool erasing)
    {
        _is_changed = true;
        setNodesInternal(nodes, erasing);
        return true;
    }



protected:

    std::vector<int> _nodes;
    bool _is_changed;
};

// + bounds
class ItemWithBoundsBase : public ItemBase
{
public:

    typedef std::shared_ptr<ItemWithBoundsBase> Ptr;

    virtual bool setBoundsInternal(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes) = 0;

    bool clearBounds()
    {
        setBounds(_initial_lower_bounds, _initial_upper_bounds, getNodes());
        return true;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
    {
        _is_changed = true;
        setBoundsInternal(lower_bounds, upper_bounds, nodes);
        return true;
    }

protected:
    // do I need to initialize these?
    Eigen::MatrixXd _lower_bounds;
    Eigen::MatrixXd _upper_bounds;

    Eigen::MatrixXd _initial_lower_bounds;
    Eigen::MatrixXd _initial_upper_bounds;
};

class ItemWithValuesBase : public ItemBase
{
public:

    typedef std::shared_ptr<ItemWithValuesBase> Ptr;
    virtual bool assignInternal(Eigen::MatrixXd values, std::vector<int> nodes) = 0;
    virtual bool assignInternal(Eigen::MatrixXd values) = 0;


    bool clearValues()
    {
        assignInternal(_initial_values);
        return true;
    }

    bool assign(Eigen::MatrixXd values, std::vector<int> nodes)
    {
        _is_changed = true;
        assignInternal(values, nodes);
        return true;
    }

    bool assign(Eigen::MatrixXd values)
    {
        _is_changed = true;
        assignInternal(values);
        return true;
    }


protected:
    // do I need to initialize these?
    Eigen::MatrixXd _values;
    Eigen::MatrixXd _initial_values;
};


// model that calls the desired function from the original objects
template <typename T>
class Wrapper : public ItemBase
{
public:
    Wrapper(std::shared_ptr<T> item):
        m_item(item) {}

    bool setNodesInternal(std::vector<int> nodes) { return m_item->setNodes(nodes); }
    std::string getName() { return m_item->getName(); }
    int getDim() { return m_item->getDim(); }
    std::vector<int> getNodes() { return m_item->getNodes(); }

private:

    std::shared_ptr<T> m_item;
};


template <typename T>
class WrapperWithBounds : public ItemWithBoundsBase
{
public:
    WrapperWithBounds(std::shared_ptr<T> item):
        m_item(item)
        {
            // what if empty?
//            _lower_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());
//            _upper_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());

            _lower_bounds = std::get<0>(m_item->getBounds());
            _upper_bounds = std::get<1>(m_item->getBounds());

            _initial_lower_bounds =  _lower_bounds;
            _initial_upper_bounds =  _upper_bounds;
        }

    bool setNodesInternal(std::vector<int> nodes, bool erasing) { return m_item->setNodes(nodes, erasing); }
    bool setBoundsInternal(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes) { return m_item->setBounds(lower_bounds, upper_bounds, nodes); }
    int getDim() { return m_item->getDim(); }
    std::vector<int> getNodes() { return m_item->getNodes(); }

    std::string getName() { return m_item->getName(); }

private:

    std::shared_ptr<T> m_item;
};

template <typename T>
class WrapperWithValues : public ItemWithValuesBase
{
public:
    WrapperWithValues(std::shared_ptr<T> item):
        m_item(item)
        {
            // what if empty?
            _values = Eigen::MatrixXd::Zero(getDim(), getNodes().size());
            _initial_values = _values;
        }

    bool setNodesInternal(std::vector<int> nodes, bool erasing) { return m_item->setNodes(nodes, erasing); }

    bool assignInternal(Eigen::MatrixXd values) { return m_item->assign(values); }
    bool assignInternal(Eigen::MatrixXd values, std::vector<int> nodes) { return m_item->assign(values, nodes); }
    int getDim() { return m_item->getDim(); }
    std::vector<int> getNodes() { return m_item->getNodes(); }
    std::string getName() { return m_item->getName(); }

private:

    std::shared_ptr<T> m_item;
};




#endif
