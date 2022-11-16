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
public:

    typedef std::shared_ptr<ItemBase> Ptr;

    virtual bool setNodes(std::vector<int> nodes) = 0;
    virtual std::string getName() = 0;
    virtual int getDim() = 0;
    virtual std::vector<int> getNodes() = 0;

    bool addNodes(std::vector<int> nodes)
    {
        _nodes.insert(_nodes.end(), nodes.begin(), nodes.end());
        return true;
    }
    bool flushNodes()
    {
        _nodes.erase(std::unique(_nodes.begin(), _nodes.end()), _nodes.end());
        setNodes(_nodes);

        return true;
    }
    bool clearNodes()
    {
        _nodes.clear();
        return true;
    }

protected:
    std::vector<int> _nodes;
};

//// + bounds
class ItemWithBoundsBase : public ItemBase
{
public:

    typedef std::shared_ptr<ItemWithBoundsBase> Ptr;

    virtual bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds) = 0;


    bool addBounds(std::vector<int> nodes, Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
//        std::cout << "setting bounds to nodes: ";
//        for (auto node : nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

        _lower_bounds(Eigen::indexing::all, nodes) << lower_bounds;
        _upper_bounds(Eigen::indexing::all, nodes) << upper_bounds;

        _nodes.insert(_nodes.end(), nodes.begin(), nodes.end());

        return true;

    }

    bool flushNodes()
    {
        ItemBase::flushNodes();
        return true;
    }

    bool flushBounds()
    {
//        std::cout << _lower_bounds << std::endl;
        setBounds(_lower_bounds, _upper_bounds);
        return true;
    }


    bool clearBounds()
    {
        _lower_bounds = _initial_lower_bounds;
        _upper_bounds = _initial_upper_bounds;

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
    virtual bool assign(Eigen::MatrixXd values) = 0;


    bool addValues(std::vector<int> nodes, Eigen::MatrixXd values)
    {
//        std::cout << "setting bounds to nodes: ";
//        for (auto node : nodes)
//        {
//            std::cout << node << " ";
//        }
        _values(Eigen::indexing::all, nodes) << values;

        _nodes.insert(_nodes.end(), nodes.begin(), nodes.end());

        return true;

    }

    bool flushNodes()
    {
        ItemBase::flushNodes();
        return true;
    }

    bool flushValues()
    {
        assign(_values);
        return true;
    }

    bool clearValues()
    {
        _values = _initial_values;
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

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
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
            _lower_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());
            _upper_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());

            _initial_lower_bounds =  _lower_bounds;
            _initial_upper_bounds =  _upper_bounds;
        }

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds) { return m_item->setBounds(lower_bounds, upper_bounds); }
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

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
    bool assign(Eigen::MatrixXd values) { return m_item->assign(values); }
    int getDim() { return m_item->getDim(); }
    std::vector<int> getNodes() { return m_item->getNodes(); }
    std::string getName() { return m_item->getName(); }

private:

    std::shared_ptr<T> m_item;
};

#endif
