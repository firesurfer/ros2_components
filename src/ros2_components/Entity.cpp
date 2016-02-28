#include "Entity.h"
namespace ros2_components {

EntityBase::EntityBase(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> _parentNode, string _className)
{
    this->id = _id;
    this->subscriber = _subscribe;
    this->parentNode = _parentNode;
    this->className = _className;
    this->active = true;
    REFLECT(id);
    REFLECT(virtualEntity);
    REFLECT(className);
    REFLECT(active)
}

int64_t EntityBase::getId()
{
    return id;
}

string EntityBase::getName()
{
    return getClassName()  + std::to_string(id);
}

string EntityBase::getClassName()
{
    const QMetaObject* metaObject = this->metaObject();

    std::string localClassName = metaObject->className();
    localClassName.erase(0, localClassName.find_last_of(":")+1);
    // std::cout << "Local: " << localClassName << std::endl;
    //std::cout << "test: " << this->objectName().toStdString() << std::endl;
    return className;
}

void EntityBase::addChild(std::shared_ptr<EntityBase> child)
{
    childs.push_back(child);
    child->setParent(std::shared_ptr<EntityBase>(this));
    emit childAdded(child);
}

std::shared_ptr<EntityBase> EntityBase::getChild(uint64_t index)
{
    if(childs.size() < index)
        throw std::runtime_error("Index out of bounds");
    return childs[index];
}

std::shared_ptr<EntityBase> EntityBase::getChildById(int64_t id)
{
    for(auto & child: childs)
    {
        if(child->getId() == id)
            return child;
    }
    throw std::runtime_error("Child with id: " + std::to_string(id) + " not found");
}

uint64_t EntityBase::countChilds()
{

    return childs.size();
}

std::shared_ptr<EntityBase> EntityBase::getParent()
{
    return this->parent;
}

void EntityBase::setParent(std::shared_ptr<EntityBase> par)
{
    this->parent = par;
}

string EntityBase::getTopicName()
{
    if(!isSubscriber())
    {
        return pubBase->get_topic_name();
    }
    else
        return subBase->get_topic_name();
}

void EntityBase::updateParameters()
{

    std::cout << "Updating Parameters" << std::endl;
    std::vector<std::string> myParameters;
    for(auto & par: internalmap)
    {
        myParameters.push_back(getName()+ "."+par->key);
        //std::cout << "Key: " << par->key<<std::endl;
    }
    auto parameter_list_future = parameterClient->list_parameters(myParameters, 10);
    if (rclcpp::spin_until_future_complete(parentNode, parameter_list_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        std::cout <<"list_parameters service call failed"<< std::endl;
        return;
    }
    auto parameter_list = parameter_list_future.get();
    for (auto & name : parameter_list.names) {
        std::string reducedParameter = name;
        reducedParameter.erase(0,reducedParameter.find_last_of(".")+1);

        // std::cout << "Name: " << name << " Reduced: " << reducedParameter << std::endl;

        if(std::find(myParameters.begin(), myParameters.end(), name) != myParameters.end() )
        {
            //std::cout << "Found it!" << std::endl;
            auto parameters = parameterClient->get_parameters({name});
            if (rclcpp::spin_until_future_complete(parentNode, parameters) !=
                    rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                printf("get_parameters service call failed. Exiting example.\n");

            }
            for (auto & parameter : parameters.get()) {
                // std::cout << "Parameter name: " << parameter.get_name() << std::endl;
                // std::cout << "Parameter value (" << parameter.get_type_name() << "): " <<
                //                             parameter.value_to_string() << std::endl;

                for (auto & internal_val : internalmap)
                {
                    if(internal_val->key == reducedParameter)
                    {


                        /*
                 * uint8 PARAMETER_NOT_SET=0
                 * uint8 PARAMETER_BOOL=1
                 * uint8 PARAMETER_INTEGER=2
                 * uint8 PARAMETER_DOUBLE=3
                 * uint8 PARAMETER_STRING=4
                 * uint8 PARAMETER_BYTES=5
                 */

                        switch(parameter.get_type())
                        {
                        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                        {
                            SpecificElement<bool>* elem = static_cast<SpecificElement<bool>*>(internal_val);
                            elem->setValue(parameter.as_bool());
                            break;
                        }
                        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                        {
                            SpecificElement<int64_t>* elem = static_cast<SpecificElement<int64_t>*>(internal_val);
                            elem->setValue(parameter.as_int());
                            break;
                        }
                        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                        {
                            SpecificElement<double>* elem = static_cast<SpecificElement<double>*>(internal_val);
                            elem->setValue(parameter.as_double());
                            break;
                        }
                        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                        {
                            SpecificElement<std::string>* elem = static_cast<SpecificElement<std::string>*>(internal_val);
                            elem->setValue(parameter.as_string());
                            break;
                        }
                        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTES:
                        {
                            SpecificElement<std::vector<uint8_t>>* elem = static_cast<SpecificElement<std::vector<uint8_t>>*>(internal_val);
                            elem->setValue(parameter.as_bytes());
                            break;
                        }
                        case rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET:
                        {
                            break;
                        }

                        }

                    }
                }
            }
        }
    }
    emit parametersUpdated();
}

void EntityBase::publishMetaInformation()
{
    std::cout << "Publish meta information in: " <<  getName() << std::endl;
    std::vector< rclcpp::parameter::ParameterVariant> params;

    for(auto it = internalmap.begin(); it != internalmap.end(); it++)
    {
        params.push_back((*it)->getParameterVariant(getName()));
        //(*it)->print();
    }
    auto set_parameters_results = this->parameterClient->set_parameters(params);
    rclcpp::spin_until_future_complete(this->parentNode, set_parameters_results);
}

string EntityBase::getAutogeneratedClassName()
{
    int status;
    char * demangled = abi::__cxa_demangle(typeid(*this).name(),0,0,&status);
    std::string tempname =  std::string(demangled);
    if(tempname.find_last_of(":") != std::string::npos)
        tempname.erase(0, tempname.find_last_of(":")+1);

    return tempname;
}

void EntityBase::IterateThroughAllProperties(std::function<void(QMetaProperty)> func)
{
    const QMetaObject* metaObj = this->metaObject();
    while(metaObj != NULL)
    {
        for (int i = metaObj->propertyOffset(); i < metaObj->propertyCount(); ++i)
        {
            func(metaObj->property(i));
        }
        metaObj = metaObj->superClass();
    }
}








}