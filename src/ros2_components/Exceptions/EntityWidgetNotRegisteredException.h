#ifndef ENTITYWIDGETNOTREGISTEREDEXCEPTION
#define ENTITYWIDGETNOTREGISTEREDEXCEPTION

#include <exception>
#include <string>

class EntityWidgetNotRegisteredException: public std::exception
{
public:
    explicit EntityWidgetNotRegisteredException(const std::string& msg = "EntityWidget is not registered to EntityWidgetFactory");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //ENTITYWIDGETNOTREGISTEREDEXCEPTION
