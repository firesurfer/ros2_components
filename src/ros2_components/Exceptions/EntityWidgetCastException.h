#ifndef ENTITYWIDGETCASTEXCEPTION
#define ENTITYWIDGETCASTEXCEPTION

#include <exception>
#include <string>

class EntityWidgetCastException: public std::exception
{
public:
    explicit EntityWidgetCastException(const std::string& msg = "Could not cast entity widget to given type");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //ENTITYWIDGETCASTEXCEPTION
