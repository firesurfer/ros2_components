
#ifndef ENTITYCIRCLEEXCEPTION
#define ENTITYCIRCLEEXCEPTION

#include <exception>
#include <string>

class EntityCircleException: public std::exception
{
public:
    explicit EntityCircleException(const std::string& msg = "An Entity circle has been created");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //ENTITYCIRCLEEXCEPTION
