#ifndef ENTITYCASTEXCEPTION
#define ENTITYCASTEXCEPTION

#include <exception>
#include <string>

class EntityCastException: public std::exception
{
public:
    explicit EntityCastException(const std::string& msg = "Could not cast entity to given type");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //ENTITYCASTEXCEPTION
