#ifndef ENTITYNOTREGISTEREDEXCEPTION
#define ENTITYNOTREGISTEREDEXCEPTION

#include <exception>

class EntityNotRegisteredException: public std::exception
{
public:
    EntityNotRegisteredException();

    virtual const char* what() const throw()
    {
        return "Entity is not registered to the EntityFactory";
    }
};

#endif //ENTITYNOTREGISTEREDEXCEPTION
