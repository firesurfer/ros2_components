#ifndef ENTITYCASTEXCEPTION
#define ENTITYCASTEXCEPTION

#include <exception>

class EntityCastException: public std::exception
{
public:
    EntityCastException();

    virtual const char* what() const throw()
    {
        return "Could not cast entity to given type";
    }
};

#endif //ENTITYCASTEXCEPTION
