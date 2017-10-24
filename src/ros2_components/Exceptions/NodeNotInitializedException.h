#ifndef NODENOTINITIALIZEDEXCEPTION_H
#define NODENOTINITIALIZEDEXCEPTION_H

#include <exception>

class NodeNotInitializedException: public std::exception
{
public:
    NodeNotInitializedException();

    virtual const char* what() const throw()
    {
       return "Node has not been initialized";
    }
};

#endif // NODENOTINITIALIZEDEXCEPTION_H
