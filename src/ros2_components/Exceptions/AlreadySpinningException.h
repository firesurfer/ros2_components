#ifndef ALREADYSPINNINGEXCEPTION_H
#define ALREADYSPINNINGEXCEPTION_H

#include <exception>

class AlreadySpinningException: public std::exception
{
public:
    AlreadySpinningException();

    virtual const char* what() const throw()
    {
       return "Node is already spinning";
    }
};

#endif // ALREADYSPINNINGEXCEPTION_H
