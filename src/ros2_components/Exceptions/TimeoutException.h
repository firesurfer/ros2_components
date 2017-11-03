#ifndef ROS2_TIMEOUTEXCEPTION_H
#define ROS2_TIMEOUTEXCEPTION_H

#include <exception>

class TimeoutException: public std::exception
{
public:
    TimeoutException();

    virtual const char* what() const throw()
    {
        return "Timed out";
    }
};

#endif // ROS2_TIMEOUTEXCEPTION_H
