#ifndef NOTIMPLEMENTEDEXCEPETION_H
#define NOTIMPLEMENTEDEXCEPETION_H


#include <exception>
#include <string>


class NotImplementedException:  std::exception
{
public:
    explicit NotImplementedException(const std::string& msg = "Functionality not implemented");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif // NOTIMPLEMENTEDEXCEPETION_H
