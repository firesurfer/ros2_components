#ifndef UNKNOWNCOMMANDEXCEPTION_H
#define UNKNOWNCOMMANDEXCEPTION_H

#include <exception>
#include <string>

class UnknownCommandException : std::exception
{
public:
    explicit UnknownCommandException(const std::string& msg = "Unknown command");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //UNKNOWNCOMMANDEXCEPTION_H
