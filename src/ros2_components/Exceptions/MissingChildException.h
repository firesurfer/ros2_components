#ifndef MISSINGCHILDEXCEPTION
#define MISSINGCHILDEXCEPTION

#include <exception>
#include <string>

class MissingChildException : std::exception
{
public:
    explicit MissingChildException(const std::string& msg = "Requested child is missing");

    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //MISSINGCHILDEXCEPTION
