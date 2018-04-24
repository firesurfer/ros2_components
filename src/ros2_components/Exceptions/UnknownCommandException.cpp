#include "UnknownCommandException.h"

UnknownCommandException::UnknownCommandException(const std::string& msg)
    : message(msg)
{
}
