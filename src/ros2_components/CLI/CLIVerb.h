#ifndef CLIVERB_H
#define CLIVERB_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include "CLIArgument.h"
#include <QString>
#include <list>
namespace  ros2_components {


class CLIVerb
{
public:
    typedef std::shared_ptr<CLIVerb> SharedPtr;
    CLIVerb(std::string _verb, std::string _description, std::shared_ptr<CLIVerb> _parent);
    CLIVerb(std::string _verb, std::string _description, std::shared_ptr<CLIVerb> _parent, bool * _found);
    bool addVerb(CLIVerb::SharedPtr _child);
    bool addArgument(CLIArgument::SharedPtr _arg);
    /**
     * @brief parse - Parse the corresponding arguments for this verb -> Pass subverbs and arguments to childen
     * @param arguments
     * @return
     */
    bool parse(std::vector<std::string>& str);

    bool wasFound(){return found;}
    std::string getName() const;

    std::shared_ptr<CLIVerb> getParent() const;

    std::string getDescription() const;

    std::vector<CLIArgument::SharedPtr> getAllCliArguments() const;

    std::map<std::string, CLIVerb::SharedPtr> getChildVerbs() const;

private:

    std::string name;
    std::string description;
    std::shared_ptr<CLIVerb> parent;
    std::map<std::string, CLIVerb::SharedPtr> childVerbs;
    std::list <CLIArgument::SharedPtr> cliArguments;
     std::vector<CLIArgument::SharedPtr> allCliArguments;
    bool* found;

    bool isVerb(std::string arg);

};
}

#endif // CLIVERB_H
