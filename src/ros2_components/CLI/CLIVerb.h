#ifndef CLIVERB_H
#define CLIVERB_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include "CLIArgument.h"
#include "CLIParameter.h"
#include <QString>
#include <list>
namespace  ros2_components {

/**
 * @brief The CLIVerb class represents a verb for the @ref CLIParser.
 * Example: <programname> build
 * Verbs can be nested
 */
class CLIVerb
{
public:
    typedef std::shared_ptr<CLIVerb> SharedPtr;
    CLIVerb(std::string _verb, std::string _description, std::shared_ptr<CLIVerb> _parent);
    CLIVerb(std::string _verb, std::string _description, std::shared_ptr<CLIVerb> _parent, bool * _found);
    /**
     * @brief addVerb - add a sub verb to this verb
     * @param _child
     * @return
     */
    bool addVerb(CLIVerb::SharedPtr _child);
    /**
     * @brief addArgument - add an argument to this verb
     * @param _arg
     * @return
     */
    bool addArgument(CLIArgument::SharedPtr _arg);
    /**
     * @brief addParameter - add a parameter to this verb
     * @param _param
     * @return
     */
    bool addParameter(CLIParameter::SharedPtr _param);
    /**
     * @brief parse - Parse the corresponding arguments for this verb -> Pass subverbs and arguments to childen
     * @param arguments
     * @return
     */
    bool parse(std::vector<std::string>& str, bool *error);

    /**
     * @brief wasFound
     * @return true if the verb was found
     */
    bool wasFound();
    /**
     * @brief getName
     * @return The name of this verb
     */
    std::string getName() const;
    /**
     * @brief getParent
     * @return The parent of this verb
     */
    std::shared_ptr<CLIVerb> getParent() const;
    /**
     * @brief getDescription
     * @return The description string of this verb
     */
    std::string getDescription() const;

    std::vector<CLIArgument::SharedPtr> getAllCliArguments() const;

    std::map<std::string, CLIVerb::SharedPtr> getChildVerbs() const;

    std::vector<CLIParameter::SharedPtr> getAllCliParameter() const;


private:

    std::string name;
    std::string description;
    std::shared_ptr<CLIVerb> parent;
    std::map<std::string, CLIVerb::SharedPtr> childVerbs;
    std::list <CLIArgument::SharedPtr> cliArguments;
    std::list <CLIParameter::SharedPtr> cliParameters;
    std::vector<CLIArgument::SharedPtr> allCliArguments;
    std::vector<CLIParameter::SharedPtr> allCliParameter;
    bool* found;

    bool isVerb(std::string arg);

};
}

#endif // CLIVERB_H
