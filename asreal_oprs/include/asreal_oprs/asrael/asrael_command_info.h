#include <vector>
#include <string>
#include <map>


class AsraelCommandInfo
{
public:

    AsraelCommandInfo()
    {
        name_="Command Name Undefined";
        isSensing_=false;
    }

    AsraelCommandInfo(xmlrpc_c::value_struct info)
    {
        std::map<std::string, xmlrpc_c::value> const command_info(info);
        try
        {
            name_ = (xmlrpc_c::value_string(command_info.at("name")));
            argumentNames_.clear();
            xmlrpc_c::value_array const arg_list =  (xmlrpc_c::value_array(command_info.at("argumentNames")));
            for(int i=0;i<arg_list.size();i++)
            {
                argumentNames_.push_back((xmlrpc_c::value_string(arg_list.vectorValueValue()[i])));
            }
            isSensing_ = (xmlrpc_c::value_boolean (command_info.at("isSensing")));

        }
        catch(...)
        {
            isSensing_=false;
            name_="Command Name Parsing Error";
        }
    }

    friend std::ostream & operator<<(std::ostream &os, AsraelCommandInfo const & info)
    {
        os << "COMMAND name=" << info.name_ << ", sensing=" << (info.isSensing_?"TRUE":"FALSE") << std::endl;
        int j=1;
        for(std::vector<std::string>::const_iterator i=info.argumentNames_.begin();i!=info.argumentNames_.end();i++)
        {
            os << " " << j++ << ".paramter" << "=" << *i << std::endl;
        }

        return os;
    }


	std::string name_;
	std::vector<std::string> argumentNames_;
	bool isSensing_;
};

typedef std::vector<AsraelCommandInfo> AsraelCommandInfoList;
