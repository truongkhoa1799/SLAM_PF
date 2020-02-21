#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>
#include <string>
#include <iostream>
#include <map>
#include <asreal_oprs/asrael/asrael_remote_response.h>
#include <asreal_oprs/asrael/asrael_command_info.h>


class AsraelRemoteControlClient
{
	
public:
	AsraelRemoteControlClient();
	std::string getVersionNumber();
    AsraelCommandInfoList getCommandList(std::string object = "");
    std::vector<std::string> getItemList();
    AsrealRemoteCommandResponse loadLevel(std::string level);
    AsrealRemoteCommandResponse executeCommand(std::string item, std::string command, std::vector<std::string> params);
    AsrealRemoteCommandResponse executeCommand(std::string item, std::string command);
    AsrealRemoteCommandResponse executeCommand(std::string item, std::string command, std::string param);
    AsrealRemoteCommandResponse executeCommand(std::string item, std::string command, std::string param1, std::string param2);


private:
   	std::string serverUrl_; 
	xmlrpc_c::clientSimple client_;


};


/*

    std::string const serverUrl("http://localhost:5678/asrael.rem");
    std::string const methodName("asrael.executeCommand");

    

        
    myClient.call(serverUrl, methodName, "sss", &result,"God","LoadLevel","Kitchen");

    std::map<std::string, xmlrpc_c::value> const sum((xmlrpc_c::value_struct(result)));
        // Assume the method returned an integer; throws error if not

    //std::cout << "version number: " << sum << std::endl;

    for(std::map<std::string, xmlrpc_c::value>::const_iterator i=sum.begin();i!=sum.end();i++)
    {	
	try
	{
	std::string const str((xmlrpc_c::value_string(i->second)));
	std::cout << str << std::endl;
	}
	catch(...)
	{
	}
    }

*/
