#include <asreal_oprs/asrael/asrael_remote_control_client.h>

AsraelRemoteControlClient::AsraelRemoteControlClient()
{
	serverUrl_ = "http://localhost:5678/asrael.rem";
}

std::string AsraelRemoteControlClient::getVersionNumber()
{
	std::string version;
	xmlrpc_c::value result;
	
	client_.call(serverUrl_, "asrael.getVersionNumber", "", &result);

	version = (xmlrpc_c::value_string(result));

	return version;
}



AsrealRemoteCommandResponse AsraelRemoteControlClient::loadLevel(std::string level)
{
    std::vector<std::string> params;
    params.push_back(level);
    return executeCommand("God","LoadLevel",params);
}

AsrealRemoteCommandResponse AsraelRemoteControlClient::executeCommand(std::string item, std::string command)
{
    std::vector<std::string> params;
    return executeCommand(item,command,params);
}

AsrealRemoteCommandResponse AsraelRemoteControlClient::executeCommand(std::string item, std::string command, std::string param)
{
    std::vector<std::string> params;
    params.push_back(param);
    return executeCommand(item,command,params);
}

AsrealRemoteCommandResponse AsraelRemoteControlClient::executeCommand(std::string item, std::string command, std::string param1, std::string param2)
{
    std::vector<std::string> params;
    params.push_back(param1);
    params.push_back(param2);
    return executeCommand(item,command,params);
}


AsrealRemoteCommandResponse AsraelRemoteControlClient::executeCommand(std::string item, std::string command, std::vector<std::string> params)
{
    xmlrpc_c::value result;
    xmlrpc_c::paramList xmlrpc_parms;

    xmlrpc_parms.add(xmlrpc_c::value_string(item));
    xmlrpc_parms.add(xmlrpc_c::value_string(command));
    for(std::vector<std::string>::iterator i=params.begin();i!=params.end();i++)
    {
        xmlrpc_parms.add(xmlrpc_c::value_string(*i));
    }

    client_.call(serverUrl_, "asrael.executeCommand", xmlrpc_parms, &result);

    AsrealRemoteCommandResponse response(result);

    return response;
}

std::vector<std::string> AsraelRemoteControlClient::getItemList()
{
    std::vector<std::string> items;
    xmlrpc_c::value result;

    client_.call(serverUrl_, "asrael.listItems", "", &result);

    xmlrpc_c::value_array const item_list((xmlrpc_c::value_array(result)));

    for(int i=0;i<item_list.size();i++)
    {
        items.push_back((xmlrpc_c::value_string(item_list.vectorValueValue()[i])));
    }

    return items;
}

AsraelCommandInfoList AsraelRemoteControlClient::getCommandList(std::string object)
{

    AsraelCommandInfoList commands;
    xmlrpc_c::value result;

    xmlrpc_c::paramList xmlrpc_parms;

    xmlrpc_parms.add(xmlrpc_c::value_string(object));

    client_.call(serverUrl_, "asrael.listCommands", xmlrpc_parms, &result);

    xmlrpc_c::value_array const command_list((xmlrpc_c::value_array(result)));

    for(int i=0;i<command_list.size();i++)
    {
        commands.push_back(AsraelCommandInfo(xmlrpc_c::value_struct(command_list.vectorValueValue()[i])));
    }

    return commands;
}
