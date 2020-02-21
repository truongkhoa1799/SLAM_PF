#include "asreal_oprs/asrael_interface_test_node.h"

AsraelInterfaceTest::AsraelInterfaceTest()
{}


void AsraelInterfaceTest::start()
{
    try
    {
        AsraelRemoteControlClient client;

        std::cout << client.getVersionNumber() << std::endl;

        AsrealRemoteCommandResponse response = client.loadLevel("Wumpus");

        std::cout << "load level kitchen " << response.code_ << " " << response.message_ << std::endl;

    }
    catch(std::exception const& e)
    {
        std::cout << e.what() << std::endl;
    }



}

bool AsraelInterfaceTest::executeCommand(asreal_oprs::ExecuteCommand::Request &req, asreal_oprs::ExecuteCommand::Response &res)
{
    try
    {
        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;
        if(req.command_name == "getItemList")
        {
            std::vector<std::string> items = client.getItemList();

            for(std::vector<std::string>::iterator i=items.begin();i!=items.end();i++)
            {
                std::cout << "ITEM: " << *i << std::endl;
            }
            res.successful_execution = true;
            res.asrael_response = "";
            return true;
        }
        else if(req.command_name == "getCommandList")
        {
            AsraelCommandInfoList commands = client.getCommandList("");

            for (AsraelCommandInfoList::iterator i=commands.begin();i!=commands.end();i++)
            {
                std::cout << *i;
            }
            res.successful_execution = true;
            return true;
        }
        else if(req.command_name != "IsAt")
        {
            response = client.executeCommand("Player",req.command_name);
        }
        else
        {
            response = client.executeCommand("Player",req.command_name, req.parameter_1, req.parameter_2);
        }

        std::string code_str;
        res.successful_execution = false;
        switch(response.code_)
        {
        case FALSE:
            code_str = "FALSE";
            break;
        case TRUE:
            code_str = "TRUE";
            res.successful_execution = true;
            break;
        case ERROR:
            code_str = "ERROR";
            break;
        case UNKNOWN:
            code_str = "UNNOWN";
            break;
        default:
            code_str ="UNDEFINED";
        }
        res.asrael_response = response.message_;
        std::cout << response << std::endl;
    }
    catch(std::exception const& e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;

}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "asreal_interface_test");
    ros::NodeHandle n;

    AsraelInterfaceTest test;


    ROS_INFO("Starting Asrael Interface Test node");
    test.start();
    ROS_INFO("Ready to execute commands");
    ros::ServiceServer service = n.advertiseService("execute_commands", &AsraelInterfaceTest::executeCommand, &test);

    ros::spin();

    return 0;
}
