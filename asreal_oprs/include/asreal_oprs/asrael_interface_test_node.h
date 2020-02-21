#ifndef ASRAEL_INTERFACE_TEST_NODE_H
#define ASRAEL_INTERFACE_TEST_NODE_H


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>


#include "ros/ros.h"
#include <asreal_oprs/asrael/asrael_remote_control_client.h>
#include "asrael/ExecuteCommand.h"
#include <unistd.h>



class AsraelInterfaceTest
{

public:
    AsraelInterfaceTest();

    void start();

    bool executeCommand(asreal_oprs::ExecuteCommand::Request  &req, asreal_oprs::ExecuteCommand::Response &res);


};



#endif // ASRAEL_INTERFACE_TEST_NODE_H
