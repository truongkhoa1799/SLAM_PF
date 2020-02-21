//                               -*- Mode: C++ -*- 
// tb2-action.cc -- contains user defined evaluable actions for ROS/TB2.
// 
// Copyright (c) 2011-2013 LAAS/CNRS
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//    - Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    - Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
// 

/*
 * This file name may be misleading, we are not talking about ROS actions here, but OpenPRS actions.
 * I illustrate here how can one uses ROS topics/services/actions from OpenPRS.
 */

#include <ros/ros.h>
#include <asreal_oprs/asrael/asrael_remote_control_client.h>


#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <rosgraph_msgs/Log.h>


#include <stdio.h>
#include <cstdlib>

#include "openprs/macro-pub.h"
#include "openprs/opaque-pub.h"
#include "openprs/constant-pub.h"
#include "openprs/oprs-type-pub.h"
#include "openprs/oprs-sprint-pub.h"

#include "openprs/user-action.h"

#include "openprs/oprs-type_f-pub.h"
#include "openprs/fact-goal_f-pub.h"
#include "openprs/oprs_f-pub.h"

#include "openprs/pu-mk-term_f.h"
#include "openprs/pu-genom_f.h"
#include "openprs/action_f-pub.h"
#include "openprs/oprs-sprint_f-pub.h"
#include <openprs/pu-parse-tl_f.h>

#include <asreal_oprs/oprs/action_wumpus.h>
#include <asreal_oprs/oprs/oprs.h>
#include <asreal_oprs/oprs/oprs_f.h>
#include <asreal_oprs/oprs/evaluable-function_f.h>
#include <asreal_oprs/oprs/Parameters_f.h>


bool bumping_ = false;


Term *loadAction(TermList terms)
{

    char* level;

    if (PUGetOprsParameters(terms, 1, ATOM, &level))
    {
        std::cout << "Load " << std::string(level) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("God", "LoadLevel", std::string(level));

        std::cout << "Response of Load: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[loadAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *moveAction(TermList terms)
{

    std::cout << "Move Forward" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "move");

    if(response.code_ == TRUE)
        bumping_ = false;
    else
        bumping_ = true;

    std::cout << "Response of Move: " << response << std::endl;
    return(build_t());

}

/*---------------------------------------------------------------------------*/

Term *shootAction(TermList terms)
{

    std::cout << "Shoot" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "shoot");

    std::cout << "Response of Shoot: " << response << std::endl;
    return(build_t());
}

/*---------------------------------------------------------------------------*/

Term *climbAction(TermList terms)
{

    std::cout << "Climb Out" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "climb");

    std::cout << "Response of Climb Out: " << response << std::endl;
    return(build_t());
}

/*---------------------------------------------------------------------------*/

Term *turnleftAction(TermList terms)
{

    bumping_ = false;

    std::cout << "Turn Left" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "turnLeft");

    std::cout << "Response of Turn Left: " << response << std::endl;
    return(build_t());
}

/*---------------------------------------------------------------------------*/

Term *turnrightAction(TermList terms)
{

    bumping_ = false;


    std::cout << "Turn Right" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "turnRight");

    std::cout << "Response of Turn Right: " << response << std::endl;
    return(build_t());
}

/*---------------------------------------------------------------------------*/

Term *takegoldAction(TermList terms)
{

    std::cout << "Take Gold" << std::endl;

    AsraelRemoteControlClient client;
    AsrealRemoteCommandResponse response;

    response = client.executeCommand("Player", "takeGold");

    std::cout << "Response of Take Gold: " << response << std::endl;
    return(build_t());

}

/*---------------------------------------------------------------------------*/


void declare_asrael_oprs_action(void)
{
    make_and_declare_action("Load", loadAction, 1);
    make_and_declare_action("Move", moveAction, 0);
    make_and_declare_action("Shoot", shootAction, 0);
    make_and_declare_action("Climb", climbAction, 0);
    make_and_declare_action("TurnLeft", turnleftAction, 0);
    make_and_declare_action("TurnRight", turnrightAction, 0);
    make_and_declare_action("TakeGold", takegoldAction, 0);
    return;
}
