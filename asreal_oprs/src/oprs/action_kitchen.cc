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

#include <asreal_oprs/oprs/action_kitchen.h>
#include <asreal_oprs/oprs/oprs.h>
#include <asreal_oprs/oprs/oprs_f.h>
#include <asreal_oprs/oprs/evaluable-function_f.h>
#include <asreal_oprs/oprs/Parameters_f.h>

Term *moveAction(TermList terms)
{

    char* location;

    if (PUGetOprsParameters(terms, 1, ATOM, &location))
    {
        std::cout << "Move to " << std::string(location) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Move", std::string(location));

        std::cout << "Response of Move: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[moveAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *putAction(TermList terms)
{

    char* location;

    if (PUGetOprsParameters(terms, 1, ATOM, &location))
    {
        std::cout << "Put object to " << std::string(location) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Put", std::string(location));

        std::cout << "Response of Put: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[putAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *takeAction(TermList terms)
{

    char* object;

    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "Take object " << std::string(object) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Take", std::string(object));

        std::cout << "Response of Take: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[takeAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *openAction(TermList terms)
{

    char* object;

    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "Open object " << std::string(object) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Open", std::string(object));

        std::cout << "Response of Open: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[openAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *closeAction(TermList terms)
{

    char* object;

    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "Close object " << std::string(object) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Close", std::string(object));

        std::cout << "Response of Close: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[closeAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *turnOnAction(TermList terms)
{

    char* object;

    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "TurnOn object " << std::string(object) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "TurnOn", std::string(object));

        std::cout << "Response of TurnOn: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[turnOnAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

Term *turnOffAction(TermList terms)
{

    char* object;

    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "TurnOff object " << std::string(object) << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "TurnOff", std::string(object));

        std::cout << "Response of TurnOff: " << response << std::endl;
        return(build_t());
    }
    else // if the argument parsing went wrong, return nil.
    {
        std::cout << "[turnOffAction] PUGetOprsParameters failed" << std::endl;
        return(build_nil());
    }
}

/*---------------------------------------------------------------------------*/

void declare_asrael_oprs_action(void)
{
    make_and_declare_action("Move", moveAction, 1);
    make_and_declare_action("Put", putAction, 1);
    make_and_declare_action("Take", takeAction, 1);
    make_and_declare_action("Open", openAction, 1);
    make_and_declare_action("Close", closeAction, 1);
    make_and_declare_action("TurnOn", turnOnAction, 1);
    make_and_declare_action("TurnOff", turnOffAction, 1);
    return;
}
