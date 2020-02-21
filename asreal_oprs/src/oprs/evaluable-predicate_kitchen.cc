//                              -*- Mode: C++ -*- 
// tb2-evaluable-predicate.cc --- 
// 
// Filename: tb2-evaluable-predicate.cc
// Description: 
// Author: Felix Ingrand <felix@laas.fr>
// Created: Mon Feb 28 15:02:30 2011 (+0100)
// 
// Copyright (c) 2011 LAAS/CNRS
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//    - Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    - Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
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

#include <stdio.h>
#include <asreal_oprs/asrael/asrael_remote_control_client.h>
#include "openprs/opaque-pub.h"
#include "openprs/constant-pub.h"
#include "openprs/oprs-type-pub.h"
#include "openprs/user-ev-predicate.h"
#include "openprs/ev-predicate_f-pub.h"


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "openprs/pu-parse-tl_f.h"
#include "openprs/pu-mk-term_f.h"
#include "openprs/pu-genom_f.h"

#include <asreal_oprs/oprs/oprs.h>
#include <asreal_oprs/oprs/oprs_f.h>
#include <asreal_oprs/oprs/Parameters_f.h>


PBoolean see(TermList terms)
{
    std::cout << "See is called " << std::endl;

    char* object;
    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "Can Asrael see " << std::string(object) << " ?" << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "See", std::string(object));

        std::cout << "Response of See: " << response << std::endl;

        if(response.code_ == TRUE)
            return TRUE;
        else
            return FALSE;
    }
    else
    {
        std::cout << "[see] PUGetOprsParameters failed" << std::endl;
        return FALSE;
    }

}

PBoolean hot(TermList terms)
{
    char* object;
    if (PUGetOprsParameters(terms, 1, ATOM, &object))
    {
        std::cout << "Is " << std::string(object) << " hot?" << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "Hot", std::string(object));

        std::cout << "Response of Hot: " << response << std::endl;

        if(response.code_ == TRUE)
            return TRUE;
        else
            return FALSE;
    }
    else
    {
        std::cout << "[hot] PUGetOprsParameters failed" << std::endl;
        return FALSE;
    }

}

PBoolean is_at(TermList terms)
{
    char* object_top;
    char* object_below;
    if (PUGetOprsParameters(terms, 2, ATOM, &object_top, ATOM, &object_below))
    {
        std::cout << "Is object " << std::string(object_top) << " on top of " << std::string(object_below) << "?" << std::endl;

        AsraelRemoteControlClient client;
        AsrealRemoteCommandResponse response;

        response = client.executeCommand("Gargamel", "IsAt", std::string(object_top), std::string(object_below));

        std::cout << "Response of See: " << response << std::endl;

        if(response.code_ == TRUE)
            return TRUE;
        else
            return FALSE;
    }
    else
    {
        std::cout << "[is_at] PUGetOprsParameters failed" << std::endl;
        return FALSE;
    }

}




void declare_asrael_oprs_eval_pred(void)
{
    make_and_declare_eval_pred("see", see, 1, TRUE);
    make_and_declare_eval_pred("hot", hot, 1, TRUE);
    make_and_declare_eval_pred("is_at", is_at, 2, TRUE);

    return;
}






