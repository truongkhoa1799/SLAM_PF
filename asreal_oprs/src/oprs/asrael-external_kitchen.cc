/*                               -*- Mode: C -*- 
 *
 * Copyright (c) 2011 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    - Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    - Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>

#include "openprs/opaque-pub.h"
#include "openprs/default-hook.h"
#include "openprs/default-hook_f.h"
#include "openprs/user-end-hook-pub.h"
#include "openprs/user-end-hook_f-pub.h"
#include "openprs/intention_f-pub.h"

PBoolean my_intention_list_sort_example(Intention *i1, Intention *i2)
{
  return (intention_priority(i1) > intention_priority(i2));
}

void end_asrael_oprs_hook()
{
  printf("Bye, bye Gargamel...\n");
}

void declare_asrael_oprs_action(void);
void declare_asrael_oprs_eval_funct(void);
void declare_asrael_oprs_eval_pred(void);

extern "C" void init_asrael_oprs(void)
{
  declare_asrael_oprs_action();
  declare_asrael_oprs_eval_funct();
  declare_asrael_oprs_eval_pred();

  add_user_end_kernel_hook((PFV)end_asrael_oprs_hook);
}

