/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Maximum radius of orbit
 *
 * @unit m
 * @min 1.0
 * @max 10000.0
 * @increment 0.5
 * @decimal 1
 * @group FlightTaskOrbit
 */
PARAM_DEFINE_FLOAT(MC_ORBIT_RAD_MAX, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_MAX, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_XY_TRAJ_P, 1000.0f);
//PARAM_DEFINE_FLOAT(NAV_MC_ALT_RAD, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_XY_ERR_MAX, 1000.0f);
//PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_JERK_AUTO, 1000.0f);
//PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 1000.0f);
//PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_UP, 1000.0f);
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_DN, 1000.0f);
