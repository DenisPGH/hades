/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file flight_mode_manager_params.c
 */
//PARAM_DEFINE_FLOAT(LNDMC_ALT_MAX, 1.e-2f);
//PARAM_DEFINE_FLOAT(LNDMC_ALT_MAX, 1.e-2f);

PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 1.e-2f);

PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_Z_MAN_EXPO, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_YAW_EXPO, 1.e-2f);

PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 1.e-2f);
//PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 1.e-2f);



PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 1.e-2f);




PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 1.e-2f);
PARAM_DEFINE_INT32(MPC_ALT_MODE, 3);
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_LAND_ALT1, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_LAND_ALT2, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.e-2f);



PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 1.e-2f);



/* StackAcceleration.hpp*/

PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_SIDE, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_BACK, 1.e-2f);
//PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 1.e-2f);
//PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 1.e-2f);


/* FlightTaskAuto*/
PARAM_DEFINE_INT32(MPC_YAW_MODE, 3);
PARAM_DEFINE_FLOAT(MPC_LAND_CRWL, 1.e-2f);
PARAM_DEFINE_INT32(MPC_LAND_RC_HELP, 3);
PARAM_DEFINE_FLOAT(MPC_LAND_RADIUS, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_LAND_ALT3, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_TKO_RAMP_T, 1.e-2f);
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 1.e-2f);


/* FlightModeMahager.cpp*/
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 1.e-2f);
PARAM_DEFINE_INT32(MPC_POS_MODE, 3);

PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 1.e-2f);

