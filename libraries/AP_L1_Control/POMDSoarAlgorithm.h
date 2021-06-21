// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#pragma once

#include <APM_Control/APM_Control.h>
#include "PomdpSolver.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Common/Location.h>
//#include "WindExtendedKalmanFilter.h"
//#include "AP_L1_Control.h"


class AP_L1_Control;


//
// POMDSoarAlgorithm, the POMDP/Bayesian RL-based logic used by SoaringController to decide on the course of action
//
class POMDSoarAlgorithm
{
    friend class AP_L1_Control;
    //ExtendedKalmanFilterP _ekf{};
    //WindExtendedKalmanFilter _wind_ekf{};

private:
	AP_AHRS &_ahrs;
	const AP_L1_Control *_sc;
    float _roll_cmds[MAX_ACTIONS];
    int _n_actions = 2;
    int _prev_n_actions = 2;
    //bool _pomdp_active = false;
    //bool _pomdp_active = true;
    Location _pomdp_wp;
    Location _prev_pomdp_wp;
    float _sign = 1;
    float _pomdp_radius;
    uint64_t _prev_pomdp_update_time = 0;
    int _prev_pomdp_action = 0;
    uint64_t _pomdp_solve_time;
    uint64_t _pomp_loop_time;
    uint64_t _pomp_loop_min_time;
    uint64_t _pomp_loop_max_time;
    uint64_t _pomp_loop_av_time = 0;
    float _pomdp_roll_cmd = 0;
    Vector2f _pomdp_vecNE;
    int _m = 0;
    int _j = 0;
    int _n_action_samples;
    bool _new_actions_to_send = false;
    int _pomdp_mode = 0; // 0 = explore, 1 = max lift
    float _weights[4] = { 1, 1, 1, 1 };
    uint8_t _prev_run_timing_test;

    float _debug_out[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t _debug_out_mode;

    //const AP_AutoTune::ATGains &_gains;
    //AP_AutoTune::ATGains &_gains;
    float _scaling_speed = 0.1f;
	//AP_Float &_get_speed;
    PomdpSolver _solver;
    //AP_L1_Control sc;
    //AP_RollController rollController;

    AP_Int8 pomdp_delta_mode;
    AP_Int16 pomdp_n;
    AP_Int16 pomdp_k;
    AP_Float pomdp_hori;
    AP_Float pomdp_roll1;
    AP_Float pomdp_roll2;
    AP_Float pomdp_step_t;
    AP_Float pomdp_loop_load;
    AP_Int8 pomdp_n_actions;
    AP_Float pomdp_roll_rate;
    AP_Float I_moment;
    AP_Float k_aileron;
    AP_Float k_roll_damping;
    AP_Float c_lp;
    AP_Int8 pomdp_norm_pth;
    AP_Int8 pomdp_extend;
    AP_Int8 pomdp_plan_mode;
    AP_Float pomdp_pth;

    void init_actions(bool mode);
	
protected: 
	
    AP_Int8 soar_active;
    AP_Int8 soar_active_ch;
    AP_Float thermal_vspeed;
    AP_Float thermal_q1;
    AP_Float thermal_q2;
    AP_Float thermal_r;
    AP_Float thermal_distance_ahead;
    AP_Int16 min_thermal_s;
    AP_Int16 min_cruise_s;
    AP_Float polar_CD0;
    AP_Float polar_B;
    AP_Float polar_K;
    AP_Float alt_max;
    AP_Float alt_min;
    AP_Float alt_cutoff;
    AP_Int8 debug_mode;
    AP_Int8 pomdp_on;
    AP_Int8 vario_type;
    AP_Float poly_a;
    AP_Float poly_b;
    AP_Float poly_c;
    AP_Int8 aspd_src;
    AP_Int8 exit_mode;
    AP_Int8 disable_soar_prevention;
    AP_Float mccready_vspeed;
    AP_Int8 enable_geofence;
    AP_Int8 run_timing_test;
    AP_Int8 sg_filter;
    AP_Int8 gps_sync;
    AP_Float aspd_cmd;
    AP_Float test_dist;
    AP_Float test_offset;
    AP_Float test_radius;
    AP_Float test_strength;

public:
	// store time of last update
    uint64_t _prev_update_time;

    // store time of last update of the vario
    uint64_t _prev_vario_update_time;
	
	// store aircraft location at last update
	struct Location _prev_update_location;
	struct Location _prev_vario_update_location;
	struct Location _thermal_start_location;
	uint64_t _thermal_start_time_us;
	//POMDSoarAlgorithm(AP_AHRS &ahrs , AP_L1_Control *sc,AP_RollController &rollController);
	POMDSoarAlgorithm(const AP_L1_Control *sc, AP_AHRS &ahrs);
	/*
	POMDSoarAlgorithm(AP_AHRS &ahrs , AP_L1_Control *sc):
      _ahrs(ahrs),
      _sc(sc)
    {
    AP_Param::setup_object_defaults(this, var_info);
    _prev_pomdp_update_time = AP_HAL::micros64();
	_prev_update_time = AP_HAL::micros64();
     _prev_vario_update_time = AP_HAL::micros64();
     VectorN<float, 3> X = (const float[]) { 0, 0, 0 };
     MatrixN<float, 3> P = (const float[]) { 0.5, 0.5, 0.5 };
     MatrixN<float, 3> Q = (const float[]) { 0.0001, 0.001, 0.001 };
     float R = 0.5;
     _wind_ekf.reset(X, P, Q, R);
	 _gains(get_gains());
   }
   */
    //POMDSoarAlgorithm(AP_RollController &rollController, AP_Float &scaling_speed);
	//POMDSoarAlgorithm(const AP_L1_Control *sc, AP_AHRS &ahrs, const AP_SpdHgtControl *spdHgtControl);
	/*POMDSoarAlgorithm(const AP_L1_Control *sc,AP_AHRS &ahrs, const AP_SpdHgtControl *spdHgtControl):
          _sc(sc),
          _gains(ahrs.get_airspeed())
        , spdHgtControl.get_pitch_demand()
   {
     AP_Param::setup_object_defaults(this, var_info);
    _prev_pomdp_update_time = AP_HAL::micros64();
}*/
/*
   POMDSoarAlgorithm(AP_RollController &rollController, AP_Float &scaling_speed):
    _gains(rollController.get_gains()),
    _scaling_speed(scaling_speed)
    {
     AP_Param::setup_object_defaults(this, var_info);
    _prev_pomdp_update_time = AP_HAL::micros64();
	_prev_update_time = AP_HAL::micros64();
     _prev_vario_update_time = AP_HAL::micros64();
     VectorN<float, 3> X = (const float[]) { 0, 0, 0 };
     MatrixN<float, 3> P = (const float[]) { 0.5, 0.5, 0.5 };
     MatrixN<float, 3> Q = (const float[]) { 0.0001, 0.001, 0.001 };
     float R = 0.5;
     _wind_ekf.reset(X, P, Q, R);
   }
*/   	

	//~POMDSoarAlgorithm();
    static const struct AP_Param::GroupInfo var_info[];
    void init_thermalling();
    //float assess_thermalability(uint8_t exit_mode);
    bool are_computations_in_progress();
    void stop_computations();
    void update_internal_state();
    void update_internal_state_test();
    //bool is_set_to_continue_past_thermal_locking_period();
    int get_curr_mode();
    uint64_t get_latest_pomdp_solve_time();
    float get_action();
   // void send_test_out_msg(mavlink_channel_t chan);
    bool update_thermalling(const Location &current_loc);
    void run_tests();
	void get_heading_estimate(float *hdx, float *hdy) const;
	//const AP_AutoTune::ATGains &get_gains(void) { return _gains; }
	float get_rate() const;
};
