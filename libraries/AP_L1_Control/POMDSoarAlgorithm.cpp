// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include <GCS_MAVLink/GCS.h>
#include "AP_L1_Control.h"
#include "POMDSoarAlgorithm.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Common/Location.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo POMDSoarAlgorithm::var_info[] = {

    // @Param: POMDP_ON
    // @DisplayName: Is the POMDSoar algorithm on?
    // @Description: If 1, the soaring controller uses the POMDSoar algorithm. If 0, the soaring controller uses the ArduSoar algorithm.
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_ON", 0, POMDSoarAlgorithm, pomdp_on, 0),

    // @Param: POMDP_N
    // @DisplayName: Number of samples per action trajectory used by POMDSoar
    // @Description: Number of samples per action trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_N", 1, POMDSoarAlgorithm, pomdp_n, 10),

    // @Param: POMDP_K
    // @DisplayName: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Description: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.POMDSoar使用的动作轨迹每1秒的POMDP采样点数。 
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_K", 2, POMDSoarAlgorithm, pomdp_k, 5),

    // @Param: POMDP_HORI
    // @DisplayName: POMDP planning horizon used by POMDSoar.//探索模式动作持续时间
    // @Description: POMDP planning horizon used by POMDSoar.
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_HORI", 3, POMDSoarAlgorithm, pomdp_hori, 4.0),

    // @Param: SG_FILTER
    // @DisplayName: use Savitzky Golay vario filter
    // @Description: 0 = 1st order filter (don't use the SG filter), 1 = SG filter
    // @Units: none
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SG_FILTER", 4, POMDSoarAlgorithm, sg_filter, 1),

    // @Param: GPS_SYNC
    // @DisplayName: Enable synchronization between vario updates and GPS updates.
    // @Description: Enable synchronization between vario updates and GPS updates. 0 = off, 1 = sync vario update with GPS update.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("GPS_SYNC", 5, POMDSoarAlgorithm, gps_sync, 1),

    // @Param: POMDP_STEP_T
    // @DisplayName:POMDP planning step solve time
    // @Description: The amount of computation time the POMDP solver has for computing the next actionPOMDP解算器用于计算下一个动作的计算时间量 
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_STEP", 6, POMDSoarAlgorithm, pomdp_step_t, 1),

    // @Param: POMDP_LOOP
    // @DisplayName: Number of POMDP solver's inner loop executions per planning step
    // @Description: Number of POMDP solver's inner loop executions per planning step (see also the POMDP_STEP_T parameter)
    //每个计划步骤的POMDP求解器的内循环执行次数（另请参阅POMDP_STEP_T参数）
    // @Units:
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_LOOP",7, POMDSoarAlgorithm, pomdp_loop_load, 1),


// @Param: DEBUG
    // @DisplayName: Debug the POMDP solver
    // @Description: Turn on POMDP solver debugging mode. WARNING: to be used on the ground only. Make sure it is set to 0 before takeoff.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 8, POMDSoarAlgorithm, debug_mode, 0),

    // @Param: POMDP_ROLL1
    // @DisplayName: POMDP's maximum commanded roll angle.
    // @Description: Maximum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL1", 9, POMDSoarAlgorithm, pomdp_roll1, 4),

    // @Param: POMDP_ROLL2
    // @DisplayName: POMDP's minimum commanded roll angle.
    // @Description: Minimum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL2", 10, POMDSoarAlgorithm, pomdp_roll2, 8),

    // @Param: POMDP_RRATE
    // @DisplayName: The sailplane UAV's roll rate increment used by POMDSoar
    // @Description: The sailplane UAV's roll rate increment used by POMDSoar.
    // @Units: degrees/second
    // @Range: 0 1000/75
    // @User: Advanced
    AP_GROUPINFO("POMDP_RRATE", 11, POMDSoarAlgorithm, pomdp_roll_rate, 10),

    // @Param: POMDP_N_ACT
    // @DisplayName: POMDP number of actions
    // @Description: Number of actions in the POMDP used by POMDSoar. The roll angle input commands corresponding to actions are endpoints of (POMDP_N_ACT-1) equal intervals between POMDP_ROLL2 and POMDP_ROLL1 (inclusive). 
    // @Units: seconds
    // @Range: 1 254
    // @User: Advanced
    AP_GROUPINFO("POMDP_N_ACT", 12, POMDSoarAlgorithm, pomdp_n_actions, 2),

    // @Param: I_MOMENT
    // @DisplayName: I-moment coefficient
    // @Description: Airframe-specific I-moment coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    //POMDSoar使用机身特定的i型力矩系数来模拟给定的控制侧滚角对应的轨迹。
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("I_MOMENT", 13, POMDSoarAlgorithm, I_moment, 0.00257482),

    // @Param: K_AILERON
    // @DisplayName: Aileron K coefficient
    // @Description: Airframe-specific aileron K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    //机身特定的副翼K系数被POMDSoar用来建立与给定的控制侧滚角相对应的轨迹模型。
    // @Units: seconds
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_AILERON", 14, POMDSoarAlgorithm, k_aileron, 1.44833047),

    // @Param: K_ROLLDAMP
    // @DisplayName: Roll dampening K coefficient
    // @Description: Airframe-specific roll-dampening K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    //POMDSoar使用的特定于飞机的侧倾阻尼K系数对与给定的指令侧倾角相对应的轨迹进行建模。
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_ROLLDAMP", 15, POMDSoarAlgorithm, k_roll_damping, 0.41073589),

    // @Param: ROLL_CLP
    // @DisplayName: CLP coefficient
    // @Description: Airframe-specific CLP coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    //机身特定的CLP系数被POMDSoar用来建立与给定的控制侧滚角相对应的轨迹模型
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("ROLL_CLP", 16, POMDSoarAlgorithm, c_lp, -1.12808702679),
     // @Param: POMDP_TH
    // @DisplayName: POMDSoar's threshold on tr(P) for switching between explore and max-lift modes.
    // @Description: POMDSoar's threshold on the P matrix trace for switching between explore and max-lift modes.POMDSoar在P矩阵轨迹上的阈值，用于在探索模式和最大升力模式之间切换。 
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_PTH", 17, POMDSoarAlgorithm, pomdp_pth, 50),
    
// @Param: POMDP_NORM
    // @DisplayName: Normalize the P matrix trace when solving the POMDP
    // @Description: Normalize the trace of the P matrix used for switching between explore and max-lift modes in POMDSoar. 0 = no normalizing, 1 = normalize tr(P)
    //归一化用于在POMDSoar中的探索模式和最大提升模式之间切换的P矩阵的轨迹。 0 =不规范化，1 =规范tr（P）
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_NORM", 18, POMDSoarAlgorithm, pomdp_norm_pth, 0),
    // @Param: DELTA_MODE
    // @DisplayName: Enable delta actions
    // @Description: Enable delta actions, whereby an action's roll angle is added to the UAV's current roll angle. 0 = off, 1 = on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DELTA_MODE", 19, POMDSoarAlgorithm, pomdp_delta_mode, 0),

    // @Param: POMDP_EXT
    // @DisplayName: Enable action duration extension in POMDSoar's max-lift mode compared to the explore mode
    // @Description: 0 = off, > 1 = multiplicative factor by which to extend action duration in max-lift compared to the explore mode.利用模式就是max-lift模式
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_EXT", 20, POMDSoarAlgorithm,pomdp_extend, 0),

    // @Param: POMDP_PLN
    // @DisplayName: Enable deterministic trajectory planning mode for the POMDP
    // @Description: Enable deterministic trajectory planning mode for the POMDP. 0 = off, 1 on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_PLN", 21, POMDSoarAlgorithm,pomdp_plan_mode, 0),

    // @Param: RUN_TEST
    // @DisplayName: Run a timing testRun a timing test
    // @Description: 0 = off, 1 = exp test
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RUN_TEST", 22, POMDSoarAlgorithm, run_timing_test, 0),

    // @Param: ARSP_CMD
    // @DisplayName: Commanded airspeed
    // @Description: Commanded airspeed.
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("ARSP_CMD", 23, POMDSoarAlgorithm, aspd_cmd, 9),

	AP_GROUPINFO("POLY_A", 24, POMDSoarAlgorithm, poly_a, -0.03099261),

    // @Param: POLY_B
    // @DisplayName: Sink polynomial coefficient b
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_B", 25, POMDSoarAlgorithm, poly_b, 0.44731854),

    // @Param: POLY_C
    // @DisplayName: Sink polynomial coefficient c
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_C", 26, POMDSoarAlgorithm, poly_c, -2.30292972),

	// @Param: ASPD_SRC
    // @DisplayName: Airspeed source
    // @Description: 0 = airspeed sensor, 1 = wind corrected ground speed
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ASPD_SRC", 27, POMDSoarAlgorithm, aspd_src, 1),

	//AP_GROUPINFO("_scaling_speed", 27, POMDSoarAlgorithm, _scaling_speed, 0.11111111),
    
    AP_GROUPEND
};

POMDSoarAlgorithm::POMDSoarAlgorithm(const AP_L1_Control *sc, AP_AHRS &ahrs):
       _sc(sc),
	  _ahrs(ahrs)
    {
    AP_Param::setup_object_defaults(this, var_info);
   // _prev_pomdp_update_time = AP_HAL::micros64();
	//_prev_update_time = AP_HAL::micros64();
     //_prev_vario_update_time = AP_HAL::micros64();
   	 
   }

/*
POMDSoarAlgorithm::POMDSoarAlgorithm(AP_AHRS &ahrs , AP_L1_Control *sc,AP_RollController &rollController):
      _ahrs(ahrs),
      _sc(sc),
      _gains(rollController.get_gains())
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

void
POMDSoarAlgorithm::init_actions(bool mode)
{
    //gcs().send_text(MAV_SEVERITY_INFO, "Tuning: init_actions");
    _n_actions = MIN(pomdp_n_actions, MAX_ACTIONS);
    float max_roll = fmax(pomdp_roll1 * _sign, pomdp_roll2 * _sign);
    float min_roll = MIN(pomdp_roll1 * _sign, pomdp_roll2 * _sign);
	
    //gcs().send_text(MAV_SEVERITY_INFO, "init_actions");
    if (mode)
    {
        float new_max_roll = _pomdp_roll_cmd + pomdp_roll_rate;
        float new_min_roll = _pomdp_roll_cmd - pomdp_roll_rate;

        if (new_max_roll > max_roll)
        {
            new_max_roll = max_roll;
            new_min_roll = max_roll - 2 * pomdp_roll_rate;
        }

        if (new_min_roll < min_roll)
        {
            new_min_roll = min_roll;
            new_max_roll = min_roll + 2 * pomdp_roll_rate;
        }

        float roll = new_max_roll;
        float roll_rate = (new_max_roll - new_min_roll) / (_n_actions - 1);
        for (int i = 0; i < _n_actions; i++) {
            _roll_cmds[i] = roll;
            //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
            roll -= roll_rate;
        }
    }
    else
    {
        float roll = max_roll;
        float roll_rate = (max_roll - min_roll) / (_n_actions - 1);
        for (int i = 0; i < _n_actions; i++)
        {
            _roll_cmds[i] = roll;
            //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
            roll -= roll_rate;
        }
    }

    _prev_n_actions = _n_actions;
}


void
POMDSoarAlgorithm::init_thermalling()
{
    //float ground_course = radians(AP::gps().ground_course());
   // float head_sin = sinf(ground_course);
    //float head_cos = cosf(ground_course);
    //float xprod = head_cos - head_sin;
    float xprod = _sc->Perr;//轨迹误差
	gcs().send_text(MAV_SEVERITY_INFO, "Soaring: X %f", (double)xprod);
	if(xprod > 0.6f && xprod < 0.63f)
		{
		_sign = 1.0;

	}
	else 
		{
		_sign = 2.0 * _sc->Perr;

	}
   // _sign = xprod < 5 ? 1.0 : erate;
    _pomdp_roll_cmd = pomdp_roll1 * _sign;
    init_actions(pomdp_delta_mode);//动作空间
    //float aspd = _sc->get_aspd();
    //Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
    //Calculate groundspeed
   // float groundSpeed = _groundspeed_vector.length();
    float aspd = 0.01f;
    float eas2tas = _ahrs.get_EAS2TAS();
    // This assumes that SoaringController called get_position(_prev_update_location) right before this call to init_thermalling
    _pomdp_wp = _prev_update_location;
    //_ahrs.get_relative_position_NE_home(_pomdp_vecNE);
    //printf("_pomdp_wp = %f %f\n", ((double)_pomdp_wp.lat) * 1e-7, ((double)_pomdp_wp.lng) * 1e-7);
    float hdx, hdy;
    get_heading_estimate(&hdx, &hdy);
    float wind_corrected_heading = atan2f(hdy, hdx);

    // Prepare everything necessary for generating action trajectories (by modelling trajectories resulting from various commanded roll angles)
    _solver.set_pid_gains(sinf(degrees(_ahrs.roll)), cosf(degrees(_ahrs.roll)), sinf(degrees(_ahrs.yaw)), cosf(degrees(_ahrs.yaw)), sinf(degrees(_ahrs.pitch)), cosf(degrees(_ahrs.pitch)), eas2tas, _scaling_speed);
    _solver.set_polar(float(poly_a), float(poly_b), float(poly_c));
    _n_action_samples = pomdp_hori * pomdp_k;
    float pomdp_aspd = aspd;

    if (aspd_cmd > 0)
    {
        pomdp_aspd = aspd_cmd;
		/*

        if (aspd_src == 2)
        {
            pomdp_aspd -= _sc; // correct for sensor bias using wind ekf
        }
        */
    }

    _solver.generate_action_paths(pomdp_aspd, eas2tas, wind_corrected_heading, degrees(_ahrs.roll),
        degrees(get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
        pomdp_step_t, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), 0);
    _m = 0;
    _solver.init_step(pomdp_loop_load, pomdp_n, _sc->Pab, _sc->Perr, _sc->Proll, _sc->Pv, _weights, false);
    // This assumes that SoaringController updated _thermal_start_time_us right before this call to init_thermalling
    _prev_pomdp_update_time = _thermal_start_time_us;
    _prev_pomdp_wp = _prev_update_location;
    //_pomdp_active = true;
    _pomdp_mode = 0;
    _prev_pomdp_action = _sign > 0 ? _n_actions - 1 : 0;
	gcs().send_text(MAV_SEVERITY_INFO, "Soaring: pomdp_action %d", (int)_prev_pomdp_action);
	//gcs().send_text(MAV_SEVERITY_INFO, "Tuning: init_thermalling");
}

/*
float
POMDSoarAlgorithm::assess_thermalability(uint8_t exit_mode)
{
    float thermalability = -1e6;
    float aspd = _sc->get_aspd();

    if (exit_mode == 1)
    {
		// The vario type for POMDSoar *must* be 1 
        float expected_thermalling_sink = _sc->_vario->calculate_aircraft_sinkrate(radians(_pomdp_roll_cmd));
        float dist_sqr = _sc->_ekf.X[2] * _sc->_ekf.X[2] + _sc->_ekf.X[3] * _sc->_ekf.X[3];
        thermalability = (_sc->_ekf.X[0] * expf(-dist_sqr / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink;
    }
    else if (exit_mode == 2)
    {
        int n_samps = 20;
        float theta_step = fmax(pomdp_roll1, pomdp_roll2) / (float)n_samps;
        float theta = 0;

        for (int i = 0; i < n_samps; i++)
        {
            float expected_thermalling_sink = _sc->_vario->calculate_aircraft_sinkrate(radians(theta));
            float r = (aspd * aspd) / (GRAVITY_MSS * tanf(radians(theta)));
            thermalability = MAX(thermalability, (_sc->_ekf.X[0] * expf(-(r*r) / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink);
            theta += theta_step;
        }
    }
	gcs().send_text(MAV_SEVERITY_INFO, "Tuning: assess_thermalability");

    return thermalability;
}
*/
/*
bool
POMDSoarAlgorithm::are_computations_in_progress()
{
    return _pomdp_active;
}
*/

void
POMDSoarAlgorithm::update_internal_state()
{
    if (_solver.running())
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.update();
        _pomp_loop_time = AP_HAL::micros64() - start_time;
        //gcs().send_text(MAV_SEVERITY_INFO, "slice time: %lluus  samps: %d", _pomp_loop_time, samps);
        _pomp_loop_min_time = (_pomp_loop_min_time > _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_min_time;
        _pomp_loop_max_time = (_pomp_loop_max_time < _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_max_time;
        _pomp_loop_av_time = _pomp_loop_av_time * 0.9 + _pomp_loop_time * 0.1;
		//gcs().send_text(MAV_SEVERITY_INFO, "Tuning: _pomp_loop_av_time");
    }
}


void
POMDSoarAlgorithm::update_internal_state_test()
{
    _solver.update_test();
}


/*void
POMDSoarAlgorithm::stop_computations()
{
    _pomdp_active = false;
}
*/

/*
bool
POMDSoarAlgorithm::is_set_to_continue_past_thermal_locking_period()
{
    return (pomdp_pth > 0);
}
*/

int POMDSoarAlgorithm::get_curr_mode()
{
    return _pomdp_mode;
}


uint64_t POMDSoarAlgorithm::get_latest_pomdp_solve_time()
{
    return _pomdp_solve_time;
}


float
POMDSoarAlgorithm::get_action()
{
    return _solver.kout;
}



bool POMDSoarAlgorithm::update_thermalling(const Location &current_loc)
{
    bool is_ok_to_stop = true;

    if (!_solver.running())
    {
        uint8_t action = 254;
        _pomdp_solve_time = AP_HAL::micros64() - _prev_pomdp_update_time;
        //float pdx = 0;
        //float pdy = 0;
        /*gcs().send_text(MAV_SEVERITY_INFO, "Soaring: loop %dms %lluus %lluus %lluus",
        (int)((double)(AP_HAL::micros64() - _prev_pomdp_update_time) * (double)1e-3),
        _pomp_loop_av_time,
        _pomp_loop_min_time,
        _pomp_loop_max_time);*/
        //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: X %f %f %f %f", (double)_ekf.X[0], (double)_ekf.X[1], (double)_ekf.X[2], (double)_ekf.X[3]);
        //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: P %f %f %f %f", (double)_ekf.P(0, 0), (double)_ekf.P(1, 1), (double)_ekf.P(2, 2), (double)_ekf.P(3, 3));
        float eas2tas = _ahrs.get_EAS2TAS();
        float aspd = 0.01f;
        action = (uint8_t)_solver.get_best_action();
		gcs().send_text(MAV_SEVERITY_INFO, "action %f", action);
        _pomdp_roll_cmd = _roll_cmds[action];
        //pdx = _pomdp_vecNE.y;
        //pdy = _pomdp_vecNE.x;
        //_sc->get_relative_position_wrt_home(_pomdp_vecNE);
        //_ahrs.get_relative_position_NE_home(_pomdp_vecNE);
        float hdx, hdy;
        get_heading_estimate(&hdx, &hdy);
        float wind_corrected_heading = atan2f(hdy, hdx);
        //gcs().send_text(MAV_SEVERITY_INFO, "head %f %f %f", hdx, hdy, wind_corrected_heading);
        /*
        float n[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

        if (pomdp_norm_pth)
        {
            n[0] = fabs(_ekf.X[0]) > 0.0f ? fabs(_ekf.X[0]) : 1.0f;
            n[1] = fabs(_ekf.X[1]) > 0.0f ? fabs(_ekf.X[1]) : 1.0f;
            n[2] = n[1];
            n[3] = n[1];
        }

        float trP = _ekf.P(0, 0) / n[0] + _ekf.P(1, 1) / n[1] + _ekf.P(2, 2) / n[2] + _ekf.P(3, 3) / n[3];
        bool max_lift = trP < pomdp_pth && pomdp_pth > 0.0f;
        */
        float trerr = _sc->Perr;
		bool _err = trerr < 2.0f && trerr > -2.0f;
        init_actions(_err);
        int extend = 0;
        _n_action_samples = pomdp_hori * pomdp_k;

        if (_err)
        {
            extend = pomdp_extend;
            _n_action_samples = MIN(MAX_ACTION_SAMPLES, int(pomdp_hori * pomdp_k * extend));
        }

        int n_samples = pomdp_n;
        float step_w = 1.0f;

        if (_err && pomdp_plan_mode)
        {
            n_samples = 1;
            step_w = 1.0f / pomdp_n;
        }

        float pomdp_aspd = aspd;

        if (aspd_cmd > 0) {
            pomdp_aspd = aspd_cmd;
        }

        _solver.generate_action_paths(pomdp_aspd, eas2tas, wind_corrected_heading, degrees(_ahrs.roll), degrees(get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
            pomdp_step_t * step_w, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), extend);
        _m = 0;
        _solver.init_step(pomdp_loop_load, n_samples, _sc->Pab, _sc->Perr, _sc->Proll, _sc->Pv, _weights, _err);

        if (_err)
        {
            _pomdp_mode = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDP maxlift %f",(double)trerr);
        }
        else
        {
            _pomdp_mode = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDP explore %f",(double)trerr);
        }

        _prev_pomdp_update_time = AP_HAL::micros64();
        _prev_pomdp_action = action;

         is_ok_to_stop = true;
/*
        DataFlash_Class::instance()->Log_Write("POMP", "TimeUS,id,n,k,action,x,y,sign,lat,lng,rlat,rlng,roll,mode,Q", "QQHHBfffLLLLfB",
            AP_HAL::micros64(),
            _sc->_thermal_id,
            pomdp_n,
            pomdp_k,
            action,
            (double)pdx,
            (double)pdy,
            (double)_sign,
            current_loc.lat,
            current_loc.lng,
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_pomdp_roll_cmd,
            (uint8_t)max_lift,
            (double)_solver.get_action_Q(action));
        DataFlash_Class::instance()->Log_Write("POMT", "TimeUS,loop_min,loop_max,loop_time,solve_time,load,n,k", "QQQQQHHH",
            AP_HAL::micros64(),
            _pomp_loop_min_time,
            _pomp_loop_max_time,
            _pomp_loop_time,
            _pomdp_solve_time,
            pomdp_loop_load,
            pomdp_n,
            pomdp_k
        );
        DataFlash_Class::instance()->Log_Write("PDBG", "TimeUS,lat,lng,v0,psi,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6", "QLLffffffffffffff",
            AP_HAL::micros64(),
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_solver.get_action_v0(),
            (double)_solver.get_action_path_psi(action, 0),
            (double)_solver.get_action_path_x(action, 1),
            (double)_solver.get_action_path_y(action, 1),
            (double)_solver.get_action_path_x(action, 2),
            (double)_solver.get_action_path_y(action, 2),
            (double)_solver.get_action_path_x(action, 3),
            (double)_solver.get_action_path_y(action, 3),
            (double)_solver.get_action_path_x(action, 4),
            (double)_solver.get_action_path_y(action, 4),
            (double)_solver.get_action_path_x(action, 5),
            (double)_solver.get_action_path_y(action, 5),
            (double)_solver.get_action_path_x(action, 6),
            (double)_solver.get_action_path_y(action, 6)
        );
*/
        _pomdp_wp = current_loc;
        _prev_pomdp_wp = current_loc;
        _pomp_loop_min_time = (unsigned long)1e12;
        _pomp_loop_max_time = 0;
    }

	//gcs().send_text(MAV_SEVERITY_INFO, "update_thermalling");

    return is_ok_to_stop;
}

/*
void
POMDSoarAlgorithm::run_tests()
{
    if (run_timing_test == 1)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_exp_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring exp test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 2)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_fast_exp_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fast exp test: %llu us", total_time);

    }
    else if (_sc->run_timing_test == 3)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.fill_random_array();
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 4)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_rnd_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring rnd test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 5)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_ekf_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring EKF test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 6)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, false);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, true);
        uint64_t maxlift_total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring loop test: %llu us ML: %llu us", total_time, maxlift_total_time);
    }
    else if (_sc->run_timing_test == 7)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_multivariate_normal_sample_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring multivariate_normal test: %llu us", total_time);
    } // test #8 is run by the SoaringController class itself
    else if (_sc->run_timing_test == 9)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_trig_box_muller_test(1000);
        uint64_t total_trig_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_polar_box_muller_test(1000);
        uint64_t total_polar_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring box-muller trig: %llu us polar: %llu us", total_trig_time, total_polar_time);
    }

    _prev_run_timing_test = run_timing_test;
}
*/
void POMDSoarAlgorithm::get_heading_estimate(float *hdx, float *hdy) const
{
  
        Vector2f gnd_vel = _ahrs.groundspeed_vector();
        Vector3f wind = _ahrs.wind_estimate();
        *hdx = gnd_vel.x - wind.x;
        *hdy = gnd_vel.y - wind.y;
}

float POMDSoarAlgorithm::get_rate() const
{
    
      return _ahrs.get_gyro().x;
}



