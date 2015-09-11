//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : cEnsemble.h : specific Ensemble Axis interface
//-------------------------------------------------------------------

#ifndef __C_ENSEMBLE_H__
#define __C_ENSEMBLE_H__

#include "AbstractAerotech.h"
#include <iostream>
#include <yat/Exception.h>


namespace Aerotech_ns
{

class cEnsemble : public AbstractAerotech
{
public : 
	cEnsemble (char *axis_name);
	virtual ~cEnsemble ();
	/**
	* All the following functions are explained in "AbstractAerotech.h"
	**/

private:
	bool get_param (char *param,char *string_received);
	bool set_param (char *param,char *string_to_send);

public:

	bool axis_move_abs (double target);
	bool axis_move_rel (double target);
	bool get_axis_position (double &position);
	bool get_axis_position_error (double &position);
    
	bool get_axis_fault_status ( int &fault_status);
    bool get_axis_status ( int &status);
	void error_to_string( int fault_status,char *explanation);
    
    bool get_axis_velocity_feedback (double &velocity_vfbk);
    bool get_axis_velocity_command (double &velocity_vcmd);
	bool set_axis_velocity (double velocity);
    
    //- these 2 methods are not implemented for ensemble
    bool get_axis_velocity (double &velocity){} //- declared for interface compliance
    bool set_axis_default_speed (double speed){} //- declared for interface compliance
    
	bool set_axis_software_limit_low (double limit);
	bool set_axis_software_limit_high (double limit);

	bool set_axis_ramp_rate (double ramp_rate);
	bool get_axis_ramp_rate (double &ramp_rate);
    
	bool axis_brake_on (void);
	bool axis_brake_off (void);
    
	bool get_digital_input (int &state);  
	bool set_digital_output (int state);
	bool get_analog_input (double &state);
	bool set_analog_output (double state);

	bool axis_is_enabled (void);
	bool axis_is_moving (void);
	bool axis_is_homing (void);
	bool axis_is_homed (void);
	bool axis_calibration_is_active (void);
	bool axis_is_in_position (void);
	bool axis_is_accelerating (void);
	bool axis_is_decelerating (void);
	bool axis_is_accel_or_decel (void);
	bool axis_is_cw_EOT (void);
	bool axis_is_ccw_EOT (void);
	bool axis_is_EOT (void);
	bool axis_is_emergency_stop (void);
	bool axis_is_brake_on (void);
	bool axis_is_brake_off (void);

	/**
	* All the following functions are only usable with "cEnsemble" class
	**/

	/**
	*  The "reset" function resets the controller.
	*  The communication is lost
	*  After this command, to communicate again: call the destructor and build a new object
	*   or call "close_socket" then "create_socket"
	**/
	bool reset(void);

	/**
	*  The function "commit_parameters" commits the value of parameters.
	*  It is important to call "commit_parameters" before using "reset" function
	**/
	bool commit_parameters(void);

	bool axis_is_sense_plus (void);

};

} //- namespace

#endif // __C_ENSEMBLE_H__

