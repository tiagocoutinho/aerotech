//--------------------------------------------------------------------------------
//- Project : Aerotech
//- Abstract : control of Aerotech Axes drivers Ensemble and A3200
//- file : cA3200.h
//- implementation of AbstractAxis for A3200 drivers
//--------------------------------------------------------------------------------
#ifndef __C_A3200_H__
#define __C_A3200_H__

#include "AbstractAerotech.h"
#include <yat/Exception.h>
#include <iostream>


namespace Aerotech_ns
{

  class cA3200 : public AbstractAerotech
  {
  public:

    cA3200(char * axis_name);
    virtual ~cA3200(void);

  private :
    bool set_param(char *param,char *string_to_send);
    bool get_param(char *param,char *string_received);

  public :
    //- pure virtual AbstractAxis methods implementation
    bool axis_move_abs(double target);
    bool axis_move_rel(double target);
    bool get_axis_position(double &position);
    bool get_axis_position_error(double &position);
    bool get_axis_fault_status( int &fault_status);
    void error_to_string( int fault_status,char *explanation);
    bool get_axis_velocity(double &velocity);
    bool set_axis_velocity(double velocity);
    bool set_axis_software_limit_low(double limit);
    bool set_axis_software_limit_high(double limit);
    bool set_axis_default_speed(double speed);
    bool set_axis_ramp_rate(double ramp_rate);
    bool get_axis_ramp_rate(double &ramp_rate);
    bool axis_brake_on(void);
    bool axis_brake_off(void);
    bool get_axis_status( int &status);
    bool get_drive_status( int &status);
    bool get_digital_input(int &state);
    bool set_digital_output(int state);
    bool get_analog_input(double &state);
    bool set_analog_output(double state);

    bool axis_is_enabled(void);
    bool axis_is_moving (void);
    bool axis_is_homing (void);
    bool axis_is_homed (void);
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
    * The following functions are only usable with "cA3200" class
    * In this device an arry or 255 doubles is available as aglobal register
    * to set a double use setdoubleregister
    * to read it use getdoubleregister
    **/
    bool setdoubleregister(int number, double value);
    bool getdoubleregister(int number, double &value);

  private:
    //- declared in AbstractAerotech.h
    //- double target_speed;

  };
}

#endif // __C_A3200_H__
