#ifndef SPID_H
#define SPID_H
#include "math.h"  // Just using for signbit() function. Note signbit returns true for negative signed argument
#include <cmath>
// Here is the brake PID math
// Our target is the desired amount of the measured value. The error is what we must add to our current value to get there
// We make 3 terms P I and D which add to become our output which goes to the actuator.
// P term scales proportionally to error, however it can never reach the setpoint 
// I term steadily grows the longer the error is the same sign,
// D term counteracts fast changes from P (and I), serving to prevent overshooting target. 
// Error = Setpoint - ProcessValue
// Output  =  P + I + D  =  (K * Error) + (K / Tau_I) + (Error - LastError)

class SPID {  // Soren's home-made pid loop
  public:
    #define RANGED 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
    #define CENTERED 1  // assign to outCenterMode if pid output deviates from a centerpoint 
    #define ERROR_TERM 0  // What the proportional term is proportional_to
    #define SENSED_INPUT 1  // What the proportional term is proportional_to
    static const int32_t FWD = 1;  // Actuator_direction influences sensed value in the same direction
    static const int32_t REV = -1;  // Actuator_direction influences sensed value in the opposite direction
    bool clamp_integral = false;  // Squashes integral windup if pushing the wrong direction 
  private:
    double kp_coeff = 0.01, ki_coeff = 0.01, kd_coeff = 0.01, kp, ki_hz, kd_s;
    int32_t actuator_direction = FWD;
    bool rounding = true, open_loop = false, enabled = false, never_enabled = true;
    int32_t max_precision = 4;
    uint32_t sample_period_ms;
    double output, out_min, out_max;
    double target, target_last, error, error_last, output_last, p_term, i_term, d_term;
    double near_target_error_thresh_percent = 0.005;  // Fraction of the input range where if the error has not exceeded in either dir since last zero crossing, it is considered zero (to prevent endless microadjustments) 
    double myinput, mytarget, myoutput, input_last = 0, near_target_error_thresh = 0, near_target_lock = 0;
    double in_center = 2047, out_center = 2047;
    double* p_input; double* p_in_min; double* p_in_max; double* p_output = &output; double* p_out_min = &out_min; double* p_out_max = &out_max;
    bool out_center_mode = CENTERED, in_center_mode = CENTERED, proportional_to = ERROR_TERM, differential_of = SENSED_INPUT, saturated = false, output_hold = false;
    // Transducer* in_device; Transducer* out_device;
  public:
    // int32_t disp_kp_1k, disp_ki_mhz, disp_kd_ms;  // disp_* are integer version of tuning variables scaled up by *1000, suitable for screen display

    SPID (double* arg_input, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t direction, uint32_t arg_sample_period_ms) {  // , bool arg_in_center_mode, bool arg_out_center_mode
        p_input = arg_input;
        set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
        set_actuator_direction(direction);
        // set_center_modes(arg_in_center_mode, arg_out_center_mode);
        sample_period_ms = arg_sample_period_ms;
        printf ("SPID Constructor1: min (%ld): %4lf max (%ld): %4lf\n", p_out_min, *p_out_min, p_out_max, *p_out_max);
    }
 
 // SPID brakeSPID (&pressure_filt_psi, brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, pid_period_ms);

    SPID (double* arg_input, double* arg_output, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t direction, uint32_t arg_sample_period_ms) {  // , bool arg_in_center_mode, bool arg_out_center_mode
        p_input = arg_input;
        *p_in_min = *arg_input;  // This has to point somewhere for now until it gets set
        *p_in_max = *arg_input;  // This has to point somewhere for now until it gets set
        p_output = arg_output;
        *p_out_min = *arg_output;  // This has to point somewhere for now until it gets set
        *p_out_max = *arg_output;  // This has to point somewhere for now until it gets set
        set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
        set_actuator_direction(direction);
        // set_center_modes(arg_in_center_mode, arg_out_center_mode);
        sample_period_ms = arg_sample_period_ms;
        printf ("SPID Constructor2: min (%ld): %4lf max (%ld): %4lf\n", p_out_min, *p_out_min, p_out_max, *p_out_max, p_out_max);
    }
    // SPID(Sensor* arg_in_device, ServoPWM* arg_out_device, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t direction, uint32_t arg_sample_period_ms) {
    //     in_device = arg_in_device;
    //     p_input = &in_device->val_native;
    //     p_in_min = &in_device->min_native;  // This has to point somewhere for now until it gets set
    //     p_in_max = &in_device->max_native;  // This has to point somewhere for now until it gets set
    //     out_device = arg_out_device;
    //     p_output = &out_device->val_native;
    //     p_out_min = &out_device->min_native;  // This has to point somewhere for now until it gets set
    //     p_out_max = &out_device->max_native;  // This has to point somewhere for now until it gets set
    //     set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
    //     set_actuator_direction(out_device.get_direction());
    //     // set_center_modes(arg_in_center_mode, arg_out_center_mode);
    //     sample_period_ms = arg_sample_period_ms;
    // }
    bool constrain_value (double* value, double min, double max, double margin = 0.0) {  // Constrains referred value to given range, returning 1 if value was beyond range + margins
        bool sat = false;
        if (*value < min) {
            if (*value < min - margin) sat = true;
            *value = min;
        }
        else if (*value > max) {
            if (*value > max + margin) sat = true;
            *value = max;
        }
        return sat;
    }
    void set_input (double arg_input) {
        *p_input = round (arg_input);
        constrain_value (p_input, *p_in_min, *p_in_max);
    }
    double round (double val, int32_t digits) { return (rounding) ? (std::round (val * std::pow (10, digits)) / std::pow (10, digits)) : val; }
    double round (double val) { return round (val, max_precision); }
    double clamp_value (double arg_value) {  // If output is overshooting in the same direction as the error and the integral term is contributing, reset i_term
        bool err_sign = signbit (error) ^ (actuator_direction < 0);  // reverse error value sign if actuator direction is reversed
        if (signbit (arg_value) == err_sign) return 0;  // (&& saturated)  if sign of arg_value is the same as that of error, then zero arg_value
        return arg_value;  // otherwise return arg_value unmolested
        // Textbook says saturation is also a precondition of clamping, but I left it out because that doesn't make sense to me right now
        // if ((signbit (*p_output) == err_sign) && (signbit (arg_value) == err_sign)) // Considering also checking output sign is also the same
    }
    double compute (void) {
        myinput = *p_input;
        mytarget = target;
        myoutput = *p_output;
        error = mytarget - myinput;
        if (!enabled) {
            input_last = myinput;
            target_last = mytarget;
            error_last = error;
            output_last = myoutput;
            return myoutput;
        }
        // if (out_center_mode) myoutput = out_center;
        // else myoutput = *p_out_min;

        printf ("in:%4.0lf tg:%4.0lf er:%4.0lf i1:%4.0lf", myinput, mytarget, error, i_term);
        
        i_term += ki_coeff * error;  // Update integral
        printf (" i2:%4.0lf", i_term);

        if (clamp_integral) i_term = clamp_value(i_term);
        printf (" i3:%4.0lf", i_term);

        if (proportional_to == ERROR_TERM) {  // If proportional_to Error (default)
            // if (out_center_mode) saturated = constrain_value (&i_term, out_center-*p_out_min, *p_out_max-out_center);  // Constrain i_term before adding p_term. Todo: Pass margin into constrain() for saturation determination
            // else
            saturated = constrain_value (&i_term, *p_out_min-*p_out_max, *p_out_max-*p_out_min);  // Constrain i_term before adding p_term. Todo: Pass margin into constrain() for saturation determination
            
            printf (" i4:%4.0lf s1:%d", i_term, saturated);

            p_term = kp_coeff * error;
            printf (" p:%4.0lf", p_term);

            myoutput += i_term + p_term;
            printf (" o1:%4.0lf", myoutput);

        }
        else if (proportional_to == SENSED_INPUT) {  // If proportional_to Input
            p_term = kp_coeff * (myinput - input_last);  // p_term is based on input change (with opposite sign) rather than distance from target
            myoutput += i_term + p_term;
            saturated = constrain_value (&myoutput, *p_out_min, *p_out_max);  // Constrain combined i_term + p_term.  Todo: Pass margin into constrain() for saturation determination
        }
        d_term = -kd_coeff * (myinput - input_last);  // Note d_term is opposite sign to input change
        printf (" d:%4.0lf", d_term);

        myoutput += d_term;  // Include d_term in output
        printf (" o2:%4.0lf", *p_output);

        saturated = constrain_value (&myoutput, *p_out_min, *p_out_max);  // Constrain output to range. Todo: Pass margin into constrain() for saturation determination
        printf (" o3:%4.0lf s2:%d\n", myoutput, saturated);

        input_last = myinput;  // store previously computed input
        error_last = error;
        output_last = myoutput;
        target_last = mytarget;
        // printf (" in=%-+9.4lf lst=%-+9.4lf err=%-+9.4lf tgt=%-+9.4lf", myinput, input_last, error, mytarget);
        // printf (" pt=%-+9.4lf it=%-+9.4lf dt=%-+9.4lf out=%-+9.4lf\n", p_term, i_term, d_term, output);
        return *p_output;
    }
    // Comments related to compute() function
    // printf (" in=%-+9.4lf err=%-+9.4lf errlast=%-+9.4lf ntet=%-+9.4lf kp_co=%-+9.4lf ki_co=%-+9.4lf kd_co=%-+9.4lf kds=%-+9.4lf", input, error, error_last, near_target_error_thresh, kp_coeff, ki_coeff, kd_coeff, kd_s);
    //
    // Add a layer of history here, with an additional condition that output only holds if change in error has been small for X time period
    // Also add code to hold output (below in this function) if output_hold = true
    // if (signbit(error_last) != signbit(error) && abs(error_last - error) < near_target_error_thresh) {
    //     near_target_lock = target;
    //     output_hold = true;
    // }
    // else if (abs (near_target_lock - myinput) > near_target_error_thresh) output_hold = false;
    //
    //
    // p_term = round (p_term);
    // i_term = round (i_term);
    // d_term = round (d_term);
    //
    // Add handling for CENTERED controller!!  (our brake)
    // if (out_center_mode == CENTERED) *p_output += out_center;
    //
    // printf (" ntl2=%-+9.4lf sat=%1d outh=%1d pterm=%-+9.4lf iterm=%-+9.4lf dterm=%-+9.4lf out=%-+9.4lf\n", near_target_lock, saturated, output_hold, p_term, i_term, d_term, output);
    //
    // printf ("uc output: %7.2lf, min: %7.2lf, max:%7.2lf, ", *p_output, *p_out_min, *p_out_max);
    // printf ("c output: %7.2lf, min: %7.2lf, max:%7.2lf\n", *p_output, *p_out_min, *p_out_max);

    void set_target (double arg_target) {
        // printf ("SPID::set_target():  received arg_target=%-+9.4lf, *p_in_min=%-+9.4lf, *p_in_max=%-+9.4lf\n", arg_target, *p_in_min, *p_in_max);
        target = round (arg_target, max_precision);
        constrain_value (&target, *p_in_min, *p_in_max);
        error = target - *p_input;
    }
    void set_tunings (double arg_kp, double arg_ki_hz, double arg_kd_s) {  // Set tuning parameters to negative values if higher actuator values causes lower sensor values and vice versa 
        if (arg_kp < 0 || arg_ki_hz < 0 || arg_kd_s < 0) {  // || ( arg_kp <= 0 && arg_ki_hz <= 0 && arg_kd_s <= 0 ) ) ) {
            printf ("Warning: SPID::set_tunings() ignored request to set tuning parameters to negative values.\n");
            return;
        }
        double sample_period_s = ((double)sample_period_ms)/1000;
        kp = arg_kp;  ki_hz = arg_ki_hz;  kd_s = arg_kd_s;
        kp_coeff = arg_kp * ((proportional_to == SENSED_INPUT) ? -1 : 1);
        ki_coeff = arg_ki_hz * sample_period_s;
        if (sample_period_s) kd_coeff = arg_kd_s * ((differential_of == ERROR_TERM) ? -1 : 1) / sample_period_s;
        else {
            printf ("set_tunings refused to divide by zero to set kd_coeff\n");
            kd_coeff = 0;
        }
    }
        // printf ("dispkds=%ld", disp_kd_ms);
        // printf (", kds=%lf", kd_s);
        // printf (", speriodms=%ld", sample_period_ms);
        // printf (", speriods=%lf", sample_period_s);
        // printf (", argkds=%lf", arg_kd_s);

        // if (arg_kd_s != 0) kd_coeff = arg_kd_s /(((double)sample_period_ms)/1000);  // ??? yes, all those parentheses are needed or kd_coeff is infinite (due to div/0?)
        // else kd_coeff = 0;
        
        // printf (", kd_coeff=%lf", kd_coeff);
        // printf ("\n");

        // disp_kp_1k = (int32_t)(arg_kp * 1000);  // Scaled integer version of tuning parameters suitable for screen display
        // disp_ki_mhz = (int32_t)(arg_ki_hz * 1000);
        // disp_kd_ms = (int32_t)(arg_kd_s * 1000);
    void set_actuator_direction (int32_t direction) {
        if (direction != actuator_direction) {
            kp_coeff = (0 - kp_coeff);
            ki_coeff = (0 - ki_coeff);
            kd_coeff = (0 - kd_coeff);
            actuator_direction = direction;
        }
    }
   void set_input_limits (double* arg_p_min, double* arg_p_max) {
        if (*arg_p_min >= *arg_p_max) {
            printf ("SPID ignored attempt to set input min %lf > max %lf.\n", *arg_p_min, *arg_p_max);
            return;
        }
        p_in_min = arg_p_min;  p_in_max = arg_p_max;
        constrain_value (p_input, *p_in_min, *p_in_max);
        near_target_error_thresh = (*p_in_max - *p_in_min) * near_target_error_thresh_percent;
    }
    void set_near_target_thresh (double arg_near_target_thresh_percent) { near_target_error_thresh_percent = arg_near_target_thresh_percent; }
    void set_output_limits (double arg_min, double arg_max) {
        if (arg_min >= arg_max) {
            printf ("SPID ignored attempt to set output min %lf > max %lf.\n", arg_min, arg_max);
            printf ("Warning: SPID::set_output_limits() ignored request to set output minimum limit %lf > maximum limit %lf.\n", arg_min, arg_max);
            return;
        }
        *p_out_min = arg_min;
        *p_out_max = arg_max;
        out_center = (*p_out_min + *p_out_max)/2;
        printf ("Limit: min:%4lf max:%4lf cent:%lf\n", *p_out_min, *p_out_max, out_center);

        constrain_value (p_output, *p_out_min, *p_out_max);
    }
    void set_output_limits (double* p_arg_min, double* p_arg_max) {
        if (*p_arg_min >= *p_arg_max) {
            printf ("Warning: SPID::set_output_limits() ignored request to set output minimum limit %lf > maximum limit %lf.\n", *p_arg_min, *p_arg_max);
            return;
        }
        p_out_min = p_arg_min;
        p_out_max = p_arg_max;
        out_center = (*p_out_min + *p_out_max)/2;
        constrain_value (p_output, *p_out_min, *p_out_max);
    }
        // if (*myOutput > outMax) *myOutput = outMax;
        // else if (*myOutput < outMin) *myOutput = outMin;
        // if (outputSum > outMax) outputSum = outMax;
        // else if (outputSum < outMin) outputSum = outMin;
        // if (*myInput > inMax) *myInput = inMax;
        // else if (*myInput < inMin) *myInput = inMin;
        // if (inputSum > inMax) inputSum = inMax;
        // else if (inputSum < inMin) inputSum = inMin;
    void set_input_center (void) { in_center_mode = RANGED; }  // Call w/o arguments to set input to RANGED mode
    void set_input_center (double arg_in_center) {  // Sets input to CENTERED (centerpoint) mode and takes value of center point. 
        if (arg_in_center < *p_in_min || arg_in_center > *p_in_max) {
            printf ("Warning: SPID::set_input_center() ignored request to set input centerpoint outside input range.\n");
            return;
        }
        in_center_mode = CENTERED;
        in_center = arg_in_center;
    }
    void set_output_center (void) { out_center_mode = RANGED; }  // Call w/o arguments to set output to RANGED mode
    void set_output_center (double arg_out_center) {  // Sets output to CENTERED (centerpoint) mode and takes value of center point. 
        
        printf ("Set_Cent: min (%ld): %4lf max (%ld): %4lf cent:%lf\n", p_out_min, *p_out_min, p_out_max, *p_out_max, arg_out_center);
    
        if (arg_out_center < *p_out_min || arg_out_center > *p_out_max) {
            printf ("Warning: SPID::set_output_center() ignored request to set output centerpoint outside output range.\n");
            return;
        }
        out_center_mode = CENTERED;
        out_center = arg_out_center;
    }
    void set_pd_modes (bool arg_prop_to, bool arg_diff_of) {  // For each specify ERROR_TERM or SENSED_INPUT
        if (proportional_to != arg_prop_to) {
            kp_coeff = (0 - kp_coeff);
            proportional_to = arg_prop_to;
        }
        if (differential_of != arg_diff_of) {
            kd_coeff = (0 - kd_coeff);
            differential_of = arg_diff_of;
        }
    }    
    void set_open_loop (bool arg_open_loop) { open_loop = arg_open_loop; }
    void set_enable (bool arg_enable) {
        if (arg_enable && never_enabled) {
            if (out_center_mode) *p_output = out_center;
            else *p_output = (*p_out_max - *p_out_min)/2;
            output_last = *p_output;
            never_enabled = false;
        }
        enabled = arg_enable;
    }
    double get_kp_coeff (void) { return kp_coeff; }
    double get_ki_coeff (void) { return ki_coeff; }
    double get_kd_coeff (void) { return kd_coeff; }
    double get_kp (void) { return kp; }
    double get_ki_hz (void) { return ki_hz; }
    double get_kd_s (void) { return kd_s; }
    bool get_out_center_mode (void) { return out_center_mode; }
    bool get_open_loop (void) { return open_loop; }
    bool get_in_center_mode (void) { return in_center_mode; }
    bool get_p_mode (void) { return proportional_to; }
    bool get_d_mode (void) { return differential_of; }
    bool get_actuator_direction (void) { return actuator_direction; }
    double get_near_target_error_thresh_percent (void) { return near_target_error_thresh_percent; }
    double get_near_target_error_thresh (void) { return near_target_error_thresh; }
    bool get_out_center (void) { return out_center; }
    bool get_in_center (void) { return in_center; }
    bool get_saturated (void) { return saturated; }
    double get_p_term (void) { return p_term; }  // ((p_term >= 0.001) ? p_term : 0); }
    double get_i_term (void) { return i_term; }  // ((i_term >= 0.001) ? i_term : 0); }
    double get_d_term (void) { return d_term; }  // ((d_term >= 0.001) ? d_term : 0); }
    double get_error (void) { return error; }
    double get_target (void) { return target; }
    double get_output (void) { return *p_output; }
    double get_input (void) { return *p_input; }
    bool get_enabled (void) { return enabled; }    
};

// Instantiate PID loops
//
// Steering:  Motor direction and velocity are controlled with PWM, proportional to joystick horizontal direction and magnitude
//   Setpoint Value: Proportional to Joystick Horz ADC value.  0V = Full Left, 2.5V = Stop, 5V = Full Right
//   Measured Value: We have no feedback, other than the joystick current horizontal position
//   Actuator Output Value: PWM square wave sent to Jaguar, w/ high pulse width of: ~2.2ms = Full Left, 1.5ms = Stop, ~800us = Full Right
//   Limits: Reed switch limit signals for left and right may be handled by us, or by the jaguar controller
//   Setpoint scaling: Kp/Ki/Kd values should decrease appropriately as a function of vehicle speed (safety) 
//
//   Notes: The steering has no feedback sensing, other than two digital limit switches at the ends of travel.  
//   So just consider the error to be the difference between the joystick position and the last output value.
//
// Brakes:  Motor direction & velocity are controlled with PWM until brake pressure matches pressure setpoint
//   Setpoint Value: * Default: Pressure setpoint proportional to Joystick Vert distance from center when below center.
//       * In Hold Mode: Brake adjusts automatically to keep car stopped, as long as joystick below center
//       * In Cruise Mode: Brake is kept released 
//   Measured Value: Analog voltage from brake fluid pressure sensor. 0-3.3V proportional to 0-1000psi
//   Actuator Output Value: PWM signal to Brake Jaguar unit.
//       0% duty = Full speed extend (less brake), 50% = Stop, 100% = Full speed Retract (more brake)
//   Position: Analog 0-3.3V proportional to the travel length of the actuator (not used as feedback)
//
// Gas:  Servo angle is adjusted with PWM until engine rpm matches rpm target setpoint
//   Setpoint Value: * Default: RPM Setpoint proportional to Joystick Vert distance from center when above center.
//       * In Cruise Mode: Upward or downward joy vert motions modify vehicle speed setpoint
//                          Gas pid setppoints are output from cruise pid
//   Measured Value: * Default: Engine speed determined from tach pulses
//   Actuator Output Value: PWM signal to throttle servo
//       0% duty = Fully close throttle.  This will idle.  100% duty = Fully open throttle.
//
// Cruise:
//   Setpoint Value: * Default: Set to the current vehicle speed when mode is entered.
//       * In Cruise Mode: Upward or downward joy vert motions suspend loop and accelerate or decelerate,
//                         upon return to center loop resumes with new speed target set to vehicle speed when released.
//   Measured Value: * Vehicle speed determined from pulley sensor pulses
//   Actuator Output Value: Cruise PID output values become setpoint values for the Gas PID above
//       0% duty = Car stopped.  100% duty = Car max speed.
//
// One way to tune a PID loop:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate
// 3) Increase Ki until oscillation stops
// 4) If improved response time is needed, increase Kd slightly and go back to step 2
//
// Ziegler-Nichols method:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate.
// 3) Record Kc = critical value of Kp, and Pc = period of oscillations
// 4) Set Kp=0.6*Kc and Ki=1.2*Kc/Pc and Kd=Kc*Pc/13.33  (Or for P only:  Kp=Kc/2)  (Or for PI:  Kp=0.45*Kc and Ki=0.54*Kc/Pc)

// If output constrain prevents us from exceeding the limits of the actuator. But we need to know two things as we constrain, to preevent "Windup", a condition where the I term exploded due to an extended error when maybe the motor couldn't meet the target. Once back to normal, don't want I term wound up.
// So improve this Clamp to check for A. Is it saturating? I.e. was constrain necessary or not? and B. Is the sign (polarity) of the output the same as that of the error?  If both are true, we have Integrator Windup.  So when we detect this, we can temporarily "Clamp" the I-term to 0 until we are "recovered".
// Recovered can be either of these conditions:  1. We are no longer saturated (constrain is doing nothing)m or, 2. The error changes sign. (then reconnect I term.)
// When determining saturation or not, add a margin.

#endif  // SPID_H