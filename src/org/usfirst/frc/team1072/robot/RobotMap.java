/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Represents all the global constants to be used in the code.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class RobotMap {
    public static int LEFT_CIM_TALON = 5;
    public static int RIGHT_CIM_TALON = 2;
    public static int LEFT_CIM_VICTOR = 4;
    public static int RIGHT_CIM_VICTOR = 3;
    public static int ELEVATOR_TALON = 9; // bottom right
    public static int ELEVATOR_VICTOR_TOPLEFT = 7;
    public static int ELEVATOR_VICTOR_BOTTOMLEFT = 8;
    public static int ELEVATOR_VICTOR_TOPRIGHT = 10;
    public static int INTAKE_TALON_LEFT = 6;
    public static int INTAKE_TALON_RIGHT = 1;
    public static int DT_PIGEON_PORT = 1;
    
    public static int JOYSTICK = 0;
    
    public static int TIMEOUT = 10;
    
    public static int DT_VEL_PID = 1;
    public static int DT_POS_PID = 0;
    public static int DT_ANGLE_PID = 2;
    public static int DT_MOTION_PROFILE_PID = 3;
    
    public static double MAX_RAMP_TIME = 0;
    
    public static double NOMINAL_BATTERY_VOLTAGE = 11.0; 
    
    public static double PEAK_OUTPUT_LEFT = 1.0;
    public static double PEAK_OUTPUT_RIGHT = 1.0;
    public static double NOMINAL_OUTPUT_LEFT = 0.04;
    public static double NOMINAL_OUTPUT_RIGHT = 0.04; 
    public static double DRIVETRAIN_SCALE = 1;
    
    public static boolean DT_LEFT_TALON_PHASE = false;
    public static boolean DT_RIGHT_TALON_PHASE = false;
    public static boolean EL_TALON_PHASE = true;

    public static boolean EL_BOTTOM_LEFT_VICTOR_INVERT = true;
    public static boolean EL_TOP_LEFT_VICTOR_INVERT = false;
    public static boolean EL_TALON_INVERT = false;
    public static boolean EL_TOP_RIGHT_VICTOR_INVERT = false;
    
    // CONCRETE CONSTANTS
    /*public static double VEL_KF_LEFT = .197; //0.178
    // when modifying KP, double until disastrous
    public static double VEL_KP_LEFT = 0; //0.3;
    public static double VEL_KI_LEFT = 0;//0;
    public static double VEL_KD_LEFT = 0; //10;
    
    public static double VEL_KF_RIGHT = .188; //0.18
    public static double VEL_KP_RIGHT = 0;//0.3;
    public static double VEL_KI_RIGHT = 0;//0;
    public static double VEL_KD_RIGHT = 0;//10;*/
    
    // CARPET CONSTANTS
    public static double DT_VEL_KF_LEFT = 0.197; //0.178
    // when modifying KP, double until disastrous
    public static double DT_VEL_KP_LEFT = 0.5; //0.3;
    public static double DT_VEL_KI_LEFT = 0;//0;
    public static double DT_VEL_KD_LEFT = 0; //10;
    
    public static double DT_VEL_KF_RIGHT = 0.197; //0.18
    public static double DT_VEL_KP_RIGHT = 0.5;//0.3;
    public static double DT_VEL_KI_RIGHT = 0;//0;
    public static double DT_VEL_KD_RIGHT = 0;//10;
    
    public static double EL_SLOW_DOWN_POS = 2000;
    
    // tune (double) until oscillating, then tune KD
    // CONCRETE
//    public static double POS_KF_LEFT = 0;
//    public static double POS_KP_LEFT = 0.1; //0.2 
//    public static double POS_KI_LEFT = 0.000004; //
//    public static double POS_KD_LEFT = 25;
//    
//    public static double POS_KF_RIGHT = 0;
//    public static double POS_KP_RIGHT = 0.1;
//    public static double POS_KI_RIGHT = 0.000004; // 
//    public static double POS_KD_RIGHT = 25;
    
    // CARPET 
    public static double DT_POS_KF_LEFT = 0;
    public static double DT_POS_KP_LEFT = 0.09;//0.22; //0.2 
    public static double DT_POS_KI_LEFT = 0.000015;//.0008;//0.00001;//0.00001; //
    public static double DT_POS_KD_LEFT = 3.5;//40;//40;
    
    public static double DT_POS_KF_RIGHT = 0;
    public static double DT_POS_KP_RIGHT = 0.09;;//0.05;//0.22;
    public static double DT_POS_KI_RIGHT = 0.000015;//0.00001;//0.00001; // 
    public static double DT_POS_KD_RIGHT = 3.5;//40;
    
    public static int DT_POS_ALLOWABLE_ERROR = 100;
    public static int DT_POS_TOLERANCE_BUFFER = 100;
    
    public static int TICKS_PER_REV = 4096;
    public static double WHEELDIAMETER = 4.0;
    
    public static double MAX_DRIVE_SPEED_FPS = 14.0;
    public static double MAX_TURN_SPEED_FPS = 14; 
    
    public static int DT_PEAK_CURRENT_LIMIT = 60;
    public static int DT_PEAK_TIME_MS = 200;
    public static int DT_CONTINUOUS_CURRENT_LIMIT = 40;
    
    
    
    public static final int COMPRESSOR_PORT = 0;
    
    public static final int FIRST_PCM_ID = 0;
    
    public static final int INTAKE_DOWN_SOL = 1;
    public static final DoubleSolenoid.Value INTAKE_DOWN = DoubleSolenoid.Value.kReverse;
    
    public static final int INTAKE_UP_SOL = 3;
    public static final DoubleSolenoid.Value INTAKE_UP = DoubleSolenoid.Value.kForward;
    
    public static final int INTAKE_COMPRESS_SOL = 0;
    public static final DoubleSolenoid.Value INTAKE_COMPRESS = DoubleSolenoid.Value.kReverse;
    
    public static final int INTAKE_DECOMPRESS_SOL = 2;
    public static final DoubleSolenoid.Value INTAKE_DECOMPRESS = DoubleSolenoid.Value.kForward;
    
    public static final String INTAKE_UPDOWN_KEY = "UPDOWN";
    public static final String INTAKE_COMPRESSDECOMPRESS_KEY = "COMPRESSDE";
    
    //I AM A GOON LIKE MR. MUCKY BECAUSE I GET DISTRACTED VERY EASILY
    // in autonomous, adds this value to all trajectory times
    public static final int AUTON_BASE_PERIOD = 0;
    
    
    public static int EL_MANUAL_CURRENT_LIMIT_CONT = 10;
    
    public static int EL_PEAK_CURRENT_LIMIT = 10; //25
    public static int EL_PEAK_TIME_MS = 750;
    public static int EL_CONTINOUS_CURRENT_LIMIT = 5; //15
    
    public static int EL_VEL_PID = 0;
    public static int EL_POS_PID = 1;
    
    public static double EL_NOMINAL_OUTPUT = 0;
    public static double EL_PEAK_OUTPUT = 1.0;
    
    public static double EL_POS_FGRAV = 0.12; 
    public static double EL_POS_KF = 0;
    public static double EL_POS_KP = 0.1;
    public static double EL_POS_KI = 0.0001;
    public static double EL_POS_KD = 18;
    public static int EL_POS_ALLOWABLE_ERROR = 75;
    
    public static int EL_FORWARD_SOFT = 34500;
    public static int EL_REVERSE_SOFT = 2000;
    public static double EL_RAMP_RATE = 0.75;
    
    // elevator max RPM: 500 RPM
    public static double EL_VEL_KF = 0.37;
    public static double EL_VEL_KP = 0.1;
    public static double EL_VEL_KI = 0.0001;
    public static double EL_VEL_KD = 14;
    
    public static int EL_VEL_ACCEL =  5000; // 9000
    public static int EL_VEL_VEL = 1500; //2600
    public static int EL_VEL_ALLOWABLE_ERROR = 100;
    
    public static int EL_INTAKE_HEIGHT = 1000;
    public static int EL_SWITCH_HEIGHT = 10000;
    public static int EL_SCALE_LOW_HEIGHT = 20000;
    public static int EL_SCALE_HIGH_HEIGHT = 34000;
    
    public static int EL_MANUAL_VEL_KF = 0;
    public static int EL_MANUAL_VEL_KP = 0;
    public static int EL_MANUAL_VEL_KI = 0;
    public static int EL_MANUAL_VEL_KD = 0;
            
    public static int PIGEON_TURN_TRAVEL_PER_ROTATION = 3600;
    public static int PIGEON_UNITS_PER_ROTATION = 8192;
    public static int REMOTE_0 = 0;
    public static int REMOTE_1 = 1;

    public static double PIGEON_ANGLE_KP = 1.3;
    public static double PIGEON_ANGLE_KI = 0.001;
    public static double PIGEON_ANGLE_KD = 75;
    
    public static int INT_PEAK_CURRENT_LIMIT = 15;
    public static int INT_PEAK_TIME_MS = 2000;
    public static int INT_CONTINUOUS_CURRENT_LIMIT = 5;
    
    public static boolean NO_MANUAL_INTAKE = false;
    public static boolean INTAKE_BOOL = true;
    public static boolean OUTTAKE_BOOL = false;
    // Conversion constants
    public static final double DEGREES_PER_RADIAN = 180 / Math.PI;
    public static final int MS_PER_SEC = 1000;
    
    public static final int PRIMARY_PID_INDEX = 0;
    public static final int AUXILIARY_PID_INDEX = 1;
    public static final int PIGEON_ID = 1;
    public static final int PIGEON_PERIOD_MS = 4;
    public static final int ANGLE_INTEGRAL_BAND = 300;
    public static final int MAX_TALON_FRAME_PERIOD_MS = 160;
    public static final double TALON_ENCODER_SUM_PERIOD_MS = 0.7;
    public static final boolean ENABLE_NOTIFIER = true;
    //public static final double BUFFER_NEXT_OUTPUT_PERIOD = 1.0/1000;
    public static final int TIME_PER_TRAJECTORY_POINT_MS = 50;
    public static final double PID_OUTPUT_PERIOD_S = 3.0/1000;
    public static final double TALON_MOTOR_OUTPUT_UNITS = 1023;
    public static final RemoteFeedbackDevice PIGEON_REMOTE_SENSOR_TYPE = RemoteFeedbackDevice.RemoteSensor0;
    public static final int NUM_PID_SLOTS = 4;

    
    // motion profiling constants
    // CONCRETE
//    public static double DT_MOTION_PROF_KF_LEFT = .182; //0.197
//    public static double DT_MOTION_PROF_KP_LEFT = 0.1; //0.2 
//    public static double DT_MOTION_PROF_KI_LEFT = 0.000004; //
//    public static double DT_MOTION_PROF_KD_LEFT = 25;
//    
//    public static double DT_MOTION_PROF_KF_RIGHT = .178; //0.188
//    public static double DT_MOTION_PROF_KP_RIGHT = 0.1;
//    public static double DT_MOTION_PROF_KI_RIGHT = 0.000004; // 
//    public static double DT_MOTION_PROF_KD_RIGHT = 25;
    
    public static double DT_MOTION_PROF_KF_LEFT = DT_VEL_KF_LEFT; //0.197
  public static double DT_MOTION_PROF_KP_LEFT = 0.22; //0.2 
  public static double DT_MOTION_PROF_KI_LEFT = 0.00001; //
  public static double DT_MOTION_PROF_KD_LEFT = 40;
  
  public static double DT_MOTION_PROF_KF_RIGHT = DT_VEL_KF_RIGHT; //0.188
  public static double DT_MOTION_PROF_KP_RIGHT = 0.22;
  public static double DT_MOTION_PROF_KI_RIGHT = 0.00001; // 
  public static double DT_MOTION_PROF_KD_RIGHT = 40;
    
    
    //public static int TIME_PER_TRAJECTORY_POINT_MS = 50;
}
