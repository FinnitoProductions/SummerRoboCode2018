/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Represents all the global constants to be used in the code.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class RobotMap {
    
    public static boolean IS_COMP = false;
    public static int JOYSTICK = 0;
    
    public static int TIMEOUT = 10;
    
    public static double NOMINAL_BATTERY_VOLTAGE = 11.0; 
    

    // in autonomous, adds this value to all trajectory times
    public static final int AUTON_BASE_PERIOD = 0;
    
    /**
     * A wrapper class to house all CAN IDs.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class CAN_IDs 
    {

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
        
        public static int PIGEON = 1;
    }
    
    /**
     * A wrapper class to house all of the drivetrain-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class DrivetrainConstants
    {
        public static final double MOT_PROF_ADD_TO_VEL_INIT = 7;
        public static final double TIME_TO_OVERCOME_S_FRICTION_MS = 45;
        public static final double TALON_DEADBAND = 0.01;
        public static final boolean LEFT_TALON_INVERT = true;
        public static final boolean LEFT_VICTOR_INVERT = true;
        public static int VEL_PID = 1;
        public static int POS_PID = 2;
        public static int ANGLE_PID = 0;
        public static int MOTION_PROFILE_PID = 3;
        
        public static double MAX_RAMP_TIME = 0;
        public static double NOMINAL_OUTPUT_LEFT = 0.04; //0.084;
        public static double NOMINAL_OUTPUT_RIGHT = 0.04; //0.084;
        public static double PEAK_OUTPUT_LEFT = 1;//1.0;
        public static double PEAK_OUTPUT_RIGHT = 1;//1.0;
        public static double DRIVETRAIN_SCALE = 1;
        
        public static boolean LEFT_TALON_PHASE = false;
        public static boolean RIGHT_TALON_PHASE = false;
        
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
        public static double VEL_KF_LEFT = 0.197; //0.178
        // when modifying KP, double until disastrous
        public static double VEL_KP_LEFT = 0.5; //0.3;
        public static double VEL_KI_LEFT = 0;//0;
        public static double VEL_KD_LEFT = 0; //10;
        
        public static double VEL_KF_RIGHT = 0.197; //0.18
        public static double VEL_KP_RIGHT = 0.5;//0.3;
        public static double VEL_KI_RIGHT = 0;//0;
        public static double VEL_KD_RIGHT = 0;//10;
        
        // tune (double) until oscillating, then tune KD
        // CONCRETE
//        public static double POS_KF_LEFT = 0;
//        public static double POS_KP_LEFT = 0.1; //0.2 
//        public static double POS_KI_LEFT = 0.000004; //
//        public static double POS_KD_LEFT = 25;
    //    
//        public static double POS_KF_RIGHT = 0;
//        public static double POS_KP_RIGHT = 0.1;
//        public static double POS_KI_RIGHT = 0.000004; // 
//        public static double POS_KD_RIGHT = 25;
        // CARPET 
        public static double POS_KF_LEFT = 0;
        public static double POS_KP_LEFT = 0.19;//0.22; //0.2 
        public static double POS_KI_LEFT = 0.00001;//0.00001;//0.001;//.0008;//0.00001;//0.00001; //
        public static double POS_KD_LEFT = 16;//40;//40;
        public static int POS_IZONE_LEFT = 250;
        
        public static double POS_KF_RIGHT = 0;
        public static double POS_KP_RIGHT = 0.19;
        public static double POS_KI_RIGHT = 0.00001;//0.00001;//0.001;//0.00001;//0.00001; // 
        public static double POS_KD_RIGHT = 16;//40;
        public static int POS_IZONE_RIGHT = 250;
        
        public static int POS_ALLOWABLE_ERROR = 650;
        public static int POS_TOLERANCE_BUFFER = 150;
        
        public static double WHEELDIAMETER = 4.0;
        public static double MAX_DRIVE_SPEED_FPS = 14.0;
        public static double MAX_TURN_SPEED_FPS = 14;
        
        public static int PEAK_CURRENT_LIMIT = 60;
        public static int PEAK_TIME_MS = 200;
        public static int CONTINUOUS_CURRENT_LIMIT = 40;
        
        
        // motion profiling constants
        // CONCRETE
//        public static double DT_MOTION_PROF_KF_LEFT = .182; //0.197
//        public static double DT_MOTION_PROF_KP_LEFT = 0.1; //0.2 
//        public static double DT_MOTION_PROF_KI_LEFT = 0.000004; //
//        public static double DT_MOTION_PROF_KD_LEFT = 25;
    //    
//        public static double DT_MOTION_PROF_KF_RIGHT = .178; //0.188
//        public static double DT_MOTION_PROF_KP_RIGHT = 0.1;
//        public static double DT_MOTION_PROF_KI_RIGHT = 0.000004; // 
//        public static double DT_MOTION_PROF_KD_RIGHT = 25;
        // CARPET
        public static double MOTION_PROF_KF_LEFT = 0;//VEL_KF_LEFT; //0.197
        public static double MOTION_PROF_KP_LEFT = 0;//0.22; //0.2 
        public static double MOTION_PROF_KI_LEFT = 0;//0.00001; //
        public static double MOTION_PROF_KD_LEFT = 0;//40;
        
        public static double MOTION_PROF_KF_RIGHT = 0.235;//0.38;//0.38;//207; //0.197
        public static double MOTION_PROF_KP_RIGHT = 0.8;//4.5 / 10;//4;//0.5;//0.22 / 2; //0.1
        public static double MOTION_PROF_KI_RIGHT = 0.0001; //0.00015;//0.000015;//0.00023 / 10;//0.00001;//.00001; // //0.00004 
        public static double MOTION_PROF_KD_RIGHT = 4;//8; //3.5//6;//10;//10;//10;//40; //25
     
        
        
    }
    
    /**
     * A wrapper class to house all of the elevator-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class ElevatorConstants
    {
        public static final double SWITCH_HEIGHT_AUTON = 4600;
        public static final double SWITCH_HEIGHT_THIRD_CUBE = 800;
        public static int MANUAL_CURRENT_LIMIT_CONT = 10;
        public static int PEAK_CURRENT_LIMIT = 10; //25
        public static int PEAK_TIME_MS = 750;
        public static int CONTINOUS_CURRENT_LIMIT = 5; //15
        
        public static int VEL_PID = 0;
        public static int POS_PID = 1;
        
        public static double NOMINAL_OUTPUT = 0;
        
        public static double POS_FGRAV = 0.12;
        public static double PEAK_OUTPUT = 1.0;
        
        public static double POS_KF = 0;
        public static double POS_KP = 0.1;
        public static double POS_KI = 0.0001;
        public static double POS_KD = 18;
        
        public static int POS_ALLOWABLE_ERROR = 254;
        public static int FORWARD_SOFT = 34500;
        public static int REVERSE_SOFT = 2500;
        public static double RAMP_RATE = 0.75;
        
        // elevator max RPM: 500 RPM
        public static double VEL_KF = 0.37;
        public static double VEL_KP = 0.1;
        public static double VEL_KI = 0.0001;
        public static double VEL_KD = 14;
        
        public static int VEL_ACCEL = 9000 / 4;
        public static int VEL_VEL = 2600 / 4;
        public static int VEL_ALLOWABLE_ERROR = 100;
        
        public static int INTAKE_HEIGHT = 300;
        public static int SWITCH_HEIGHT = 10000;
        public static int SCALE_LOW_HEIGHT = 20000;
        public static int SCALE_HIGH_HEIGHT = 34000;
        
        public static int MANUAL_VEL_KF = 0;
        public static int MANUAL_VEL_KP = 0;
        public static int MANUAL_VEL_KI = 0;
        public static int MANUAL_VEL_KD = 0;
        
        public static double SLOW_DOWN_POS = REVERSE_SOFT + 2000;
        public static boolean BOTTOM_LEFT_VICTOR_INVERT = true;
        public static boolean TOP_LEFT_VICTOR_INVERT = false;
        public static boolean TALON_INVERT = false;
        public static boolean TOP_RIGHT_VICTOR_INVERT = false;
        public static boolean TALON_PHASE = true;
        
    }
    
    /**
     * A wrapper class to house all of the intake-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class IntakeConstants
    {
        public static final int COMPRESSOR_PORT = 0;
        public static final int FIRST_PCM_ID = 0;
        
        public static final int INTAKE_DOWN_SOL = 1;
        public static final DoubleSolenoid.Value DOWN = DoubleSolenoid.Value.kReverse;
        public static final int INTAKE_UP_SOL = 3;
        public static final DoubleSolenoid.Value UP = DoubleSolenoid.Value.kForward;
        public static final int INTAKE_COMPRESS_SOL = 0;
        public static final DoubleSolenoid.Value COMPRESS = DoubleSolenoid.Value.kReverse;
        public static final int INTAKE_DECOMPRESS_SOL = 2;
        public static final DoubleSolenoid.Value DECOMPRESS = DoubleSolenoid.Value.kForward;
        
        public static final String UPDOWN_KEY = "UPDOWN";
        public static final String COMPRESSDECOMPRESS_KEY = "COMPRESSDE";
        public static final double INTAKE_CURRENT_SPIKE = 10;
        
        public static int PEAK_CURRENT_LIMIT = 15;
        public static int PEAK_TIME_MS = 2000;
        public static int CONTINUOUS_CURRENT_LIMIT = 5;
        
        public static boolean NO_MANUAL_INTAKE = false;
        public static boolean INTAKE_BOOL = true;
        public static boolean OUTTAKE_BOOL = false;
    }
    
    /**
     * A wrapper class to house all of the Pigeon-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class PigeonConstants
    {
        public static double MOT_PROF_KF = 0;
        public static double MOT_PROF_KP = 1.1; // 1.3
        public static double MOT_PROF_KI = 0;//.001;
        public static double MOT_PROF_KD = 7;//75;
        
        public static double TURN_KF = 0.5;//1.45;//0.6; // 1.28
        public static double TURN_KP = 3.6; //1.3;//0.2;//0.5;//0.3; 
        public static double TURN_KI = 0.003; //0.002;//0.0001;//0.007;//0.001;//0;//.001;
        public static double TURN_KD = 1; //4;//4;//25;//100
        public static int TURN_IZONE = 150; //150;
        public static int TURN_VEL = 500;
        public static int TURN_ACCEL = 600;
        public static double NOMINAL_OUTPUT_LEFT = 0.11; //0.084;
        public static double NOMINAL_OUTPUT_RIGHT = 0.11;
        
        public static final int PERIOD_MS = 4;
        public static final int INTEGRAL_BAND = 150; // where the integral accumulator should be reset
        public static final RemoteFeedbackDevice REMOTE_SENSOR_SLOT = RemoteFeedbackDevice.RemoteSensor0;
        public static final int ANGLE_ALLOWABLE_ERROR = 300;
        
        public static final boolean LEFT_SENSOR_PHASE = false;
        public static final boolean RIGHT_SENSOR_PHASE = false;
        
        public static final double SET_YAW_SCALE_FACTOR =  64;//.01562881562882;
    }
    
    /**
     * Stores the file paths for every trajectory to be loaded in.
     * 
     * @author Finn Frankis
     * @version Jul 13, 2018
     */
    public static class AutonomousConstants
    {
        public static final String CLH_P1_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/1f/part1 _left_detailed.csv";
        public static final String CLH_P1_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/1f/part1 _right_detailed.csv";
        
        public static final String CLH_P2_LEFT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/2b/part2_left_detailed.csv";
        public static final String CLH_P2_RIGHT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/2b/part2_right_detailed.csv";
        
        public static final String CLH_P3_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/3f/part3_left_detailed.csv";
        public static final String CLH_P3_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/3f/part3_right_detailed.csv";

        public static final String CLH_P4_LEFT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/4b/part4_left_detailed.csv";
        public static final String CLH_P4_RIGHT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/4b/part4_right_detailed.csv";
        
        public static final String CLH_P5_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/5f/part5 _left_detailed.csv";
        public static final String CLH_P5_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/5f/part5 _right_detailed.csv";
        
        public static final String CLH_P6_LEFT_REV = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/6b/part6_left_detailed.csv";
        public static final String CLH_P6_RIGHT_REV = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/6b/part6_right_detailed.csv";
        
        public static final String CLH_P7_LEFT = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/7f/part7_left_detailed.csv";
        public static final String CLH_P7_RIGHT = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/7f/part7_right_detailed.csv";
        
        public static final String CLH_P8_LEFT_REV = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/8b/part8_left_detailed.csv";
        public static final String CLH_P8_RIGHT_REV = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/8b/part8_right_detailed.csv";
        
        public static final String CLH_P9_LEFT = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/9f/part9_left_detailed.csv";
        public static final String CLH_P9_RIGHT = "/home/summer2018/paths/3_cube/center_left_headon(switch_3)/9f/part9_right_detailed.csv";
        
        public static final double DISTANCE_TO_SCALE = 26;
        public static final double ANGLE_FROM_SCALE_TO_CUBES = 135;
    }

    public static int REMOTE_SLOT_0 = 0;
    public static int REMOTE_SLOT_1 = 1;

    public static final int PRIMARY_PID_INDEX = 0;
    public static final int AUXILIARY_PID_INDEX = 1;
    public static final int PIGEON_ID = 1;

    public static final int TIME_PER_TRAJECTORY_POINT_MS = 10;
    public static final double TALON_MOTOR_OUTPUT_UNITS = 1023;
    public static final int NUM_PID_SLOTS = 4;
    public static final int MAX_TALON_FRAME_PERIOD_MS = 160;

    public static final int MOTION_PROFILE_ALLOWABLE_ERROR = 500;
}
