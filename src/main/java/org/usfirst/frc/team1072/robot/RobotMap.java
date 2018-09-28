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
    /**
     * Whether or not the robot is in competition mode.
     */
    public static boolean IS_COMP = false;
    
    /**
     * Whether or not two controllers are being used to control the robot.
     */
    public static boolean TWO_CONTROLLERS = true;
    
    /**
     * The joystick port.
     */
    public static int JOYSTICK = 0;
    
    /**
     * The global time out for which CAN commands will stop attempting to send (in ms).
     */
    public static int TIMEOUT = 10;
    
    /**
     * The assumed battery voltage for scaling purposes.
     */
    public static double NOMINAL_BATTERY_VOLTAGE = 11.0; 
    
    /**
     * A wrapper class to house all CAN IDs.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class CAN_IDs 
    {
        /**
         * The ID of the Talon on the left CIM.
         */
        public static int LEFT_CIM_TALON = 5;
        
        /**
         * The ID of the Talon on the right CIM.
         */
        public static int RIGHT_CIM_TALON = 2;
        
        /**
         * The ID of the Victor on the left CIM.
         */
        public static int LEFT_CIM_VICTOR = 4;
        
        /**
         * The ID of the Victor on the right CIM.
         */
        public static int RIGHT_CIM_VICTOR = 3;
        
        /**
         * The ID of the elevator Talon on the bottom right 775 Pro.
         */
        public static int ELEVATOR_TALON = 9; 
        
        /**
         * The ID of the elevator Victor on the top left 775 Pro.
         */
        public static int ELEVATOR_VICTOR_TOPLEFT = 7;
        
        /**
         * The ID of the elevator Victor on the bottom left 775 Pro.
         */
        public static int ELEVATOR_VICTOR_BOTTOMLEFT = 8;
        
        /**
         * The ID of the elevator Victor on the top right 775 Pro.
         */
        public static int ELEVATOR_VICTOR_TOPRIGHT = 10;
        
        /**
         * The ID of the intake Talon on the left 775 Pro.
         */
        public static int INTAKE_TALON_LEFT = 6;
        
        /**
         * The ID of the intake Talon on the right 775 Pro.
         */
        public static int INTAKE_TALON_RIGHT = 1;
        
        /**
         * The ID of the Pigeon IMU.
         */
        public static int PIGEON = 1;
    }
    
    /**
     * A wrapper class to house all of the drivetrain-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class DrivetrainConstants
    {
        /**
         * The velocity amount to which all motion profile trajectories will be added to more easily
         * break static friction at the beginning of the profile.
         */
        public static final double MOT_PROF_ADD_TO_VEL_INIT = 7;
        
        /**
         * The time required to overcome static friction to allow for easier ramping.
         */
        public static final double TIME_TO_OVERCOME_S_FRICTION_MS = 45;
        
        /**
         * The Talon deadband (the output at which the Talon assumes zero output).
         */
        public static final double TALON_DEADBAND = 0.01;
        
        /**
         * Whether the invert the left Talon.
         */
        public static final boolean LEFT_TALON_INVERT = true;
        
        /**
         * Whether to invert the left Victor.
         */
        public static final boolean LEFT_VICTOR_INVERT = true;
        
        /**
         * The PID slot for velocity PID constants.
         */
        public static int VEL_PID = 1;
        
        /**
         * The PID slot for position PID constants.
         */
        public static int POS_PID = 2;
        
        /**
         * The PID slot for angle PID constants.
         */
        public static int ANGLE_PID = 0;
        
        /**
         * The PID slot for motion profile PID constants.
         */
        public static int MOTION_PROFILE_PID = 3;
        
        /**
         * The time for which the drivetrain should ramp up when beginning.
         */
        public static double MAX_RAMP_TIME = 0;
        
        /**
         * The nominal output (or the constant output percemt such that static friction is more easily broken)
         * for the left.
         */
        public static double NOMINAL_OUTPUT_LEFT = 0.04; //0.084;
        
        /**
         * The nominal output (or the constant output percent such that static friction is more easily broken)
         * for the right.
         */
        public static double NOMINAL_OUTPUT_RIGHT = 0.04; //0.084;
        
        /**
         * The peak output (or the maximum possible output percent) for the left.
         */
        public static double PEAK_OUTPUT_LEFT = 1;
        
        /**
         * The peak output (or the maximum possible output percent) for the right.
         */
        public static double PEAK_OUTPUT_RIGHT = 1;
        
        /**
         * The sensor phase for the left Talon when driving straight.
         */
        public static boolean LEFT_TALON_PHASE = false;
        
        /**
         * The sensor phase for the right Talon when driving straight.
         */
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
        
        
       
        /**
         * The practice bot F constant for the velocity closed loop on the left.
         */
        public static double VEL_KF_LEFT_PRACTICE = 0.197;
        
        /**
         * The practice bot P constant for the velocity closed loop on the left.
         */
        public static double VEL_KP_LEFT_PRACTICE = 0.5;
        
        /**
         * The practice bot I constant for the velocity closed loop on the left.
         */
        public static double VEL_KI_LEFT_PRACTICE = 0;
        
        /**
         * The practice bot D constant for the velocity closed loop on the left.
         */
        public static double VEL_KD_LEFT_PRACTICE = 0; 
        
        /**
         * The practice bot F constant for the velocity closed loop on the right.
         */
        public static double VEL_KF_RIGHT_PRACTICE = 0.197; 
        
        /**
         * The practice bot P constant for the velocity closed loop on the right.
         */
        public static double VEL_KP_RIGHT_PRACTICE = 0.5;//0.3;
        
        /**
         * The practice bot I constant for the velocity closed loop on the right.
         */
        public static double VEL_KI_RIGHT_PRACTICE = 0;//0;
        
        /**
         * The practice bot D constant for the velocity closed loop on the right.
         */
        public static double VEL_KD_RIGHT_PRACTICE = 0;//10;
        
        /**
         * The competition bot F constant for the velocity closed loop on the left.
         */
        public static double VEL_KF_LEFT_COMP = 0.197;
        
        /**
         * The competition bot P constant for the velocity closed loop on the left.
         */
        public static double VEL_KP_LEFT_COMP = 0.5;
        
        /**
         * The competition bot I constant for the velocity closed loop on the left.
         */
        public static double VEL_KI_LEFT_COMP = 0;
        
        /**
         * The competition bot D constant for the velocity closed loop on the left.
         */
        public static double VEL_KD_LEFT_COMP = 0; 
        
        /**
         * The competition bot F constant for the velocity closed loop on the right.
         */
        public static double VEL_KF_RIGHT_COMP = 0.197; 
        
        /**
         * The competition bot P constant for the velocity closed loop on the right.
         */
        public static double VEL_KP_RIGHT_COMP = 0.5;//0.3;
        
        /**
         * The competition bot I constant for the velocity closed loop on the right.
         */
        public static double VEL_KI_RIGHT_COMP = 0;//0;
        
        /**
         * The competition bot D constant for the velocity closed loop on the right.
         */
        public static double VEL_KD_RIGHT_COMP = 0;//10;
        
        /**
         * The F constant for the velocity closed loop on the left.
         */
        public static double VEL_KF_LEFT = IS_COMP ? VEL_KF_LEFT_COMP : VEL_KF_LEFT_PRACTICE;
        
        /**
         * The P constant for the velocity closed loop on the left.
         */
        public static double VEL_KP_LEFT = IS_COMP ? VEL_KP_LEFT_COMP : VEL_KP_LEFT_PRACTICE;
        
        /**
         * The I constant for the velocity closed loop on the left.
         */
        public static double VEL_KI_LEFT = IS_COMP ? VEL_KI_LEFT_COMP : VEL_KI_LEFT_PRACTICE;
        
        /**
         * The D constant for the velocity closed loop on the left.
         */
        public static double VEL_KD_LEFT = IS_COMP ? VEL_KD_LEFT_COMP : VEL_KD_LEFT_PRACTICE;
        
        /**
         * The F constant for the velocity closed loop on the right.
         */
        public static double VEL_KF_RIGHT = IS_COMP ? VEL_KF_RIGHT_COMP : VEL_KF_RIGHT_PRACTICE;
        
        /**
         * The P constant for the velocity closed loop on the right.
         */
        public static double VEL_KP_RIGHT = IS_COMP ? VEL_KP_RIGHT_COMP : VEL_KP_RIGHT_PRACTICE;
        
        /**
         * The I constant for the velocity closed loop on the right.
         */
        public static double VEL_KI_RIGHT = IS_COMP ? VEL_KI_RIGHT_COMP : VEL_KI_RIGHT_PRACTICE;
        
        /**
         * The D constant for the velocity closed loop on the right.
         */
        public static double VEL_KD_RIGHT = IS_COMP ? VEL_KD_RIGHT_COMP : VEL_KD_RIGHT_PRACTICE;

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
        
        /**
         * The F constant for the position closed loop on the left.
         */
        public static double POS_KF_LEFT = 0;
        
        /**
         * The P constant for the position closed loop on the left.
         */
        public static double POS_KP_LEFT = 0.19;//0.22; //0.2 
        
        /**
         * The I constant for the position closed loop on the left.
         */
        public static double POS_KI_LEFT = 0.00001;//0.00001;//0.001;//.0008;//0.00001;//0.00001; 
        
        /**
         * The D constant for the position closed loop on the left.
         */
        public static double POS_KD_LEFT = 16;//40;//40;
        
        /**
         * The integral zone constant for the position closed loop on the left.
         */
        public static int POS_IZONE_LEFT = 250;
        
        /**
         * The F constant for the position closed loop on the right.
         */
        public static double POS_KF_RIGHT = 0;
        
        /**
         * The P constant for the position closed loop on the right.
         */
        public static double POS_KP_RIGHT = 0.19;
        
        /**
         * The I constant for the position closed loop on the right.
         */
        public static double POS_KI_RIGHT = 0.00001;//0.00001;//0.001;//0.00001;//0.00001;
        
        /**
         * The D constant for the position closed loop on the right.
         */
        public static double POS_KD_RIGHT = 16;//40;
        
        /**
         * The integral zone constant for the position closed loop on the right.
         */
        public static int POS_IZONE_RIGHT = 250;
       
        /**
         * The allowable error for the position closed loop.
         */
        public static int POS_ALLOWABLE_ERROR = 200;

        /**
         * The wheel diameter on this drivetrain.
         */
        public static double WHEELDIAMETER = 4.0;
        
        /**
         * The maximum allowed drive speed for the drivetrain.
         */
        public static double MAX_DRIVE_SPEED_FPS = 14.0;
        
        /**
         * The maximum allowed turn speed for the drivetrain.
         */
        public static double MAX_TURN_SPEED_FPS = 14;
        
        /**
         * The peak current limit for the drivetrain.
         */
        public static int PEAK_CURRENT_LIMIT = 60;
        
        /**
         * The time for which the peak currrent limit is allowed (in ms).
         */
        public static int PEAK_TIME_MS = 200;
        
        /**
         * The continuous current limit for the drivetrain.
         */
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
        
        /**
         * The F constant for the motion profile closed loop on the left.
         */
        public static double MOTION_PROF_KF_LEFT = 0;
        
        /**
         * The P constant for the motion profile closed loop on the left.
         */
        public static double MOTION_PROF_KP_LEFT = 0;
        
        /**
         * The I constant for the motion profile closed loop on the left.
         */
        public static double MOTION_PROF_KI_LEFT = 0;
        
        /**
         * The D constant for the motion profile closed loop on the left.
         */
        public static double MOTION_PROF_KD_LEFT = 0;
        
        /**
         * The F constant for the motion profile closed loop on the right.
         */
        public static double MOTION_PROF_KF_RIGHT = 0.235;
        
        /**
         * The P constant for the motion profile closed loop on the right.
         */
        public static double MOTION_PROF_KP_RIGHT = 0.8;
        
        /**
         * The I constant for the motion profile closed loop on the right.
         */
        public static double MOTION_PROF_KI_RIGHT = 0.0001;
        
        /**
         * The D constant for the motion profile closed loop on the right.
         */
        public static double MOTION_PROF_KD_RIGHT = 4;

        /**
         * The allowable error for a motion profile closed loop.
         */
        public static final int MOTION_PROFILE_ALLOWABLE_ERROR = 500;
    }
    
    /**
     * A wrapper class to house all of the elevator-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class ElevatorConstants
    {
        /**
         * The height to which the elevator should raise for a switch autonomous (in encoder units).
         */
        public static final double SWITCH_HEIGHT_AUTON = 8000;
        
        /**
         * The height to which the elevator should raise for the third cube in a
         *  switch autonomous (in encoder units).
         */
        public static final double SWITCH_HEIGHT_THIRD_CUBE = 5000;
        
        /**
         * The continuous current limit when in manual control.
         */
        public static int MANUAL_CURRENT_LIMIT_CONT = 10;
        
        /**
         * The peak current limit for any control.
         */
        public static int PEAK_CURRENT_LIMIT = 10; // 25
        
        /**
         * The time (in ms) at which the peak current limit is valid.
         */
        public static int PEAK_TIME_MS = 750;
        
        /**
         * The continuous current limit for non-manual control.
         */
        public static int CONTINOUS_CURRENT_LIMIT = 5; //15
        
        /**
         * The PID slot to house motion magic constants.
         */
        public static int MOTION_MAGIC_PID = 0;
        
        /**
         * The PID slot to house position constants.
         */
        public static int POS_PID = 1;
        
        /**
         * The elevator nominal output.
         */
        public static double NOMINAL_OUTPUT = 0;
        
        /**
         * The feed forward to constantly add to the elevator to resist the effect
         * of gravity.
         */
        public static double POS_FGRAV = 0.06;
        
        /**
         * The peak output for the elevator motor controllers.
         */
        public static double PEAK_OUTPUT = 1.0;
        
        /**
         * The F constant for the position closed loop.
         */
        public static double POS_KF = 0;
        
        /**
         * The P constant for the position closed loop.
         */
        public static double POS_KP = 0.1;
        
        /**
         * The I constant for the position closed loop.
         */
        public static double POS_KI = 0.0001;
        
        /**
         * The D constant for the position closed loop.
         */
        public static double POS_KD = 18;
        
        /**
         * The allowable error for the position closed loop.
         */
        public static int POS_ALLOWABLE_ERROR = 1000;
        
        /**
         * The soft limit in the forward (upward) direction.
         */
        public static int FORWARD_SOFT = 34500;
        
        /**
         * The soft limit in the reverse (downward) direction.
         */
        public static int REVERSE_SOFT = 2000;
        
        /**
         * The time (in seconds) for which the elevator should ramp up to full speed in 
         * manual control.
         */
        public static double RAMP_RATE = 0.75;
        
        // elevator max RPM: 500 RPM
        /**
         * The F constant for the motion magic closed loop.
         */
        public static double MOTION_MAGIC_KF = 0.37;
        
        /**
         * The P constant for the motion magic closed loop.
         */
        public static double MOTION_MAGIC_KP = 0.1;
        
        /**
         * The I constant for the motion magic closed loop.
         */
        public static double MOTION_MAGIC_KI = 0.0001;
        
        /**
         * The D constant for the motion magic closed loop.
         */
        public static double MOTION_MAGIC_KD = 14;
        
        /**
         * The acceleration constant for the motion magic closed loop.
         */
        public static int MOTION_MAGIC_ACCEL = 9000/5;
        
        /**
         * The velocity constant for the motion magic closed loop.
         */
        public static int MOTION_MAGIC_VEL = 2600/5;
        
        /**
         * The allowable error for the motion magic closed loop.
         */
        public static int MOTION_MAGIC_ALLOWABLE_ERROR = 1000;
        
        /**
         * The height (in encoder units) for optimal intaking.
         */
        public static int INTAKE_HEIGHT = 2000;
        
        /**
         * The height (in encoder units) for optimal switch scoring.
         */
        public static int SWITCH_HEIGHT = 10000;
        
        /**
         * The height (in encoder units) for optimal low scale scoring.
         */
        public static int SCALE_LOW_HEIGHT = 20000;
        
        /**
         * The height (in encoder units) for optimal high scale scoring.
         */
        public static int SCALE_HIGH_HEIGHT = 34000;
        
        /**
         * The position at which the elevator should begin to slow.
         */
        public static double SLOW_DOWN_POS = 1000;
        
        /**
         * Whether to invert the bottom left victor (those not shown can be assumed false).
         */
        public static boolean BOTTOM_LEFT_VICTOR_INVERT = true;
        
        /**
         * The sensor phase for the elevator Talon.
         */
        public static boolean TALON_PHASE = true;
        
    }
    
    /**
     * A wrapper class to house all of the intake-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class IntakeConstants
    {
        /**
         * The CAN ID of the compressor.
         */
        public static final int COMPRESSOR_PORT = 0;
        
        /**
         * The CAN ID of the PCM.
         */
        public static final int FIRST_PCM_ID = 0;
        
        /**
         * The port of the solenoid to lower the intake.
         */
        public static final int INTAKE_DOWN_SOL = 1;
        
        /**
         * The solenoid value for lowering the intake.
         */
        public static final DoubleSolenoid.Value DOWN = DoubleSolenoid.Value.kReverse;
        
        /**
         * The port of the solenoid to raise the intake.
         */
        public static final int INTAKE_UP_SOL = 3;
        
        /**
         * The solenoid value for raising the intake.
         */
        public static final DoubleSolenoid.Value UP = DoubleSolenoid.Value.kForward;
        
        /**
         * The port of the solenoid to compress the intake.
         */
        public static final int INTAKE_COMPRESS_SOL = 0;
        
        /**
         * The solenoid value for compressing the intake.
         */
        public static final DoubleSolenoid.Value COMPRESS = DoubleSolenoid.Value.kReverse;
        
        /**
         * The port of the solenoid to decompress the intake.
         */
        public static final int INTAKE_DECOMPRESS_SOL = 2;
        
        /**
         * The solenoid value for decompressing the intake.
         */
        public static final DoubleSolenoid.Value DECOMPRESS = DoubleSolenoid.Value.kForward;
        
        /**
         * The key in the solenoid map of the up/down double solenoid.
         */
        public static final String UPDOWN_KEY = "UPDOWN";
        
        /**
         * The key in the solenoid map of the compress/decompress double solenoid.
         */
        public static final String COMPRESSDECOMPRESS_KEY = "COMPRESSDE";

        /**
         * The direction of Talon input such that the intake will intake.
         */
        public static final double INTAKE_DIR = 1;
        
        /**
         * The peak current limit for the intake.
         */
        public static int PEAK_CURRENT_LIMIT = 15;
        
        /**
         * The time (in ms) for which the peak current limit remains in use.
         */
        public static int PEAK_TIME_MS = 2000;
        
        /**
         * The continuous current limit for the intake.
         */
        public static int CONTINUOUS_CURRENT_LIMIT = 15;
        
        /**
         * The constant to signify that the intake will be controlled without manual control.
         */
        public static boolean NO_MANUAL_INTAKE = false;
    }
    
    /**
     * A wrapper class to house all of the Pigeon-related constants.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public static class PigeonConstants
    {
        /**
         * The F constant for pigeon motion profiling.
         */
        public static double MOT_PROF_KF = 0;
        
        /**
         * The P constant for pigeon motion profiling.
         */
        public static double MOT_PROF_KP = 1.1;
        
        /**
         * The I constant for pigeon motion profiling.
         */
        public static double MOT_PROF_KI = 0;
        
        /**
         * The D constant for pigeon motion profiling.
         */
        public static double MOT_PROF_KD = 7;
        
        /**
         * The F constant for pigeon turning in place.
         */
        public static double TURN_KF = 0.5;
        
        /**
         * The P constant for pigeon turning in place.
         */
        public static double TURN_KP = 3.6; 
        
        /**
         * The I constant for pigeon turning in place.
         */
        public static double TURN_KI = 0.003;
        
        /**
         * The D constant for pigeon turning in place.
         */
        public static double TURN_KD = 1; 
        
        /**
         * The integral zone constant for pigeon turning in place.
         */
        public static int TURN_IZONE = 150; //150;
        
        /**
         * The velocity constant for pigeon turning in place (w/ motion magic).
         */
        public static int TURN_VEL = 500;
        
        /**
         * The acceleration constant for pigeon turning in place (w/ motion magic).
         */
        public static int TURN_ACCEL = 600;
        
        /**
         * The frame period (in ms) for pigeon status updates.
         */
        public static final int PERIOD_MS = 4;

        /**
         * The Talon slot to configure the Pigeon as a remote sensor.
         */
        public static final RemoteFeedbackDevice REMOTE_SENSOR_SLOT = RemoteFeedbackDevice.RemoteSensor0;
        
        /**
         * The allowable error for an angle closed loop.
         */
        public static final int ANGLE_ALLOWABLE_ERROR = 50;
        
        /**
         * The left sensor phase for a turn in place.
         */
        public static final boolean LEFT_SENSOR_PHASE = false;
        
        /**
         * The right sensor phase for a turn in place.
         */
        public static final boolean RIGHT_SENSOR_PHASE = false;
    }
    
    /**
     * Stores the file paths for every trajectory to be loaded in.
     * 
     * @author Finn Frankis
     * @version Jul 13, 2018
     */
    public static class AutonomousConstants
    {
        /**
         * The left file name for the first autonomous path.
         */
        public static final String CLH_P1_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/1f/part1 _left_detailed.csv";
        
        /**
         * The right file name for the first autonomous path.
         */
        public static final String CLH_P1_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/1f/part1 _right_detailed.csv";
        
        
        /**
         * The left file name for the second autonomous path.
         */
        public static final String CLH_P2_LEFT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/2b/part12 _left_detailed.csv";
       
        /**
         * The right file name for the second autonomous path.
         */
        public static final String CLH_P2_RIGHT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/2b/part12 _right_detailed.csv";
        
        
        /**
         * The left file name for the third autonomous path.
         */
        public static final String CLH_P3_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/3f/part3_left_detailed.csv";
       
        /**
         * The right file name for the third autonomous path.
         */
        public static final String CLH_P3_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/3f/part3_right_detailed.csv";

        /**
         * The left file name for the fourth autonomous path.
         */
        public static final String CLH_P4_LEFT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/4b/part4_left_detailed.csv";
        
        /**
         * The right file name for the fourth autonomous path.
         */
        public static final String CLH_P4_RIGHT_REV = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/4b/part4_right_detailed.csv";
        
        /**
         * The left file name for the fifth autonomous path.
         */
        public static final String CLH_P5_LEFT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/5f/path15 _left_detailed.csv";
        
        /**
         * The right file name for the fifth autonomous path.
         */
        public static final String CLH_P5_RIGHT = "/home/summer2018/paths/2_cube/center_left_headon(switch_2)/5f/path15 _right_detailed.csv";     
    
        public static final double BASELINE_DISTANCE = 15;
    }

    /**
     * The constant to signify the first remote slot on a Talon.
     */
    public static int REMOTE_SLOT_0 = 0;
    
    /**
     * The constant to signify the second remote slot on a Talon.
     */
    public static int REMOTE_SLOT_1 = 1;

    /**
     * The constant to signify the primary PID loop index on a Talon.
     */
    public static final int PRIMARY_PID_INDEX = 0;
    
    /**
     * The constant to signify the auxiliary PID loop index on a Talon.
     */
    public static final int AUXILIARY_PID_INDEX = 1;

    /**
     * The general time for each trajectory point on a path.
     */
    public static final int TIME_PER_TRAJECTORY_POINT_MS = 10;

    /**
     * The total number of PID slots on a Talon.
     */
    public static final int NUM_PID_SLOTS = 4;
    
    /**
     * The maximum (and default) frame period for a Talon status frame.
     */
    public static final int MAX_TALON_FRAME_PERIOD_MS = 160;
}
