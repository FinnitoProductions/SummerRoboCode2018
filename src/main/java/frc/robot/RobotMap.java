/*----------------------------------------------------------------------------*/
 /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    
        public static final String LLS_P1_SWITCH_LEFT = "/home/summer2018/paths/1_cube/left_left_side(switch_1)/left_left_side(switch_1)_left_detailed.csv";

        public static final String LLS_P1_SWITCH_RIGHT = "/home/summer2018/paths/1_cube/left_left_side(switch_1)/left_left_side(switch_1)_right_detailed.csv";

        public static final String RRS_P1_SWITCH_LEFT = "/home/summer2018/paths/1_cube/right_right_side(switch_1)/right_right_side(switch_1)_left_detailed.csv";

        public static final String RRS_P1_SWITCH_RIGHT = "/home/summer2018/paths/1_cube/right_right_side(switch_1)/right_right_side(switch_1)_right_detailed.csv";

        public static final String LEFT_SCALE_SNEAKY_LEFT = "/home/summer2018/paths/Test/Lscale/Lscale_left_Jaci.csv";
        public static final String LEFT_SCALE_SNEAKY_RIGHT = "/home/summer2018/paths/Test/Lscale/Lscale_right_Jaci.csv";

        public static final int SCALE_DISTANCE_FEET = 27;

        public static final double BASELINE_DISTANCE = 13;

        public static final double SCALE_OUTTAKE_TIME = 15;
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
