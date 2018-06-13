/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
    
    public static int LEFT_CIM_TALON = 5;
    public static int RIGHT_CIM_TALON = 2;
    public static int LEFT_CIM_VICTOR = 4;
    public static int RIGHT_CIM_VICTOR = 3;
    public static int JOYSTICK = 0;
    
    public static int TIMEOUT = 10;
    public static int VEL_PID = 0;
    
    public static double MAX_RAMP_TIME = 0.5;
    
    public static double NOMINAL_BATTERY_VOLTAGE = 11.0;
    
    public static double NOMINAL_OUTPUT_LEFT = 0;
    public static double NOMINAL_OUTPUT_RIGHT = 0; 
    public static double DRIVETRAIN_SCALE = 1;
    
    public static double VEL_KF_LEFT = .2;
    public static double VEL_KP_LEFT = .1;
    public static double VEL_KI_LEFT = 0;
    public static double VEL_KD_LEFT = 0;
    
    public static double VEL_KF_RIGHT = .2;
    public static double VEL_KP_RIGHT = 0;
    public static double VEL_KI_RIGHT = 0;
    public static double VEL_KD_RIGHT = 0;
    
    public static int TICKS_PER_REV = 4096;
    public static double WHEELDIAMETER = 4.0;
    
    public static double MAX_DRIVE_SPEED = 14.0;
    public static double MAX_TURN_SPEED = 8.0; 
    
 
}
