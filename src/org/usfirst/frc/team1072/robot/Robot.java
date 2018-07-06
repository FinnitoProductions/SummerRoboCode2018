/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.robot.commands.DriveWithVelocityCommand;

import java.io.File;
import java.io.IOException;

import org.usfirst.frc.team1072.robot.commands.AutonomousCommand;
//import org.harker.robotics.harkerrobolib.*;
//import org.harker.robotics.harkerrobolib.wrappers.GamepadWrapper;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
//import org.usfirst.frc.team1072.robot.commands.ExampleCommand;
import org.usfirst.frc.team1072.robot.commands.IntakeOuttakeCubeCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorMotionMagicCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorPositionCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorVelocityCommand;
import org.usfirst.frc.team1072.robot.commands.SetCompressorCommand;
import org.usfirst.frc.team1072.robot.commands.SetSolenoidCommand;
import org.usfirst.frc.team1072.robot.commands.ToggleCompressorCommand;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Gamepad;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Represents the central code for the robot.
 * @author Finn Frankis 
 * @version 6/11/18
 */
public class Robot extends TimedRobot
{
    public static Drivetrain dt;
    public static Intake intake;
    public static Elevator el;

    public static Joystick jt = new Joystick (OI.XBOX_360_PORT);

    
    public static OI oi;

    public static AutonomousCommand m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public void robotInit()
    {
        //m_chooser.addDefault("Default Auto", new ExampleCommand());
        // chooser.addObject("My Auto", new MyAutoCommand());j
        
        intake = Intake.getInstance();
        dt = Drivetrain.getInstance();
        el = Elevator.getInstance();
        oi = OI.getInstance();
    }
    
    


    /**
     * Converts an encoder value in units of ticks per 100 ms to a speed value in
     * fps (feet-per-second).
     * 
     * @param encoderVal the encoder value in ticks per 100 ms
     * @return the converted speed
     */
    public static double encoderUnitsToSpeed(int encoderVal)
    {
        // encoder value is stored in ticks per 100 ms
        return encoderVal * 10.0 // convert to ticks per second
                / RobotMap.TICKS_PER_REV // convert to revolutions per second
                * (RobotMap.WHEELDIAMETER * Math.PI) // convert to inches per second
                / 12.0; // convert to feet per second
    }

    /**
     * Converts a speed value in fps to encoder units of ticks per 100 ms
     * 
     * @param speed the speed in fps
     * @return the converted encoder value
     */
    public static double speedToEncoderUnits(double speed)
    {
        return speed * 12.0 / (RobotMap.WHEELDIAMETER * Math.PI) * RobotMap.TICKS_PER_REV / 10.0;
    }
    
    /**
     * Converts a given angle in radians (from Jaci's pathfinder) to pigeon units (where one rotation is
     * 8192 units)
     * @param angle the angle to convert, in radians
     * @return the original angle, but converted to radians
     */
    public static double radiansToPigeonUnits(double angle)
    {
        return angle / (2 * Math.PI) * RobotMap.PIGEON_UNITS_PER_ROTATION;
    }

    

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    public void disabledInit() { }

    /**
     * Called periodically while the robot is disabled.
     */
    public void disabledPeriodic() { Scheduler.getInstance().run(); }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    public void autonomousInit()
    {

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new
         * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
         * ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)

        dt.talonInitAutonomous();
        el.talonInit();
        intake.talonInit();
//        pigeonInit();
        (m_autonomousCommand = new AutonomousCommand(new Subsystem[] {dt, el, intake, Intake.pn})).start();
        
        
        
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    { 
        /*SmartDashboard.putNumber("Drivetrain Left Speed", dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
        SmartDashboard.putNumber("Drivetrain Right Speed", dt.getRightTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
        SmartDashboard.putNumber("Drivetrain Left Speed", dt.getLeftTalon().getSelectedSensorPosition(RobotMap.DT_VEL_PID));
        SmartDashboard.putNumber("Drivetrain Right Speed", dt.getRightTalon().getSelectedSensorPosition(RobotMap.DT_VEL_PID));
        
        Robot.dt.getLeftTalon().selectProfileSlot(RobotMap.POS_PID, RobotMap.PRIMARY_PID);
        Robot.dt.getRightTalon().selectProfileSlot(RobotMap.POS_PID, RobotMap.PRIMARY_PID);*/
        
        /*SmartDashboard.putNumber("LEFT POSITION ERROR", Robot.dt.getLeftTalon().getClosedLoopError(RobotMap.PRIMARY_PID));
        SmartDashboard.putNumber("RIGHT POSITION ERROR", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID));*/
        SmartDashboard.putNumber("RIGHT Position ERROR", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        /*SmartDashboard.putNumber("RIGHT SUM", Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("LEFT SUM", Robot.dt.getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("PIGEON THRU TALON", Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));*/
        Scheduler.getInstance().run();
    }

    /**
     * Calls necessary methods to initialize for the teleoperated period.
     */
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
        
        dt.talonInitTeleop();
        el.talonInit();
        intake.talonInit();
        dt.zeroPigeon();
        
        // setSelectedFeedback(pos, loopindex, timeout)
        // configSelectedFeedback(feedbackdevice, loopindex, timeout)
        // getSelectedFeedback
        //System.out.println("INITIALIZING TELEOP SETTING POSITION");
        //Robot.dt.getLeftTalon().getSelectedSensorPosition(4942197);
        //System.out.println("POSITION SET");
        //Intake.pn.getSolenoid(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY).set(RobotMap.INTAKE_COMPRESS); 
    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic() 
    {
          Scheduler.getInstance().run();
//        /*SmartDashboard.putNumber("Drivetrain Left Speed", dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));
//        SmartDashboard.putNumber("Drivetrain Right Speed", dt.getRightTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));*/
//        SmartDashboard.putNumber("Pigeon Value Through Right Talon", dt.getRightTalon().getSelectedSensorPosition(RobotMap.DT_ANGLE_PID));
//        SmartDashboard.putNumber("Pigeon Value Through Left Talon", dt.getLeftTalon().getSelectedSensorPosition(RobotMap.DT_ANGLE_PID));
//        SmartDashboard.putNumber("LEFT POSITION", dt.getLeftTalon().getSelectedSensorPosition(RobotMap.DT_POS_PID));
//        SmartDashboard.putNumber("LEFT OUTPUT VOLTAGE", dt.getLeftTalon().getMotorOutputVoltage());
//        SmartDashboard.putNumber("EIGHT OUTPUT VOLTAGE", dt.getRightTalon().getMotorOutputVoltage());
//        
//        //SmartDashboard.putNumber("RIGHT POSITION", dt.getLeftTalon().getSelectedSensorPosition(RobotMap.VEL_PID));*/
//        SmartDashboard.putNumber("PIGEON YAW", dt.getPigeonYaw());
        
        
        
    }

    /**
     * This function is called periodically during test mode.
     */
    public void testPeriodic()
    {
    }
}
