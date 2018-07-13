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

import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
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
import org.usfirst.frc.team1072.util.Speed;
import org.usfirst.frc.team1072.util.Speed.SpeedUnit;

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
import edu.wpi.first.wpilibj.Timer;
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

    public static double startTime;
    
    public static OI oi;

    public static AutonomousCommand m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public void robotInit()
    {
        intake = Intake.getInstance();
        dt = Drivetrain.getInstance();
        el = Elevator.getInstance();
        oi = OI.getInstance();
    }
    

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    public void disabledInit() 
    { 
        Robot.dt.getLeftTalon().setNeutralMode(NeutralMode.Coast);
        Robot.dt.getRightTalon().setNeutralMode(NeutralMode.Coast);
        Robot.dt.getLeftVictor().setNeutralMode(NeutralMode.Coast);
        Robot.dt.getRightVictor().setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Called periodically while the robot is disabled.
     */
    public void disabledPeriodic() { Scheduler.getInstance().run(); }

    /**
     * Initializes the autonomous period of the robot.
     */
    public void autonomousInit()
    {
        startTime = Timer.getFPGATimestamp();
        System.out.println("INITIALIZING AUTON " + Robot.getCurrentTimeMs());
        dt.talonInitAutonomous();
        System.out.println("DRIVE INITIALIZED " + Robot.getCurrentTimeMs());
        el.talonInit();
        System.out.println("ELEVATOR INITIALIZED " + Robot.getCurrentTimeMs());
        intake.talonInit();
        System.out.println("DRIVE/EL/INT AUTON INITIALIZED " + Robot.getCurrentTimeMs());
        m_autonomousCommand = new AutonomousCommand(new Subsystem[] {dt, el, intake, Intake.pn});
        System.out.println("AUTON COMMAND STARTED" + 1000 * (Timer.getFPGATimestamp() - startTime));
        Scheduler.getInstance().add(m_autonomousCommand);
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    { 
        Scheduler.getInstance().run();
        /*if (!m_autonomousCommand.isFinished())
            m_autonomousCommand.execute();*/
    }

    /**
     * Calls necessary methods to initialize for the teleoperated period.
     */
    public void teleopInit()
    {
        // stops autonomous command when teleop begins
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
        
        dt.talonInitTeleop();
        el.talonInit();
        intake.talonInit();
        dt.zeroPigeon();        
    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic() 
    {
          Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    public void testPeriodic()
    {
    }
    
    public static double getCurrentTimeMs()
    {
        return 1000 * (Timer.getFPGATimestamp() - startTime);
    }
}
