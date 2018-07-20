/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.commands.auton.AutonomousCommand;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.util.Conversions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        Conversions.setWheelDiameter(DrivetrainConstants.WHEELDIAMETER);
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
        Robot.dt.getLeftTalon().setNeutralMode(NeutralMode.Brake);
        Robot.dt.getRightTalon().setNeutralMode(NeutralMode.Brake);
        Robot.dt.getLeftVictor().setNeutralMode(NeutralMode.Brake);
        Robot.dt.getRightVictor().setNeutralMode(NeutralMode.Brake);
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
        Robot.dt.talonInit();
        Robot.dt.configureAngleClosedLoop();
        System.out.println("START TIME: " + Robot.getCurrentTimeMs());
        System.out.println("START YAW: " + Robot.dt.getPigeonYaw());
        double oldYaw = Robot.dt.getPigeonYaw();
        Robot.dt.addPigeonYaw(1000);
        while (Robot.dt.getPigeonYaw() < (oldYaw + 1000 - 1) || Robot.dt.getPigeonYaw() > (oldYaw + 1000 + 1));
        System.out.println("YAW SET " + + Robot.dt.getPigeonYaw() + " " + Robot.getCurrentTimeMs());
        /*try
        {
            Thread.sleep(100l);
        }
        catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("PIGEON POSITION AFTER YAW: " + Robot.dt.getLeftTalon().
                getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));
        Robot.dt.getLeftTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, RobotMap.REMOTE_SLOT_0, RobotMap.TIMEOUT);
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, RobotMap.AUXILIARY_PID_INDEX,
                RobotMap.TIMEOUT);*/
        /*try
        {
            Thread.sleep(100l);
        }
        catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("PIGEON POSITION AFTER CONFIG: " + Robot.dt.getLeftTalon().
                getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));
       
        
        Robot.dt.getLeftTalon().setSelectedSensorPosition(12000, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        
        try
        {
            Thread.sleep(100l);
        }
        catch (InterruptedException e)
        {
            
        }
        System.out.println("PIGEON POSITION AFTER SET SELECTED SENSOR: " + Robot.dt.getLeftTalon().
                getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));

        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, RobotMap.AUXILIARY_PID_INDEX,
                RobotMap.TIMEOUT);
        try
        {
            Thread.sleep(100l);
        }
        catch (InterruptedException e)
        {
            
        }
        System.out.println("PIGEON POSITION AFTER CONFIG: " + Robot.dt.getLeftTalon().
                getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));*/
        
        
        
        /*startTime = Timer.getFPGATimestamp();
        m_autonomousCommand = new AutonomousCommand(new Subsystem[] {dt, el, intake, Intake.pn});

        m_autonomousCommand.start();
        System.out.println("AUTON COMMAND STARTED" + Robot.getCurrentTimeMs());*/
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    { 

        /*Scheduler.getInstance().run();
        Robot.dt.printMotorOutputPercentage();
        Robot.dt.printClosedLoopError(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.printClosedLoopError(RobotMap.AUXILIARY_PID_INDEX);
        Robot.dt.printSensorPositions(RobotMap.PRIMARY_PID_INDEX);*/
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
