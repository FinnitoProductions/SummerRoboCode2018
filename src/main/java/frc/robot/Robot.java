/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.*;
import harkerrobolib.auto.AutoMode;
import harkerrobolib.auto.ParallelCommandGroup;
import harkerrobolib.auto.Path;
import harkerrobolib.auto.SequentialCommandGroup;
import harkerrobolib.auto.AutoMode.Location;
import harkerrobolib.util.Conversions;
import jaci.pathfinder.Trajectory.Segment;

import frc.robot.auto.modes.CompatibleScale;
import frc.robot.auto.paths.LeftToLeftScaleSide;
import frc.robot.commands.auton.AutonomousCommand;
import frc.robot.commands.auton.FollowPathRio;
import frc.robot.commands.drivetrain.DriveWithVelocityTimed;
import frc.robot.commands.drivetrain.TurnToAngleTimed;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.elevator.MoveElevatorVelocityTimed;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.intake.IntakeOuttakeTimed;
import frc.robot.commands.intake.SetSolenoid;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;

import com.ctre.phoenix.motorcontrol.NeutralMode; 

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents the central code for the robot.
 * @author Finn Frankis 
 * @version 6/11/18
 */
public class Robot extends TimedRobot
{
    public static Location location = Location.LEFT;

    /**
     * The drivetrain on the robot.
     */
    public static Drivetrain dt;
    
    /**
     * The intake on the robot.
     */
    public static Intake intake;
    
    /**
     * The elevator on the robot.
     */
    public static Elevator el;

    /**
     * The time at which the program began.
     */
    public static double startTime;
    
    /**
     * The class to hold all constants and methods to deal with input and output.
     */
    public static OI oi;

    /**
     * The command to be utilized during autonomous.
     */
    public static AutonomousCommand m_autonomousCommand;
    
    /**
     * The autonomous chooser.
     */
    private SendableChooser<Location> loc_chooser;

    private SendableChooser<AutoMode> LLL;  
    
    private String gameData = "";
    
    private Subsystem[] subsystems;

    public static Path baseline;
    

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public void robotInit() 
    {
        Conversions.setWheelDiameter(Drivetrain.WHEELDIAMETER);

        intake = Intake.getInstance();
        dt = Drivetrain.getInstance();
        el = Elevator.getInstance();
        oi = OI.getInstance();

        FollowPathRio.setDefaultLeftTalon(Robot.dt.getLeftMaster());
        FollowPathRio.setDefaultRightTalon(Robot.dt.getRightMaster());
        
        subsystems = new Subsystem[] {dt, el, intake, Intake.pn};
    
        loc_chooser = new SendableChooser<Location>();
        LLL = new SendableChooser<AutoMode>();

        loc_chooser.addDefault("Left", Location.LEFT);
        loc_chooser.addObject ("Center", Location.CENTER);
        loc_chooser.addObject ("Right", Location.RIGHT);

        SmartDashboard.putData("Robot Location", loc_chooser);
        
        dt.talonInitTeleop();
        el.talonInit();
        intake.talonInit();
        //System.out.println(new LeftToLeftScaleSide().getLeftPath().segments);
    }
    

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    public void disabledInit() 
    { 
        NeutralMode nm;
        if (Robot.dt.getLeftMaster().getMotorOutputPercent() > Drivetrain.NOMINAL_OUTPUT_LEFT * 1.5
                && Robot.dt.getRightMaster().getMotorOutputPercent() > Drivetrain.NOMINAL_OUTPUT_RIGHT * 1.5)
        {
            nm = NeutralMode.Brake;
        }
        else
            nm = NeutralMode.Coast;
        
        Robot.dt.getLeftMaster().setNeutralMode(nm);
        Robot.dt.getRightMaster().setNeutralMode(nm);
        Robot.dt.getLeftFollower().setNeutralMode(nm);
        Robot.dt.getRightFollower().setNeutralMode(nm);
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
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        //(m_autonomousCommand = new AutonomousCommand (location, subsystems, DriverStation.getInstance().getGameSpecificMessage())).start();
        location = loc_chooser.getSelected();
        System.out.println(location);

        //new DriveWithVelocityTimed(1, 0.85).start();
        //new TurnToAngleTimed(0.22, Drivetrain.TurnDirection.RIGHT)


        //new SequentialCommandGroup( new TurnToAngleTimed(0.22, Drivetrain.TurnDirection.LEFT)).start();

//        CommandGroup leftCommand = new SequentialCommandGroup(new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT_AUTON),
//                new TurnToAngleTimed(0.22, Drivetrain.TurnDirection.RIGHT),
//                new ParallelCommandGroup(new SetSolenoid(Pneumatics.SolenoidDirection.DECOMPRESS),
//                        new IntakeOuttakeTimed(3.0, Intake.IntakeType.OUTTAKE, 0.5)));

        //rightCommand.start();
        /*intake.pn.getSolenoid(IntakeConstants.UPDOWN_KEY).set(IntakeConstants.UP);
        intake.pn.getSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY).set(IntakeConstants.COMPRESS);*/
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    {
        /*String newData = DriverStation.getInstance().getGameSpecificMessage();
        if (newData.length() == 3 && (gameData.length() != 3 || !newData.equals(gameData))) {
        	gameData = newData;
        	if (m_autonomousCommand != null)
        		m_autonomousCommand.cancel();
        	(m_autonomousCommand = new AutonomousCommand (location, subsystems, newData)).start();
        }*/

        Scheduler.getInstance().run();

        SmartDashboard.putNumber("Left Motor Output", Robot.dt.getLeftMaster().getMotorOutputPercent());
        SmartDashboard.putNumber("Right Motor Output", Robot.dt.getRightMaster().getMotorOutputPercent());

        SmartDashboard.putNumber("Left Error", Robot.dt.getLeftMaster().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("Right Error", Robot.dt.getRightMaster().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));

        Robot.dt.printSensorPositions(RobotMap.PRIMARY_PID_INDEX);
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

    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic() 
    {
          Scheduler.getInstance().run();
          SmartDashboard.putNumber("Elevator Height", Robot.el.getBottomRightTalon().getSelectedSensorPosition(0));
          SmartDashboard.putNumber("Elevator Speed", Robot.el.getBottomRightTalon().getSelectedSensorVelocity(0));
          System.out.println(Elevator.getInstance().getCurrentCommand());
          //SmartDashboard.putNumber("Elevator Height", Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition(0));
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.updateValues();
        SmartDashboard.putData("Robot Location", loc_chooser);
        SmartDashboard.putNumber("el encoder", Robot.el.getBottomRightTalon().getSelectedSensorPosition());
    }
    
    /**
     * Gets the current time elapsed since the program started.
     * @return the total time elapsed
     */
    public static double getCurrentTimeMs()
    {
        return 1000 * (Timer.getFPGATimestamp() - startTime);
    }
}
