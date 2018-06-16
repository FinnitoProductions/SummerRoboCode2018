/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.robot.commands.DriveWithVelocityCommand;
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
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
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
    
    public static OI m_oi;

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public void robotInit()
    {
        m_oi = new OI();
        //m_chooser.addDefault("Default Auto", new ExampleCommand());
        // chooser.addObject("My Auto", new MyAutoCommand());
        dt = Drivetrain.getInstance();
        intake = Intake.getInstance();
        el = Elevator.getInstance();
        SmartDashboard.putData("Auto mode", m_chooser);
        System.out.println("robot initialized");
    }

    /**
     * Performs all commands to initialize the talons.
     * 
     * @postcondition the talons are in the correct state to begin robot operation
     */
    public void driveTrainTalonInit()
    {
        dtSlaveVictors();
        // initTalonOutput(0);

        dtInvertControllers();
        dtSetNeutralMode(NeutralMode.Brake);

        dtSetRampTime(RobotMap.MAX_RAMP_TIME);

        dtConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        dtScaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);

        dtSetSensorPhase();
        dtConfigureVelocityClosedLoop();
        dtConfigurePositionClosedLoop();

        dtSetCurrentLimit(RobotMap.DT_PEAK_CURRENT_LIMIT, RobotMap.DT_PEAK_TIME_MS,
                RobotMap.DT_CONTINUOUS_CURRENT_LIMIT);

    }

    /**
     * Initializes the talons and victors on the elevator.
     */
    public void elevatorTalonInit()
    {
        elSlaveVictors();
        elSetNeutralMode(NeutralMode.Brake);
        elSetCurrentLimit(RobotMap.EL_PEAK_CURRENT_LIMIT, RobotMap.EL_PEAK_TIME_MS,
                RobotMap.EL_CONTINOUS_CURRENT_LIMIT);
        elInvertControllers();
        elScaleVoltage(RobotMap.EL_NOMINAL_OUTPUT);
        elSetSoftLimit(RobotMap.EL_FORWARD_SOFT, RobotMap.EL_REVERSE_SOFT);

        elSetRampRate (RobotMap.EL_RAMP_RATE);
        elScaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        elConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        elZeroSensors();

        elConfigurePositionClosedLoop();
        elConfigureMotionMagic();

    }

    /**
     * Sets the ramp rate for closed loop elevator motion.
     * @param rampRate the time from zero to full voltage
     */
    private void elSetRampRate (double rampRate)
    {
        el.getBottomRightTalon().configOpenloopRamp(rampRate, RobotMap.TIMEOUT);
    }
    /**
     * Slaves the three elevator Victors to the one Talon.
     */
    private void elSlaveVictors()
    {
        TalonSRX talon = el.getBottomRightTalon();
        el.getBottomLeftVictor().follow(talon);
        el.getTopLeftVictor().follow(talon);
        el.getTopRightVictor().follow(talon);
    }

    /**
     * Inverts the motor controllers so that all align correctly.
     */
    private void elInvertControllers()
    {
        el.getBottomLeftVictor().setInverted(RobotMap.EL_BOTTOM_LEFT_VICTOR_INVERT);
        el.getBottomRightTalon().setInverted(RobotMap.EL_TALON_INVERT);
        el.getTopRightVictor().setInverted(RobotMap.EL_TOP_RIGHT_VICTOR_INVERT);
        el.getTopLeftVictor().setInverted(RobotMap.EL_TOP_LEFT_VICTOR_INVERT);
    }

    /**
     * Sets the correct neutral mode for the motor controllers on the elevator
     * 
     * @param n the neutral mode (coast or brake) to be set
     */
    private void elSetNeutralMode(NeutralMode n)
    {
        el.getBottomRightTalon().setNeutralMode(n);
        el.getTopRightVictor().setNeutralMode(n);
        el.getBottomLeftVictor().setNeutralMode(n);
        el.getTopLeftVictor().setNeutralMode(n);
    }

    /**
     * Scales the voltage on the elevator controllers given nominal voltage.
     * 
     * @param nomVoltage
     *            the nominal voltage of the battery
     */
    private void elScaleVoltage(double nomVoltage)
    {
        el.getBottomRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
    }

    /**
     * Sets the elevator soft limit given it as a parameter.
     * 
     * @param softLimit
     *            the soft limit to be set
     */
    private void elSetSoftLimit(int forwardSoftLimit, int reverseSoftLimit)
    {
        el.getBottomRightTalon().configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.TIMEOUT);
        el.getBottomRightTalon().configForwardSoftLimitEnable(true, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.TIMEOUT);
        el.getBottomRightTalon().configReverseSoftLimitEnable(true, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().overrideLimitSwitchesEnable(true);
    }

    /**
     * Configures the elevator position closed loop.
     * 
     * @postcondition the F/P/I/D constants have been set, as well as the nominal
     *                output in both directions
     */
    private void elConfigurePositionClosedLoop()
    {
        el.getBottomRightTalon().configNominalOutputForward(RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configNominalOutputReverse(-1 * RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configPeakOutputForward(RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configPeakOutputReverse(-1 * RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kF(RobotMap.EL_POS_PID, RobotMap.EL_POS_KF, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kP(RobotMap.EL_POS_PID, RobotMap.EL_POS_KP, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kI(RobotMap.EL_POS_PID, RobotMap.EL_POS_KI, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kD(RobotMap.EL_POS_PID, RobotMap.EL_POS_KD, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.EL_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
    }
    
    /**
     * Configures Motion Magic on the elevator, which seamlessly allows the robot to move between positions.
     */
    private void elConfigureMotionMagic()
    {
        // set motion magic port to be the velocity PID port 
        el.getBottomRightTalon().selectProfileSlot(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_PID);
        el.getBottomRightTalon().config_kF(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KF, RobotMap.TIMEOUT);
        el.getBottomRightTalon().config_kP(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KP, RobotMap.TIMEOUT);
        el.getBottomRightTalon().config_kI(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KI, RobotMap.TIMEOUT);
        el.getBottomRightTalon().config_kD(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KD, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configMotionCruiseVelocity(RobotMap.EL_VEL_VEL, RobotMap.TIMEOUT);
        el.getBottomRightTalon().configMotionAcceleration(RobotMap.EL_VEL_ACCEL, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configAllowableClosedloopError(RobotMap.VEL_PID, RobotMap.EL_VEL_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        
    }

    /**
     * Sets the current limit for the elevator in amps.
     * 
     * @param peakCurrentLimit
     *            the peak limit (only temporary)
     * @param peakTime
     *            the time (in ms) for which the peak limit is permitted
     * @param continuousLimit
     *            the limit after the peak time has expired.
     */
    private void elSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        el.getBottomRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        el.getBottomRightTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        el.getBottomRightTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        el.getBottomRightTalon().enableCurrentLimit(true);
    }

    /**
     * Configures the elevator sensor given the feedback device.
     * 
     * @param fd the feedback device, either quadrature or absolute
     */
    private void elConfigureSensors(FeedbackDevice fd)
    {
        el.getBottomRightTalon().configSelectedFeedbackSensor(fd, RobotMap.POS_PID, RobotMap.TIMEOUT);
    }

    /**
     * Zeros the elevator sensor, should always be called during initialization.
     */
    private void elZeroSensors()
    {
        el.getBottomRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
    }
    

    /**
     * Initializes the intake talons.
     */
    private void intakeTalonInit()
    {
        intakeSetNeutralMode(NeutralMode.Brake);
        intakeSetCurrentLimit(RobotMap.INT_PEAK_CURRENT_LIMIT, RobotMap.INT_PEAK_TIME_MS,
                RobotMap.INT_CONTINUOUS_CURRENT_LIMIT);
    }

    /**
     * Sets the neutral mode for the Talons
     * 
     * @param nm the neutral mode
     */
    private void intakeSetNeutralMode(NeutralMode nm)
    {
        intake.getLeftTalon().setNeutralMode(nm);
        intake.getRightTalon().setNeutralMode(nm);
    }

    /**
     * Sets the current limit on the intake.
     * 
     * @param peakCurrentLimit
     *            the peak limit (only temporary)
     * @param peakTime
     *            the time (in ms) for the peak limit to be allowed
     * @param continuousLimit
     *            the continuous current limit (after peak has expired)
     */
    private void intakeSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        intake.getLeftTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        intake.getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        intake.getLeftTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        intake.getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        intake.getLeftTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        intake.getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        intake.getLeftTalon().enableCurrentLimit(true);
        intake.getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
    }

    /**
     * Converts an encoder value in units of ticks per 100 ms to a speed value in
     * fps (feet-per-second).
     * 
     * @param encoderVal the encoder value in ticks per 100 ms
     * @return the converted speed
     */
    private double encoderUnitsToSpeed(int encoderVal)
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
    private double speedToEncoderUnits(double speed)
    {
        return speed * 12.0 / (RobotMap.WHEELDIAMETER * Math.PI) * RobotMap.TICKS_PER_REV / 10.0;
    }

    /**
     * Slaves the Victors to directly follow the behavior of their parent talons.
     */
    private void dtSlaveVictors()
    {
        dt.getLeftVictor().follow(dt.getLeftTalon());
        dt.getRightVictor().follow(dt.getRightTalon());
    }

    /**
     * Initializes the Talon output.
     * 
     * @param percentOutput
     *            the output to which the Talons should be initialized
     */
    private void dtInitTalonOutput(int percentOutput)
    {
        dt.getLeftTalon().set(ControlMode.Velocity, percentOutput);
        dt.getRightTalon().set(ControlMode.Velocity, percentOutput);
    }

    /**
     * Inverts the Talons and Victors to account for wiring inconsistencies (must be
     * tested).
     */
    private void dtInvertControllers()
    {
        dt.getLeftTalon().setInverted(true);
        dt.getLeftVictor().setInverted(true);
        
        // Invert the following direction (left Talons and Victors were wired
        // oppositely)
        dt.getRightTalon().setInverted(false);
        dt.getRightVictor().setInverted(false);
    }

    /**
     * Sets the neutral mode to either coast or brake.
     * 
     * @param n
     *            either coast or brake
     */
    private void dtSetNeutralMode(NeutralMode n)
    {
        dt.getLeftTalon().setNeutralMode(n);
        dt.getLeftVictor().setNeutralMode(n);
        dt.getRightTalon().setNeutralMode(n);
        dt.getRightVictor().setNeutralMode(n);
    }

    /**
     * Configure the talons to ramp gradually to peak voltage.
     * 
     * @param t
     *            the ramp time (time from 0 voltage to max)
     */
    private void dtSetRampTime(double t)
    {
        dt.getLeftTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
        dt.getRightTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
    }

    /**
     * Scales the voltage to modify percent output to account for the battery
     * voltage loss.
     * 
     * @param nomVoltage
     *            the assumed nominal voltage, used to scale relative to battery
     *            voltage
     */
    private void dtScaleVoltage(double nomVoltage)
    {
        dt.getLeftTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        dt.getRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
    }

    /**
     * Configures the encoders either to use pulse width or quadrature velocity.
     * 
     * @param f
     *            the feedback device to use to configure (either quadrature or
     *            sensor velocity)
     */
    private void dtConfigureSensors(FeedbackDevice f)
    {
        dt.getLeftTalon().configSelectedFeedbackSensor(f, RobotMap.VEL_PID, RobotMap.TIMEOUT);
        dt.getRightTalon().configSelectedFeedbackSensor(f, RobotMap.VEL_PID, RobotMap.TIMEOUT);
    }

    /**
     * Aligns the sensor phase of the encoders to match the motions of the motors.
     */
    private void dtSetSensorPhase()
    {
        dt.getLeftTalon().setSensorPhase(RobotMap.DT_LEFT_TALON_PHASE);
        dt.getRightTalon().setSensorPhase(RobotMap.DT_RIGHT_TALON_PHASE);
    }

    /**
     * Configures the drivetrain velocity closed loop.
     * 
     * @postcondition nominal and peak output in both directions has been set
     */
    private void dtConfigureVelocityClosedLoop()
    {
        dt.getLeftTalon().configNominalOutputForward(RobotMap.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().configNominalOutputForward(RobotMap.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().configNominalOutputReverse(-1 * RobotMap.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().configNominalOutputReverse(-1 * RobotMap.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().configPeakOutputForward(RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);
        dt.getRightTalon().configPeakOutputForward(RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);

        dt.getLeftTalon().configPeakOutputReverse(-1 * RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);
        dt.getRightTalon().configPeakOutputReverse(-1 * RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kF(RobotMap.VEL_PID, RobotMap.VEL_KF_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kF(RobotMap.VEL_PID, RobotMap.VEL_KF_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kP(RobotMap.VEL_PID, RobotMap.VEL_KP_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kP(RobotMap.VEL_PID, RobotMap.VEL_KP_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kI(RobotMap.VEL_PID, RobotMap.VEL_KI_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kI(RobotMap.VEL_PID, RobotMap.VEL_KI_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kD(RobotMap.VEL_PID, RobotMap.VEL_KD_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kD(RobotMap.VEL_PID, RobotMap.VEL_KD_RIGHT, RobotMap.TIMEOUT);
    }

    /**
     * Configures the drivetrain position closed loop.
     * 
     * @postcondition each talon has configured F, P, I, D and D constants, the
     *                encoders have been zeroed, and the allowable error has been
     *                set
     */
    private void dtConfigurePositionClosedLoop()
    {
        dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
        dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);

        dt.getLeftTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        dt.getRightTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_RIGHT, RobotMap.TIMEOUT);

        dt.getLeftTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_RIGHT, RobotMap.TIMEOUT);
    }

    /**
     * Sets a current limit for the left and right talons on the drivetrain.
     * 
     * @param peakCurrentLimit
     *            the peak current limit (only temporary)
     * @param peakTime
     *            the time for which the peak current limit is allowed
     * @param continuousLimit
     *            the continuous limit (represents the limit after peakTime passes)
     */
    private void dtSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        dt.getLeftTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        dt.getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        dt.getLeftTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        dt.getRightTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);

        dt.getLeftTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        dt.getRightTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);

        dt.getLeftTalon().enableCurrentLimit(true);
        dt.getRightTalon().enableCurrentLimit(true);
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
        m_autonomousCommand = m_chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new
         * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
         * ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null)
            m_autonomousCommand.start();
        driveTrainTalonInit();
        elevatorTalonInit();
        intakeTalonInit();
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic(){ /* Scheduler.getInstance().run();*/ }

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
            m_autonomousCommand.cancel();
        driveTrainTalonInit();
        elevatorTalonInit();
        intakeTalonInit();
    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic()
    {
         //drivePeriodic();
        
        // POSITION PID
        /*MoveElevatorPositionCommand mepc = new MoveElevatorPositionCommand();
        mepc.execute(Math.abs(jt.getY()) * 15100 + 9900);
        
        
        SmartDashboard.putNumber("Elevator Position",
        el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.EL_POS_PID));
        SmartDashboard.putNumber("Output Voltage",
        el.getBottomRightTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("Talon Error",
        el.getBottomRightTalon().getClosedLoopError(RobotMap.POS_PID));*/
          
        MoveElevatorMotionMagicCommand memmc = new MoveElevatorMotionMagicCommand();
        memmc.execute(Math.abs(jt.getRawAxis(OI.CHEAP_WHITE_RIGHTY)) * 33500 + 1000);
        
        SmartDashboard.putNumber("Elevator Velocity",
                el.getBottomRightTalon().getSelectedSensorVelocity(RobotMap.EL_VEL_PID));
        SmartDashboard.putNumber("Motor Voltage", 
                el.getBottomRightTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("Talon Current", el.getBottomRightTalon().getOutputCurrent());
        SmartDashboard.putNumber("Tal Error", el.getBottomRightTalon().getClosedLoopError(RobotMap.EL_VEL_PID));
        
        if (el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.EL_VEL_PID) < 1000)
        {
            el.getBottomRightTalon().config_kP(RobotMap.EL_VEL_PID, 0, RobotMap.TIMEOUT);
            el.getBottomRightTalon().config_kI(RobotMap.EL_VEL_PID, 0, RobotMap.TIMEOUT);
            el.getBottomRightTalon().config_kD(RobotMap.EL_VEL_PID, 0, RobotMap.TIMEOUT);
        }
        else
        {
            elConfigureMotionMagic();
        }
        
        drivePeriodic();
         
        /*
         * double targetPos = -1 * jt.getY() * RobotMap.TICKS_PER_REV * 3;
         * DriveToPositionCommand dtp = new DriveToPositionCommand();
         * dtp.execute(targetPos);
         * 
         * SmartDashboard.putNumber("Drivetrain Current (L)",
         * dt.getLeftTalon().getOutputCurrent());
         * SmartDashboard.putNumber("Drivetrain Current (R)",
         * dt.getRightTalon().getOutputCurrent());
         * //SmartDashboard.putNumber("Joystick X", jt.getX());
         * //SmartDashboard.putNumber("Joystick Y", jt.getY());
         * SmartDashboard.putNumber("Talon Error",
         * dt.getLeftTalon().getClosedLoopError(RobotMap.POS_PID));
         */

        

         

        /*
         * MoveElevatorVelocityCommand mevc = new MoveElevatorVelocityCommand();
         * mevc.execute(-0.3 * jt.getY()); SmartDashboard.putNumber("Talon Current",
         * el.getBottomRightTalon().getOutputCurrent());
         */

        /*
         * 
         */

       
        
        /*
         * /*SmartDashboard.putNumber("Position (R)",
         * encoderUnitsToSpeed(dt.getRightTalon().getSelectedSensorVelocity(RobotMap.
         * POS_PID)));
         */
        /*
         * SmartDashboard.putNumber("Position (L)",
         * dt.getLeftTalon().getSelectedSensorPosition(RobotMap.POS_PID));
         * SmartDashboard.putNumber("Velocity (L)",
         * dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));
         * SmartDashboard.putNumber("Target Position", targetPos);
         * 
         * SmartDashboard.putNumber("Motor Output Voltage",
         * dt.getLeftTalon().getMotorOutputVoltage());
         * SmartDashboard.putNumber("Motor Output Percent",
         * dt.getLeftTalon().getMotorOutputPercent());
         */
    }
    
    /**
     * Represents important commands to be called for a standard teleoperated drive period.
     */
    private void drivePeriodic()
    {
        DriveWithVelocityCommand adc = new DriveWithVelocityCommand();
        
        // all the way up for y stick is given as -1 
        double driveSpeed = speedToEncoderUnits(-1 * jt.getY() * RobotMap.MAX_DRIVE_SPEED); 
        double turnSpeed = speedToEncoderUnits(-1 * jt.getX() * RobotMap.MAX_TURN_SPEED);
        adc.execute(driveSpeed, turnSpeed);
        
        
        SetCompressorCommand tc = new SetCompressorCommand();
        
        tc.execute(true);
         
        SetSolenoidCommand ssc = new SetSolenoidCommand(); int i = jt.getPOV();
         
         if (i > -1 && i <= 1) 
             ssc.execute(RobotMap.INTAKE_UPDOWN_KEY,
                     RobotMap.INTAKE_UP); 
         if (i >= 269 && i <= 271)
         ssc.execute(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY,
                 RobotMap.INTAKE_DECOMPRESS); 
         if (i >= 179 && i <= 181)
             ssc.execute(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN); 
         if (i >= 89 && i <= 91) 
             ssc.execute(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY,
        RobotMap.INTAKE_COMPRESS);
        
         SmartDashboard.putNumber("D-Pad", Math.round(jt.getPOV()));
         
         
         IntakeOuttakeCubeCommand iocc = new IntakeOuttakeCubeCommand();
         if (jt.getRawButton(OI.LEFT_BUMPER))
             iocc.execute(1);
         else if (jt.getRawButton(OI.RIGHT_BUMPER))
             iocc.execute(-1);
         else
             iocc.execute(0);
         
         /*MoveElevatorVelocityCommand mevc = new MoveElevatorVelocityCommand();
         boolean isDown = OI.AXIS_MULTIPLIER * jt.getRawAxis(OI.CHEAP_WHITE_RIGHTY) < 0;
         boolean reverseBeyondLimit = el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.POS_PID) <= RobotMap.EL_REVERSE_SOFT;
         if (isDown && !reverseBeyondLimit)
           mevc.execute(OI.AXIS_MULTIPLIER * jt.getRawAxis(OI.CHEAP_WHITE_RIGHTY));
         else if (isDown && reverseBeyondLimit)
             mevc.execute(0);
         else
             mevc.execute(OI.AXIS_MULTIPLIER * jt.getRawAxis(OI.CHEAP_WHITE_RIGHTY));
         SmartDashboard.putNumber("Elevator Position", el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.POS_PID));*/
         
         
         SmartDashboard.putNumber("Axis 0", jt.getRawAxis(0));
         SmartDashboard.putNumber("Axis 1", jt.getRawAxis(1));
         SmartDashboard.putNumber("Axis 2", jt.getRawAxis(2));
         SmartDashboard.putNumber("Axis 3", jt.getRawAxis(3));
         SmartDashboard.putNumber("Axis 4", jt.getRawAxis(4));
         
         SmartDashboard.putBoolean("Button 5", jt.getRawButton(5));
         SmartDashboard.putBoolean("Button 6", jt.getRawButton(6));
         
        
        
    }

    /**
     * This function is called periodically during test mode.
     */
    public void testPeriodic()
    {
    }
}
