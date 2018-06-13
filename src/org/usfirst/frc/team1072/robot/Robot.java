/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1072.robot.commands.ArcadeDriveCommand;
import org.usfirst.frc.team1072.robot.commands.ExampleCommand;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Drivetrain dt;
	public static Joystick jt = new Joystick(RobotMap.JOYSTICK);
	public static OI m_oi;
	

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
	    
		m_oi = new OI();
		m_chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		dt = Drivetrain.getInstance();
		SmartDashboard.putData("Auto mode", m_chooser);
		System.out.println("robot initialized");

	}

	/**
	 * Performs all commands to initialize the talons.
	 * 
	 * @postcondition the talons are in the correct state to begin robot operation
	 */
	public void talonInit()
	{
	    slaveVictors();
	    //initTalonOutput(0);
        
        invertControllers();
        setNeutralMode(NeutralMode.Coast);
        
        setRampTime(RobotMap.MAX_RAMP_TIME);
        
        configureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        scaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        
        configureVelocityClosedLoop();
        
	}
	
	private double encoderUnitsToSpeed(int encoderVal)
	{
	    // encoder value is stored in ticks per 100 ms
	    return encoderVal * 10.0 // convert to ticks per second
	            / RobotMap.TICKS_PER_REV // convert to revolutions per second
	            * (RobotMap.WHEELDIAMETER * Math.PI) // convert to inches per second
	            / 12.0; // convert to feet per second
	}
	
	private double speedToEncoderUnits(double speed)
	{
	    return speed 
	            * 12.0  
	            / (RobotMap.WHEELDIAMETER * Math.PI)
	            * RobotMap.TICKS_PER_REV
	            / 10.0;
	}
	
	/**
	 * Slaves the Victors to directly follow the behavior of their parent talons.
	 */
	private void slaveVictors()
	{
	    dt.getLeftVictor().follow(dt.getLeftTalon());
        dt.getRightVictor().follow(dt.getRightTalon());
	}
	
	/**
	 * Initializes the Talon output.
	 * @param percentOutput the output to which the Talons should be initialized
	 */
	private void initTalonOutput(int percentOutput)
	{
	    dt.getLeftTalon().set(ControlMode.Velocity, percentOutput);
        dt.getRightTalon().set(ControlMode.Velocity, percentOutput);
	}

	/**
	 * Inverts the Talons and Victors to account for wiring inconsistencies (must be tested).
	 */
	private void invertControllers()
	{
	    dt.getLeftTalon().setInverted(false);
        
        // Invert the following direction (left Talons and Victors were wired oppositely)
        dt.getRightTalon().setInverted(true);
        dt.getRightVictor().setInverted(true); 
	}
	
	/**
	 * Sets the neutral mode to either coast or brake.
	 * @param n either coast or brake
	 */
	private void setNeutralMode(NeutralMode n)
	{
	    dt.getLeftTalon().setNeutralMode(n);
        dt.getLeftVictor().setNeutralMode(n);
        dt.getRightTalon().setNeutralMode(n);
        dt.getRightVictor().setNeutralMode(n);
	}
	
	/**
	 * Configure the talons to ramp gradually to peak voltage.
	 * @param t the ramp time (time from 0 voltage to max)
	 */
	private void setRampTime(double t)
	{
	    dt.getLeftTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
        dt.getRightTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
	}
	
	/**
	 * Scales the voltage to modify percent output to account for the battery voltage loss.
	 * @param nomVoltage the assumed nominal voltage, used to scale relative to battery voltage
	 */
	private void scaleVoltage(double nomVoltage)
	{
	    dt.getLeftTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        dt.getRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);   
	}
	
	/**
	 * Configures the encoders either to use pulse width or quadrature velocity.
	 * @param f the feedback device to use to configure (either quadrature or sensor velocity)
	 */
	private void configureSensors(FeedbackDevice f)
	{
	    dt.getLeftTalon().configSelectedFeedbackSensor(f, RobotMap.VEL_PID, RobotMap.TIMEOUT);
        dt.getRightTalon().configSelectedFeedbackSensor(f, RobotMap.VEL_PID, RobotMap.TIMEOUT);
	}
	
	/**
	 * Configures the velocity closed loop.
	 */
	private void configureVelocityClosedLoop()
	{
	    dt.getLeftTalon().setSensorPhase(true);
	    dt.getRightTalon().setSensorPhase(true);
        
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
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
		talonInit();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//Scheduler.getInstance().run();
	    
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		talonInit();
	}

	/**
	 * This function is called periodically during operator control.
	*/
	public void teleopPeriodic() {
		ArcadeDriveCommand adc = new ArcadeDriveCommand();
		
		
		double driveSpeed = speedToEncoderUnits(jt.getY() * RobotMap.MAX_DRIVE_SPEED);
		double turnSpeed = speedToEncoderUnits(jt.getX() * RobotMap.MAX_TURN_SPEED);

	
		adc.execute(driveSpeed, turnSpeed);
		
		
        SmartDashboard.putNumber("Joystick X", jt.getX());
        SmartDashboard.putNumber("Joystick Y", jt.getY());
        SmartDashboard.putNumber("Talon Error", dt.getLeftTalon().getClosedLoopError(RobotMap.VEL_PID));
        
        SmartDashboard.putNumber("Velocity (R)", 
                dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));
        //SmartDashboard.putNumber("Velocity (L)", 
                //encoderUnitsToSpeed(dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));
        SmartDashboard.putNumber("Drive Speed", driveSpeed);
        SmartDashboard.putNumber("Turn Speed", turnSpeed);

        SmartDashboard.putNumber("Motor Output Voltage", dt.getLeftTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("Motor Output Percent", dt.getLeftTalon().getMotorOutputPercent());   
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
