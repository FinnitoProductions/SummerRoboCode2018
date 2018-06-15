/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.robot.commands.ArcadeDriveCommand;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.ExampleCommand;
import org.usfirst.frc.team1072.robot.commands.IntakeOuttakeCubeCommand;
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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Drivetrain dt;
    public static Intake intake;
    public static Elevator el;
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
	    slaveVictors();
	    //initTalonOutput(0);
        
        invertControllers();
        setNeutralMode(NeutralMode.Brake);
        
        setRampTime(RobotMap.MAX_RAMP_TIME);
        
        configureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        scaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        
        setSensorPhase();
        configureVelocityClosedLoop();
        configurePositionClosedLoop();
        
        setCurrentLimit(RobotMap.DT_PEAK_CURRENT_LIMIT, RobotMap.DT_PEAK_TIME_MS, RobotMap.DT_CONTINUOUS_CURRENT_LIMIT);
        
	}
	
	public void elevatorTalonInit()
	{
	    elevatorSlaveVictors();
	    elevatorSetNeutralMode(NeutralMode.Brake);
	    elevatorSetCurrentLimit(RobotMap.EL_PEAK_CURRENT_LIMIT, 
	            RobotMap.EL_PEAK_TIME_MS, RobotMap.EL_CONTINOUS_CURRENT_LIMIT);
	    elevatorInvertControllers();
	    elevatorScaleVoltage(RobotMap.EL_NOMINAL_OUTPUT);
	    elevatorSetSoftLimit(35000);
	    
	    scaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
	    elevatorConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
	    elevatorZeroSensors();
	    
	    elevatorConfigurePositionClosedLoop();
	    
	    
	    
	}
	
	private void elevatorSlaveVictors()
	{
	    TalonSRX talon = el.getBottomRightTalon();
        el.getBottomLeftVictor().follow(talon);
        el.getTopLeftVictor().follow(talon);
        el.getTopRightVictor().follow(talon);
	}
	
	private void elevatorInvertControllers()
	{
	    el.getBottomLeftVictor().setInverted(true);
	}
	
	private void elevatorSetNeutralMode(NeutralMode n)
	{
	    el.getBottomRightTalon().setNeutralMode(n);
	    el.getTopRightVictor().setNeutralMode(n);
	    el.getBottomLeftVictor().setNeutralMode(n);
	    el.getTopLeftVictor().setNeutralMode(n);
	}
	
	private void elevatorScaleVoltage(double nomVoltage)
    {
        el.getBottomRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT); 
    }
	
	private void elevatorSetSoftLimit (int softLimit)
	{
	    el.getBottomRightTalon().configForwardSoftLimitThreshold(softLimit, RobotMap.TIMEOUT);
	    el.getBottomRightTalon().configForwardSoftLimitEnable(true, RobotMap.TIMEOUT);
	}
	
	private void elevatorConfigurePositionClosedLoop()
	{
	    el.getBottomRightTalon().configNominalOutputForward(RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
	    el.getBottomRightTalon().configNominalOutputReverse(-1 * RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configPeakOutputForward(RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configPeakOutputReverse(-1 * RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kF(RobotMap.EL_POS_PID, RobotMap.EL_POS_KF, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kP(RobotMap.EL_POS_PID, RobotMap.EL_POS_KP, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kI(RobotMap.EL_POS_PID, RobotMap.EL_POS_KI, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().config_kD(RobotMap.EL_POS_PID, RobotMap.EL_POS_KD, RobotMap.TIMEOUT);
	}
	
	private void elevatorSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
	{
	    el.getBottomRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

	    el.getBottomRightTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        
        el.getBottomRightTalon().enableCurrentLimit(true);
 
	}
	
	private void elevatorConfigureSensors(FeedbackDevice fd)
	{
	    el.getBottomRightTalon().configSelectedFeedbackSensor(fd, RobotMap.POS_PID, RobotMap.TIMEOUT);
	}
	
	private void elevatorZeroSensors()
	{
	    el.getBottomRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
	}
	
	private void intakeTalonInit()
	{
	    intakeSetNeutralMode(NeutralMode.Brake);
	    intakeSetCurrentLimit(RobotMap.INT_PEAK_CURRENT_LIMIT, RobotMap.INT_PEAK_TIME_MS, RobotMap.INT_CONTINUOUS_CURRENT_LIMIT);
	}
	
	private void intakeSetNeutralMode (NeutralMode nm)
	{
	    intake.getLeftTalon().setNeutralMode(nm);
	    intake.getRightTalon().setNeutralMode(nm);
	}
	
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
	    dt.getLeftTalon().setInverted(true);
	    dt.getLeftVictor().setInverted(true);
        // Invert the following direction (left Talons and Victors were wired oppositely)
        dt.getRightTalon().setInverted(false);
        dt.getRightVictor().setInverted(false); 
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
	
	private void setSensorPhase()
	{
	    dt.getLeftTalon().setSensorPhase(false);
	    dt.getRightTalon().setSensorPhase(false);
	}
	/**
	 * Configures the velocity closed loop.
	 */
	private void configureVelocityClosedLoop()
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
	
	private void configurePositionClosedLoop()
	{
	    dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
	    dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
	    
	    dt.getLeftTalon().configAllowableClosedloopError(100, RobotMap.POS_PID, RobotMap.TIMEOUT);
	    dt.getRightTalon().configAllowableClosedloopError(100, RobotMap.POS_PID, RobotMap.TIMEOUT);
	    
	    dt.getLeftTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_RIGHT, RobotMap.TIMEOUT);
        
        dt.getLeftTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_RIGHT, RobotMap.TIMEOUT);
        
        dt.getLeftTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_RIGHT, RobotMap.TIMEOUT);
        
        dt.getLeftTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_LEFT, RobotMap.TIMEOUT);
        dt.getRightTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_RIGHT, RobotMap.TIMEOUT);
	}
	
	private void setCurrentLimit (int peakCurrentLimit, int peakTime, int continuousLimit)
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
		driveTrainTalonInit();
		elevatorTalonInit();
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
		driveTrainTalonInit();
		elevatorTalonInit();
	}

	/**
	 * This function is called periodically during operator control.
	*/
	public void teleopPeriodic() {
		/*ArcadeDriveCommand adc = new ArcadeDriveCommand();
		
		// all the way up for y stick is given as -1
		double driveSpeed = speedToEncoderUnits(-1 * jt.getY() * RobotMap.MAX_DRIVE_SPEED);
		double turnSpeed = speedToEncoderUnits(-1 * jt.getX() * RobotMap.MAX_TURN_SPEED);

	
		adc.execute(driveSpeed, turnSpeed);*/
		
		/*double targetPos = -1 * jt.getY() * RobotMap.TICKS_PER_REV * 3;
		DriveToPositionCommand dtp = new DriveToPositionCommand();
		dtp.execute(targetPos);
		
		SmartDashboard.putNumber("Drivetrain Current (L)", dt.getLeftTalon().getOutputCurrent());
		SmartDashboard.putNumber("Drivetrain Current (R)", dt.getRightTalon().getOutputCurrent());
        //SmartDashboard.putNumber("Joystick X", jt.getX());
        //SmartDashboard.putNumber("Joystick Y", jt.getY());
        SmartDashboard.putNumber("Talon Error", dt.getLeftTalon().getClosedLoopError(RobotMap.POS_PID));*/
        
        /*SetCompressorCommand tc = new SetCompressorCommand();
        
        tc.execute(true);
        
        SetSolenoidCommand ssc = new SetSolenoidCommand();
        int i = jt.getPOV();
        
        if (i > -1 && i <= 1)
            ssc.execute(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_UP);
        if (i >= 269 && i <= 271)
            ssc.execute(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_DECOMPRESS);
        if (i >= 179 && i <= 181)
            ssc.execute(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN);
        if (i >= 89 && i <= 91)
            ssc.execute(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_COMPRESS);
        
        SmartDashboard.putNumber("D-Pad", Math.round(jt.getPOV()));*/
		
		/*MoveElevatorVelocityCommand mevc = new MoveElevatorVelocityCommand();
		mevc.execute(-0.3 * jt.getY());
		SmartDashboard.putNumber("Talon Current", el.getBottomRightTalon().getOutputCurrent());*/
	    
	    /*MoveElevatorPositionCommand mepc = new MoveElevatorPositionCommand();
	    mepc.execute(Math.abs(jt.getY()) * 17000);
		//SmartDashboard.putNumber("Elevator Velocity", el.getBottomRightTalon().getSelectedSensorVelocity(RobotMap.EL_VEL_PID));
		SmartDashboard.putNumber("Elevator Position", el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.EL_POS_PID));
        SmartDashboard.putNumber("Output Voltage", el.getBottomRightTalon().getMotorOutputVoltage());
		SmartDashboard.putNumber("Talon Error", el.getBottomRightTalon().getClosedLoopError(RobotMap.POS_PID));*/
	    
	    IntakeOuttakeCubeCommand iocc = new IntakeOuttakeCubeCommand();
	    iocc.execute(jt.getY(), jt.getY());
        /*/*SmartDashboard.putNumber("Position (R)", 
                encoderUnitsToSpeed(dt.getRightTalon().getSelectedSensorVelocity(RobotMap.POS_PID)));*/
        /*SmartDashboard.putNumber("Position (L)", 
              dt.getLeftTalon().getSelectedSensorPosition(RobotMap.POS_PID));
        SmartDashboard.putNumber("Velocity (L)", 
                dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.VEL_PID));
        SmartDashboard.putNumber("Target Position", targetPos);

        SmartDashboard.putNumber("Motor Output Voltage", dt.getLeftTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("Motor Output Percent", dt.getLeftTalon().getMotorOutputPercent());  */ 
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
