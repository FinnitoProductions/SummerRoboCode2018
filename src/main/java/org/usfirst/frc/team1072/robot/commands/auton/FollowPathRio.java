package org.usfirst.frc.team1072.robot.commands.auton;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import harkerrobolib.auto.Path;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.AngleUnit;
import harkerrobolib.util.Conversions.PositionUnit;
import jaci.pathfinder.Trajectory.Segment;

public class FollowPathRio extends Command implements java.lang.Runnable {
	private static TalonSRX defaultLeftTalon;
	private static TalonSRX defaultRightTalon;
	
	private boolean isFinished;
	
	private static final int PID_PRIMARY = 0;
	private static final int PID_AUXILIARY = 1;
	
	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	private Path path;
	
	private Segment[] leftPath;
	private Segment[] rightPath;
	
	private Notifier processPoints;
	
	private int currentPointIdx;
	private double prevTime;
	private double angleErrorAccum;
	private double angleErrorPrev;
	private boolean hasRunOnce;
	private boolean shouldHold;

	private double leftSetpoint;
	private double rightSetpoint;
	private double leftFeedForward;
	private double rightFeedForward;

	private double leftPrevError;
	private double rightPrevError;

	private int numValidRuns;

	private static final int REQUIRED_VALID_RUNS = 10;
	
	public FollowPathRio (Path path, TalonSRX leftTalon, TalonSRX rightTalon) {
		System.out.println("CONSTRUCTING");
		this.leftTalon = leftTalon;
		this.leftPath = path.getLeftPath().segments;
		this.rightTalon = rightTalon;
		this.rightPath = path.getRightPath().segments;
		this.path = path;
		
		angleErrorPrev = -1;
		numValidRuns = 0;
		
		processPoints = new Notifier (this);
	}
	
	public FollowPathRio (Path path) {
		this(path, defaultLeftTalon, defaultRightTalon);
	}
	
	public void initialize() {
		System.out.println("INITIALIZING");
		if (leftPath.length > 0 && rightPath.length > 0)
		{
			System.out.println("PATH LENGTH NOT ZERO");

			prevTime = Timer.getFPGATimestamp();

			System.out.println(path.getDt());
			processPoints.startPeriodic(path.getDt());

			Robot.dt.setTalonSensorPhase(Drivetrain.LEFT_TALON_PHASE, 
                Drivetrain.RIGHT_TALON_PHASE);
        
			Robot.dt.configureMotionProfileDriveClosedLoop();
			Robot.dt.configureMotionProfileAngleClosedLoop();
			
			Robot.dt.getLeftMaster().selectProfileSlot(Drivetrain.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
			Robot.dt.getRightMaster().selectProfileSlot(Drivetrain.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
			
			Robot.dt.getLeftMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
			Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);

			Robot.dt.getLeftMaster().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
					RemoteSensorSource.Pigeon_Yaw, 
					RobotMap.REMOTE_SLOT_0, 
					RobotMap.TIMEOUT);
			Robot.dt.getRightMaster().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
					RemoteSensorSource.Pigeon_Yaw, 
					RobotMap.REMOTE_SLOT_0, 
					RobotMap.TIMEOUT);

			Robot.dt.getLeftMaster().configSelectedFeedbackSensor(Drivetrain.Pigeon.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
			Robot.dt.getRightMaster().configSelectedFeedbackSensor(Drivetrain.Pigeon.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);

			Robot.dt.getLeftMaster().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);
			Robot.dt.getRightMaster().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);
	}
		else
			isFinished = true;
	}
	
	@Override
	protected boolean isFinished() {
		if (shouldHold) {
			leftPrevError = leftSetpoint - Robot.dt.getLeftMaster().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX);
			rightPrevError = rightSetpoint - Robot.dt.getRightMaster().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX);

			if (Math.abs(leftPrevError) < Drivetrain.MOTION_PROFILE_ALLOWABLE_ERROR && Math.abs(rightPrevError) < Drivetrain.MOTION_PROFILE_ALLOWABLE_ERROR) 
				numValidRuns++;
			else
				numValidRuns = 0;
			return numValidRuns >= REQUIRED_VALID_RUNS;
		}
		return false;	
	}

	@Override
	public void run() {
		//System.out.println("RUNNING");
		if (currentPointIdx >= leftPath.length || currentPointIdx >= rightPath.length) {
			shouldHold = true;
		}
		
		if (!shouldHold) {
			Segment leftPoint = leftPath[currentPointIdx];
			Segment rightPoint = rightPath[currentPointIdx];

			double dt = Timer.getFPGATimestamp() - prevTime;
			prevTime = Timer.getFPGATimestamp();
			
			double angleSetpoint = Conversions.convertAngle(AngleUnit.RADIANS, leftPath[currentPointIdx].heading, AngleUnit.DEGREES);
			double angleError = Conversions.convertAngle(AngleUnit.PIGEON_UNITS, leftTalon.getSelectedSensorPosition(PID_AUXILIARY), AngleUnit.DEGREES);
			double angleErrorCurrent = angleSetpoint - angleError;

			angleErrorAccum += angleErrorCurrent * dt;
			double gyroFactor = angleErrorCurrent * Drivetrain.Pigeon.MOT_PROF_KP + angleErrorAccum * Drivetrain.Pigeon.MOT_PROF_KI + 
					(!hasRunOnce ? 0 : angleErrorCurrent - angleErrorPrev) / dt * Drivetrain.Pigeon.MOT_PROF_KD;
			angleErrorPrev = angleErrorCurrent;

			double velocityDifference = (leftPoint.velocity - rightPoint.velocity) * Drivetrain.Pigeon.MOT_PROF_KF;
			
			leftFeedForward = leftPoint.velocity * Drivetrain.MOTION_PROF_KF_LEFT + velocityDifference;
			rightFeedForward = rightPoint.velocity * Drivetrain.MOTION_PROF_KF_RIGHT - velocityDifference;
			
			leftSetpoint = Conversions.convertPosition(PositionUnit.FEET, leftPoint.position + gyroFactor, PositionUnit.ENCODER_UNITS);
			rightSetpoint = Conversions.convertPosition(PositionUnit.FEET, rightPoint.position - gyroFactor, PositionUnit.ENCODER_UNITS);

			SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
			SmartDashboard.putNumber("Right Setpoint", rightSetpoint);

			leftTalon.set(ControlMode.Position, leftSetpoint, 
			DemandType.ArbitraryFeedForward, 
			leftFeedForward);
			rightTalon.set(ControlMode.Position, rightSetpoint, 
			DemandType.ArbitraryFeedForward, 
			rightFeedForward);

			currentPointIdx++;
		}
		else {
			leftTalon.set(ControlMode.Position, leftSetpoint);
			rightTalon.set(ControlMode.Position, rightSetpoint);
		}
		hasRunOnce = true;
	}
	
	public static void setDefaultLeftTalon (TalonSRX defaultLeft) {
		defaultLeftTalon = defaultLeft;
	}
	
	public static void setDefaultRightTalon (TalonSRX defaultRight) {
		defaultRightTalon = defaultRight;
	}

	@Override
	public void end () {
		processPoints.stop();
	}

}
