package org.usfirst.frc.team1072.robot.commands.auton;

import org.usfirst.frc.team1072.robot.auto.paths.Path;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.Pigeon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory.Segment;

public class FollowPathRio extends Command implements java.lang.Runnable {
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
	
	public FollowPathRio (Path path, TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.leftPath = path.getLeftPath().segments;
		this.rightTalon = rightTalon;
		this.rightPath = path.getRightPath().segments;
		this.path = path;
		
		angleErrorPrev = -1;
		
		processPoints = new Notifier (this);
	}
	
	public void initialize() {
		if (leftPath.length > 0 && rightPath.length > 0)
		{
			prevTime = Timer.getFPGATimestamp();
			processPoints.startPeriodic(path.getDt());
		}
		else
			isFinished = true;
	}
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	@Override
	public void run() {
		if (currentPointIdx >= leftPath.length || currentPointIdx > rightPath.length) {
			isFinished = true;
			return;
		}
		
		Segment leftPoint = leftPath[currentPointIdx];
		Segment rightPoint = rightPath[currentPointIdx];
		double dt = Timer.getFPGATimestamp() - prevTime;
		prevTime = Timer.getFPGATimestamp();
		
		double angleErrorCurrent = leftPath[currentPointIdx].heading - leftTalon.getSelectedSensorPosition(PID_AUXILIARY);
		angleErrorAccum += angleErrorCurrent * dt;
		double gyroFactor = angleErrorCurrent * Drivetrain.Pigeon.MOT_PROF_KP + angleErrorAccum * Drivetrain.Pigeon.MOT_PROF_KI + 
				(!hasRunOnce ? 0 : angleErrorCurrent - angleErrorPrev) / dt * Drivetrain.Pigeon.MOT_PROF_KD;
		angleErrorPrev = angleErrorCurrent;

		double velocityDifference = (leftPoint.velocity - rightPoint.velocity) * Drivetrain.Pigeon.MOT_PROF_KF;
		
		double leftFeedForward = leftPoint.velocity * Drivetrain.MOTION_PROF_KF_LEFT + velocityDifference;
		double rightFeedForward = rightPoint.velocity * Drivetrain.MOTION_PROF_KF_RIGHT - velocityDifference;
		
		leftTalon.set(ControlMode.Position, leftPoint.position + gyroFactor, DemandType.ArbitraryFeedForward, leftFeedForward);
		rightTalon.set(ControlMode.Position, rightPoint.position - gyroFactor, DemandType.ArbitraryFeedForward, rightFeedForward);
		currentPointIdx++;
	}
	

}
