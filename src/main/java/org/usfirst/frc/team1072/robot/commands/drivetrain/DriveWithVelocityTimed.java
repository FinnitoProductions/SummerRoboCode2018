package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveWithVelocityTimed extends TimedCommand {
    private double speed;
	public DriveWithVelocityTimed(double speed, double time) {
		super (time);
		requires(Robot.dt);
		this.speed = speed;
	}
    
    /**
     * Executes the command to drive with a given velocity.
     */
    @Override
    public void execute() 
    {
        Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, -speed);
        Robot.dt.getRightMaster().set(ControlMode.PercentOutput, speed);
    }
}
