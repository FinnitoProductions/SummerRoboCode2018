package frc.robot.commands.drivetrain;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

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
        Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, speed);
        Robot.dt.getRightMaster().set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void end()
    {
        Robot.dt.getLeftMaster().set(ControlMode.Disabled, 0);
        Robot.dt.getRightMaster().set(ControlMode.Disabled, 0);
    }
}
