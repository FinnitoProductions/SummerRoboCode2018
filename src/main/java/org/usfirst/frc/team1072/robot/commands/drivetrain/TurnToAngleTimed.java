package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.TurnDirection;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class TurnToAngleTimed extends TimedCommand {
	private TurnDirection direction;
	public TurnToAngleTimed (double time, TurnDirection direction) {
		super (time);
		requires(Robot.dt);
		this.direction = direction;
	}
    
    /**
     * Executes the command to drive with a given velocity.
     */
    @Override
    public void execute() 
    {
        Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, ((direction == TurnDirection.RIGHT) ? 1 : -1));
        Robot.dt.getRightMaster().set(ControlMode.PercentOutput, ((direction == TurnDirection.RIGHT) ? -1 : 1));
        System.out.println("turn angle");
    }

    @Override
    public void end()
    {
        Robot.dt.getLeftMaster().set(ControlMode.Disabled, 0);
        Robot.dt.getRightMaster().set(ControlMode.Disabled, 0);
    }
}