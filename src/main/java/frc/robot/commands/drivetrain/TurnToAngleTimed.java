package frc.robot.commands.drivetrain;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.TurnDirection;
import frc.robot.subsystems.Elevator;

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