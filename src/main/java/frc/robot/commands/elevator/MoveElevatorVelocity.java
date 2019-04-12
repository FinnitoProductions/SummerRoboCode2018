package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import harkerrobolib.util.MathUtil;

/**
 * Represents a command to continually move the elevator given a velocity.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class MoveElevatorVelocity extends Command
{
    /**
     * Creates a new MoveElevatorVelocity.
     */
    public MoveElevatorVelocity() { requires(Robot.el); }
    
    /**
     * Initializes the command, including necessary sensors.
     */
    public void initialize()
    {
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    }
    /**
     * Executes the command using the speed from joystick input.
     */
    public void execute() 
    { 
        OI oi = OI.getInstance();
        if (Math.abs(oi.getDriverGamepad().getRightY()) > OI.BLACK_XBOX_ELEVATOR_DEADBAND)
        {
        	double rightY = oi.getDriverGamepad().getRightY();
            boolean isDown = rightY < 0;
            boolean reverseBeyondLimit = Robot.el.getBottomRightTalon()
                    .getSelectedSensorPosition(Drivetrain.POS_PID) <= Elevator.REVERSE_SOFT;
            double currentSpeed = Robot.el.getBottomRightTalon().getSelectedSensorVelocity(RobotMap.PRIMARY_PID_INDEX);
            double dist = Math.abs(Robot.el.getBottomRightTalon()
                    .getSelectedSensorPosition(Drivetrain.POS_PID) - Elevator.REVERSE_SOFT);
            double outputFactor = 1.0;
            if (isDown && reverseBeyondLimit && Math.abs(currentSpeed) / Elevator.MAX_ELEVATOR_SPEED < Elevator.SLOW_DOWN_SPEED)
            {
                outputFactor = MathUtil.map(dist, 0, Elevator.REVERSE_SOFT, 1, 0);
            }
            else if (Math.abs(currentSpeed) / Elevator.MAX_ELEVATOR_SPEED >= Elevator.SLOW_DOWN_SPEED) {
            	outputFactor = MathUtil.map(dist, 0, Elevator.REVERSE_SOFT, 1, -0.5);
            }
            rightY *= outputFactor;
            Robot.el.moveElevatorVelocity(rightY);
        }
        else
            Robot.el.moveElevatorVelocity(0);
    }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() { return false; }
}
