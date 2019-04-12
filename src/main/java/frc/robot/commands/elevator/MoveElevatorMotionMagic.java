package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the elevator using the smooth, trapezoidal motion magic.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class MoveElevatorMotionMagic extends Command
{
    /**
     * The intended position for the motion magic closed loop.
     */
    private double position;
    
    /**
     * Creates a new MoveElevatorMotionMagic.
     * @param position the position to which the robot should be moved
     */
    public MoveElevatorMotionMagic(double position) 
    { 
        requires(Robot.el); 
        this.position = position;
    }
    
    /**
     * Initializes the command.
     */
    public void initialize()
    {  
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor
        (FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.el.moveElevatorMotionMagic(position);
        ;
    }
    /**
     * Executes the command, moving the robot to a given position using motion magic.
     */
    public void execute()
    {
        OI oi = OI.getInstance();
        //double targetPos = Math.abs(oi.getGamepad().getRightY()) * 33500 + 1000;
        if (oi.getDriverGamepad().getRightY() >= OI.BLACK_XBOX_ELEVATOR_DEADBAND)
            this.cancel();
    }

   /**
    * Determines whether the command is complete.
    * 
    * @return true if the command has finished; false otherwise
    */
    protected boolean isFinished() {
        return Math.abs(Robot.el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) - position) <= 
                Elevator.POS_ALLOWABLE_ERROR;
    }
    
}
