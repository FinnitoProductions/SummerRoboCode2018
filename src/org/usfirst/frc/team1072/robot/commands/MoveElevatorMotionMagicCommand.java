package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the elevator using the smooth, trapezoidal motion magic.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class MoveElevatorMotionMagicCommand extends Command
{
    double position;
    double timeout;
    
    /**
     * Creates a new command requiring the elevator.
     */
    public MoveElevatorMotionMagicCommand(double timeout, double position) 
    { 
        requires(Robot.el); 
        this.position = position;
        this.timeout = timeout;
    }
    
    public void initialize()
    {
        try { Thread.sleep((long) timeout); } catch (Exception e) {e.printStackTrace(); }
       
        Robot.el.moveElevatorMotionMagic(position);
    }
    /**
     * Executes the command, moving the robot to a given position using motion magic.
     * @param targetPos the position to which the robot should be moved
     */
    public void execute()
    {
        OI oi = OI.getInstance();
        //double targetPos = Math.abs(oi.getGamepad().getRightY()) * 33500 + 1000;
        if (oi.getGamepad().getRightY() >= OI.BLACK_XBOX_DEADBAND)
            this.cancel();
    }

   /**
    * Determines whether the command is complete.
    * 
    * @return true if the command has finished; false otherwise
    */
    protected boolean isFinished() {
        return Math.abs(Robot.el.getBottomRightTalon().getSelectedSensorPosition(DrivetrainConstants.POS_PID) - position) <= 
                ElevatorConstants.POS_ALLOWABLE_ERROR;
    }
    
}
