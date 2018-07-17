package org.usfirst.frc.team1072.robot.commands.auton;

import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPositionCommand;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Pauses a sequence of command until a given position is reached.
 * @author Finn Frankis
 * @version Jul 17, 2018
 */
public class PauseUntilReachingPosition extends Command
{
    private DriveToPositionCommand dtpc;
    private double fraction;
    
    /**
     * Constructs a new PauseUntilReachingPosition.
     * @param dtpc the path where distance will be tracked
     * @param fraction the fraction of distance traveled out of the total at which the path will cease execution
     */
    public PauseUntilReachingPosition (DriveToPositionCommand dtpc, double fraction)
    {
        this.dtpc = dtpc;
        this.fraction = fraction;
    }
    /**
    * Determines whether the command has finished.
    * @return true if the distance traveled is at least the fraction away from the total distance;
    * false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        return dtpc.getNumExecutes() == -1 && 
                dtpc.getCurrentPosition()/dtpc.getDesiredPosition() >= fraction;
    }

}
