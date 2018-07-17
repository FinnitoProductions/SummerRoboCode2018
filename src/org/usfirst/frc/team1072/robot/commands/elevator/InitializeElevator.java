package org.usfirst.frc.team1072.robot.commands.elevator;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Initializes the elevator.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeElevator extends Command
{
    private boolean isFinished;
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        System.out.println("INITIALIZING ELEVATOR " + Robot.getCurrentTimeMs());
        Robot.el.talonInitAutonomous();
        isFinished = true;
    }

    /**
    * Determines whether the command has finished.
    * @return true if initialization is complete; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        if (isFinished)
            System.out.println("ELEVATOR INIT FINISHED " + Robot.getCurrentTimeMs());
        return isFinished;
    }
}
