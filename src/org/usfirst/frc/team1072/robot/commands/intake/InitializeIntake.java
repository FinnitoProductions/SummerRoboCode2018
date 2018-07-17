package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Initializes the intake.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeIntake extends Command
{
    private boolean isFinished;
    
    /**
     * Initializes the intake.
     */
    public void initialize()
    {
        System.out.println("INITIALIZING INTAKE " + Robot.getCurrentTimeMs());
        Robot.intake.talonInit();
        isFinished = true;
    }

    /**
    * Determines whether the command has finished.
    * @return true if the intake has been initialized; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        if (isFinished)
            System.out.println("INTAKE INIT FINISHED " + Robot.getCurrentTimeMs());
        return isFinished;
    }
}
