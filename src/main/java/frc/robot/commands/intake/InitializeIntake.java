package frc.robot.commands.intake;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Initializes the intake.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeIntake extends Command
{
    /**
     * Whether or not the intake has been successfully initialized.
     */
    private boolean isFinished;
    
    /**
     * Initializes the intake.
     */
    public void initialize()
    {
        ;
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
            ;
        return isFinished;
    }
}
