package frc.robot.commands.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Initializes the drivetrain.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeDrivetrain extends Command
{
    /**
     * Whether the initialization has finished.
     */
    private boolean isFinished;
    
    /**
     * Initializes the drivetrain.
     */
    public void initialize()
    {
        ;
        Robot.dt.talonInit();
        isFinished = true;
    }

    /**
    * Determines whether the command has finished.
    * @return true if the initialization has finished; false otherwise
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
