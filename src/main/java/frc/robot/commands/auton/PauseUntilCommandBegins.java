package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Pauses, when in a series of sequential commands, until (and any time after) a command begins.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class PauseUntilCommandBegins extends Command
{
    /**
     * The command for which this command pauses.
     */
    private Command c;
    
    /**
     * The amount of time (in seconds) to pause after the command begins running.
     */
    private double delay;
    
    /**
     * The time at which the command in question began.
     */
    private double startTime;
    
    /**
     * Whether or not startTime has been initialized. This is used to ensure that
     * it is not periodically reset.
     */
    private boolean hasInitializedStartTime;
    
    /**
     * 
     * Constructs a new PauseUntilCommandBegins.
     * @param c the command 
     * @param delay the delay, in seconds, to pause after this command begins
     */
    public PauseUntilCommandBegins(Command c, double delay)
    {
        this.c = c;
        this.delay = delay;
    }
    
    /**
     * Initializes the command.
     */
    public void initialize()
    {
        hasInitializedStartTime = false;
        startTime = Double.MAX_VALUE;
    }
    
    /**
     * Executes the command periodically.
     */
    public void execute ()
    {
        if (!hasInitializedStartTime && c.isRunning())
        {
            startTime = Timer.getFPGATimestamp();
            hasInitializedStartTime = true;
        }
    }
    /**
     * Determines whether the command has finished.
    * @return true if the commmand has started to run and delay seconds have elapsed after the command began
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return hasInitializedStartTime &&
                Timer.getFPGATimestamp() - startTime >= delay;
    }
    
}
