package org.usfirst.frc.team1072.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Pauses the thread (in a series of sequential commands) until the given path begins, waiting a set amount of time either after the command
 * begins or before it ends.
 * @author Finn Frankis
 * @version Jul 12, 2018
 */
public class PauseUntilPathBegins extends Command
{
    /**
     * The path for which this command will wait.
     */
    private FollowPath fpc;
    
    /**
     * The amount of time for which this command will pause, as passed in by the user.
     */
    private double delay;
    
    /**
     * The delay after having been recalculated to reflect the given pause type.
     */
    private double modifiedDelay;
    
    /**
     * The time at which the path began.
     */
    private double startTime;
    
    /**
     * The type of pause which the user chooses to perform.
     */
    private PauseType pauseType;
    
    /**
     * Whether or not startTime has been initialized. This is used to ensure that
     * it is not periodically reset.
     */
    private boolean hasInitializedStartTime;
    
    /**
     * Whether or not modifiedDelay has been initialized. This is used to ensure that
     * it is not periodically reset.
     */
    private boolean hasInitializedModifiedDelay;
    
    /**
     * The total time expected for the path.
     */
    private double totalTime;
    
    /**
     * The various types of pauses which this command can handle.
     * @author Finn Frankis
     * @version Jul 12, 2018
     */
    public enum PauseType
    {
        /**
         * Signifies a pause a given amount of time after the path begins.
         */
        START_OF_PATH, 
        
        /**
         * Signifies a pause a given amount of time before the path ends.
         */
        END_OF_PATH, 
        
        /**
         * Signifies no pause at all.
         */
        NO_PAUSE
    }
    
    /**
     * Constructs a new PauseUntilPathBegins.
     * @param fpc the path to be checked
     */
    public PauseUntilPathBegins(FollowPath fpc)
    {
        this.fpc = fpc;
        startTime = Double.MAX_VALUE;
        delay = 0;
        modifiedDelay = 0;
        pauseType = PauseType.NO_PAUSE;
        hasInitializedStartTime = false;
        hasInitializedModifiedDelay = false;
    }
    
    /**
     * Constructs a new PauseUntilPathBeginsCommand.
     * @param fpc the path to be followed
     * @param pauseType the type of pause to be performed
     * @param delay the time in seconds to wait either from the beginning or end of the path
     * @param totalTime the total time for the path to occur (in ms)
     */
    public PauseUntilPathBegins(FollowPath fpc, PauseType pauseType, double delay, double totalTime)
    {
        this.fpc = fpc;
        this.pauseType = pauseType;
        if (pauseType == PauseType.END_OF_PATH)
        {
            modifiedDelay = 0;
            hasInitializedModifiedDelay = false;
        }
        else
        {
            modifiedDelay = delay;
        }
        this.delay = delay;
        startTime = 0;
        hasInitializedStartTime = false;
        this.totalTime = totalTime / 1000;
    }
    
    /**
     * Executes this command periodically.
     */
    public void execute()
    {
        if (fpc != null)
        {
            if (!hasInitializedModifiedDelay)
            {
                hasInitializedModifiedDelay = true;
                if (pauseType == PauseType.END_OF_PATH)
                    modifiedDelay = totalTime - delay;
                System.out.println(modifiedDelay);
            }

            if (!hasInitializedStartTime && fpc.isSetupComplete())
            {
                hasInitializedStartTime = true;
                startTime = Timer.getFPGATimestamp();
                System.out.println("INIT START TIME");
            }
        }
    }
    
    @Override
    /**
     * Determines whether the command has finished.
     * @return true if the path has begun and modifiedDelay seconds have elapsed since the beginning of the path; false otherwise
     */
    public boolean isFinished()
    {
        return fpc != null 
                && hasInitializedModifiedDelay
                && hasInitializedStartTime
                && fpc.isSetupComplete() 
                && Timer.getFPGATimestamp() - startTime >= modifiedDelay;
    }
}
