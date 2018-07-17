package org.usfirst.frc.team1072.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 12, 2018
 */
public class PauseUntilPathBegins extends Command
{
    private FollowPath fpc;
    private double delay;
    private double modifiedDelay;
    private double startTime;
    private PauseType pauseType;
    private boolean hasInitializedStartTime;
    private boolean hasInitializedModifiedDelay;
    private double totalTime;
    
    public enum PauseType
    {
        START_OF_PATH, END_OF_PATH, NO_PAUSE
    }
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
     * 
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
    public boolean isFinished()
    {
        return fpc != null 
                && hasInitializedModifiedDelay
                && hasInitializedStartTime
                && fpc.isSetupComplete() 
                && Timer.getFPGATimestamp() - startTime >= modifiedDelay;
    }
}
