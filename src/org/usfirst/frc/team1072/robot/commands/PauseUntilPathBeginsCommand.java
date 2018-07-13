package org.usfirst.frc.team1072.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 12, 2018
 */
public class PauseUntilPathBeginsCommand extends Command
{
    private FollowPathCommand fpc;
    private double delay;
    private double modifiedDelay;
    private double startTime;
    private PauseType pauseType;
    private boolean hasInitializedStartTime;
    private boolean hasInitializedModifiedDelay;
    
    public enum PauseType
    {
        START_OF_PATH, END_OF_PATH
    }
    public PauseUntilPathBeginsCommand(FollowPathCommand fpc)
    {
        this.fpc = fpc;
        startTime = Double.MAX_VALUE;
        delay = 0;
        modifiedDelay = 0;
        pauseType = PauseType.START_OF_PATH;
        hasInitializedStartTime = false;
        hasInitializedModifiedDelay = false;
    }
    
    /**
     * Constructs a new PauseUntilPathBeginsCommand.
     * @param fpc the path to be followed
     * 
     * @param delay the time in seconds to wait either from the beginning or end of the path
     */
    public PauseUntilPathBeginsCommand(FollowPathCommand fpc, PauseType pauseType, double delay)
    {
        this.fpc = fpc;
        this.pauseType = pauseType;
        if (pauseType == PauseType.END_OF_PATH)
        {
            modifiedDelay = Double.MAX_VALUE;
            hasInitializedModifiedDelay = false;
        }
        else
            modifiedDelay = delay;
        this.delay = delay;
        startTime = Double.MAX_VALUE;
        hasInitializedStartTime = false;
    }
    
    public void execute()
    {
        if (fpc != null)
        {
            if (!hasInitializedModifiedDelay && pauseType == PauseType.END_OF_PATH)
            {
                hasInitializedModifiedDelay = true;
                modifiedDelay = fpc.getTotalTime()/1000 - delay;
                System.out.println("INIT MODDED DELAY");
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
        System.out.println("NULL: " + fpc == null);
        System.out.println("PATH FINISHED? " + fpc.getFinished());
        System.out.println("START TIME? " + startTime);
        return fpc != null && fpc.isSetupComplete() && Timer.getFPGATimestamp() - startTime >= modifiedDelay;
    }
}
