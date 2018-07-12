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
            modifiedDelay = Double.MAX_VALUE;
        else
            modifiedDelay = delay;
        this.delay = delay;
        startTime = Double.MAX_VALUE;
    }
    
    public void execute()
    {
        if (fpc != null)
        {
            if (modifiedDelay > (Double.MAX_VALUE - 100) && pauseType == PauseType.END_OF_PATH)
                modifiedDelay = fpc.getTotalTime() - delay;
            if (startTime > Double.MAX_VALUE - 100 && fpc.isSetupComplete())
                startTime = Timer.getFPGATimestamp();
        }
    }
    
    @Override
    public boolean isFinished()
    {
        return fpc != null && fpc.getFinished() && Timer.getFPGATimestamp() - startTime >= modifiedDelay;
    }
}
