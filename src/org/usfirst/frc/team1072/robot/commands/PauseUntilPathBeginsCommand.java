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
    private double startTime;
    
    public PauseUntilPathBeginsCommand(FollowPathCommand fpc)
    {
        this.fpc = fpc;
        startTime = Double.MAX_VALUE;
    }
    
    /**
     * Constructs a new PauseUntilPathBeginsCommand.
     * @param fpc the path to be followed
     * @param delay the time in seconds to wait AFTER the path begins
     */
    public PauseUntilPathBeginsCommand(FollowPathCommand fpc, double delay)
    {
        this.fpc = fpc;
        this.delay = delay;
        startTime = Double.MAX_VALUE;
    }
    
    public void execute()
    {
        if (startTime > Double.MAX_VALUE - 10 && fpc.isSetupComplete())
            startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public boolean isFinished()
    {
        return fpc.isSetupComplete() && Timer.getFPGATimestamp() - startTime >= delay;
    }
}
