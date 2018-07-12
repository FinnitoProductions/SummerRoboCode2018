package org.usfirst.frc.team1072.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 12, 2018
 */
public class PauseUntilPathBeginsCommand extends Command
{
    private FollowPathCommand fpc;
    public PauseUntilPathBeginsCommand(FollowPathCommand fpc)
    {
        this.fpc = fpc;
    }
    
    @Override
    public boolean isFinished()
    {
        return fpc.isSetupComplete();
    }
}
