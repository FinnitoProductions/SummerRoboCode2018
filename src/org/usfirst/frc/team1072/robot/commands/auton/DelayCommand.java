package org.usfirst.frc.team1072.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class DelayCommand extends CommandGroup
{
    private double delay;
    private double startTime;
    
    public DelayCommand (double time)
    {
        delay = time;
        startTime = Timer.getFPGATimestamp();
    }
    
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        return Timer.getFPGATimestamp() - startTime >= delay;
    }
    
}
