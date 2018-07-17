package org.usfirst.frc.team1072.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Accurately delays (when in a series of sequential commands) for a given amount of time.
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class Delay extends CommandGroup
{
    private double delay;
    private double startTime;
    
    /**
     * Constructs a new Delay.
     * @param time the amount of time (in seconds) for which the command should delay
     */
    public Delay (double time)
    {
        delay = time;
        startTime = Timer.getFPGATimestamp();
    }
    
    /**
    * Determines whether the command has finished.
    * @return true if delay seconds have elapsed; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        return Timer.getFPGATimestamp() - startTime >= delay;
    }
    
}
