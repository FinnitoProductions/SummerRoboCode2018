package frc.robot.commands.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * 
 * @author Finn Frankis
 * @version Jul 24, 2018
 */
public class PrintTimeToConsole extends InstantCommand
{
    public void initialize()
    {
        System.out.println(Timer.getFPGATimestamp());
    }
}
