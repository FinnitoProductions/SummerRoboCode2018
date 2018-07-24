package org.usfirst.frc.team1072.robot.commands.util;

import org.usfirst.frc.team1072.robot.Robot;

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
        System.out.println(Robot.getCurrentTimeMs());
    }
}
