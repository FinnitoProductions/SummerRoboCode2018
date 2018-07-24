package org.usfirst.frc.team1072.robot.commands.util;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * 
 * @author Finn Frankis
 * @version Jul 24, 2018
 */
public class PrintValueToConsole extends InstantCommand
{
    private Object value;
    
    public PrintValueToConsole(Object value)
    {
        this.value = value;
    }
    public void initialize()
    {
        System.out.println(value);
    }
}
