package org.usfirst.frc.team1072.robot.commands.util;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Prints a given value to the console.
 * @author Finn Frankis
 * @version Jul 21, 2018
 */
public class PrintValueCommand extends InstantCommand
{
    Object value;
    
    /**
     * Constructs a new PrintValueCommand.java.
     * @param value the value to be printed
     */
    public PrintValueCommand (Object value)
    {
        this.value = value;
    }
    
    public void initialize()
    {
        System.out.println(value);
    }
}
