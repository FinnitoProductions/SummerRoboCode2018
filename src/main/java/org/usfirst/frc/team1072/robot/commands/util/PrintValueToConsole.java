package org.usfirst.frc.team1072.robot.commands.util;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Prints a given value to the console.
 * @author Finn Frankis
 * @version Jul 24, 2018
 */
public class PrintValueToConsole extends InstantCommand
{
    /**
     * The value to be printed.
     */
    private Object value;
    
    /**
     * Constructs a new PrintValueToConsole.
     * @param value the value to be printed
     */
    public PrintValueToConsole(Object value)
    {
        this.value = value;
    }
    
    /**
     * Initializes the command by printing the passed-in value.
     */
    public void initialize()
    {
        System.out.println(value);
    }
}
