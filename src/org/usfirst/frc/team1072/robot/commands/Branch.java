package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a branch of sequential commands, where multiple branches can be run in parallel.
 * @author Finn Frankis
 * @version Jul 11, 2018
 */
public class Branch
{
    ArrayList<Command> branch = new ArrayList<Command>();
    public Branch()
    {
        
    }
    
    public void addCommand(Command c)
    {
        
    }
}
