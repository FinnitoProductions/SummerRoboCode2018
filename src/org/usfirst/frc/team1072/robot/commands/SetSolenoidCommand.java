package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a command to set the solenoid to a given state.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class SetSolenoidCommand extends Command
{
    /**
     * Sets up the solenoid command, requiring the intake.
     */
    public SetSolenoidCommand()
    {
        requires(Intake.pn);
    }
    
    /**
     * Executes a given solenoid to a given state.
     * @param key the key in the solenoid map containing the solenoid to be modified
     * @param state the state of the solenoid (forward, off, or reverse)
     */
    public void execute(String key, DoubleSolenoid.Value state)
    {
        Intake.pn.getSolenoid(key).set(state);
    }

    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }
    
}
