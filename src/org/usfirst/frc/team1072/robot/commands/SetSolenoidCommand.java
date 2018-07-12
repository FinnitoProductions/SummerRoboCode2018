package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Represents a command to set the solenoid to a given state.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class SetSolenoidCommand extends InstantCommand
{
    private String solenoidKey;
    private DoubleSolenoid.Value solenoidState;
    /**
     * Sets up the solenoid command, requiring the intake.
     */
    public SetSolenoidCommand(String key, DoubleSolenoid.Value state) 
    { 
        requires(Intake.pn); 
        solenoidKey = key;
        solenoidState = state;
    }
    
    /**
     * Sets a given solenoid to a given state.
     * @param key the key in the solenoid map containing the solenoid to be modified
     * @param state the state of the solenoid (forward, off, or reverse)
     */
    public void initialize()
    {
        System.out.println("ACTUATING SOLENOID " + solenoidKey);
        Intake.pn.getSolenoid(solenoidKey).set(solenoidState); 
    }
}
