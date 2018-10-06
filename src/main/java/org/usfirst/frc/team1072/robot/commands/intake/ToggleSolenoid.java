package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Represents a command to set the solenoid to a given state.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class ToggleSolenoid extends InstantCommand {
	/**
	 * The key for the solenoid to be actuated in the solenoid map.
	 */
	private String solenoidKey;

	/**
	 * Sets up the solenoid command, requiring the intake.
	 * 
	 * @param key   the key in the solenoid map containing the solenoid to be
	 *              modified
	 * @param state the state of the solenoid (forward, off, or reverse)
	 */
	public ToggleSolenoid(String key) {
        solenoidKey = key;
    }
    
    public void initialize () {
        Intake.pn.getSolenoid(solenoidKey).set(Intake.pn.getSolenoid(solenoidKey).get().equals(DoubleSolenoid.Value.kForward) ? 
        DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }
}
