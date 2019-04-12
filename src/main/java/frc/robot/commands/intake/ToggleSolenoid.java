package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics.SolenoidType;

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
	private SolenoidType type;

	/**
	 * Sets up the solenoid command, requiring the intake.
	 * 
	 * @param key   the key in the solenoid map containing the solenoid to be
	 *              modified
	 * @param state the state of the solenoid (forward, off, or reverse)
	 */
	public ToggleSolenoid(SolenoidType type) {
        this.type = type;
    }
    
    public void initialize () {
        Intake.pn.toggleSolenoid(type);
    }
}
