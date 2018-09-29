package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team1072.robot.RobotMap;

/**
 * Represents a command to set the solenoid to a given state.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class ToggleSolenoid extends SetSolenoid {
	/**
	 * The key for the solenoid to be actuated in the solenoid map.
	 */
	private String solenoidKey;

	/**
	 * The intended state for the solenoid.
	 */
	private DoubleSolenoid.Value solenoidState;

	/**
	 * Sets up the solenoid command, requiring the intake.
	 * 
	 * @param key   the key in the solenoid map containing the solenoid to be
	 *              modified
	 * @param state the state of the solenoid (forward, off, or reverse)
	 */
	public ToggleSolenoid(String key) {
        super (key, 
        Intake.pn.getSolenoid(key).get().equals(DoubleSolenoid.Value.kForward) ? 
        DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kReverse);
	}
}
