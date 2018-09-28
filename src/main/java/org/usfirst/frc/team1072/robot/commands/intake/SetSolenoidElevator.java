package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team1072.robot.RobotMap;

/**
 * Represents a command to set the solenoid to a given state.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class SetSolenoidElevator extends CommandGroup {
	/**
	 * Sets up the solenoid command, requiring the intake.
	 * 
	 * @param key   the key in the solenoid map containing the solenoid to be
	 *              modified
	 * @param state the state of the solenoid (forward, off, or reverse)
	 */
	public SetSolenoidElevator (String key, DoubleSolenoid.Value state) {
		requires(Intake.pn);
		
		if (Intake.pn.getSolenoid(IntakeConstants.UPDOWN_KEY).get() == IntakeConstants.DOWN && 
				key.equals(IntakeConstants.COMPRESSDECOMPRESS_KEY)&&
				state == IntakeConstants.COMPRESS) {
			requires(Robot.el);
			addSequential (new MoveElevatorMotionMagic(ElevatorConstants.INTAKE_HEIGHT));
		}
		
		addSequential (new SetSolenoid(key, state));
	}
}
