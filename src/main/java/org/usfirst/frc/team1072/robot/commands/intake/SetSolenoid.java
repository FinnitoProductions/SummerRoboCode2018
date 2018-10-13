package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidDirection;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Represents a command to set the solenoid to a given state.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class SetSolenoid extends InstantCommand {
	/**
	 * The intended state for the solenoid.
	 */
	private SolenoidDirection solenoidState;

	/**
	 * Sets up the solenoid command, requiring the intake.
	 * 
	 * @param key   the key in the solenoid map containing the solenoid to be
	 *              modified
	 * @param state the state of the solenoid (forward, off, or reverse)
	 */
	public SetSolenoid(SolenoidDirection state) {
		requires(Intake.pn);
		solenoidState = state;
	}

	/**
	 * Sets a given solenoid to a given state.
	 */
	public void initialize() {
		if (solenoidState == SolenoidDirection.DOWN
				&& Robot.el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) <= Elevator.INTAKE_HEIGHT)
				new MoveElevatorMotionMagic(Elevator.INTAKE_HEIGHT).start();
		Intake.pn.setSolenoid(solenoidState);
	}
}
