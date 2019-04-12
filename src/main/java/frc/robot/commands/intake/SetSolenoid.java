package frc.robot.commands.intake;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics.SolenoidDirection;

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
		//requires(Intake.pn);
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
