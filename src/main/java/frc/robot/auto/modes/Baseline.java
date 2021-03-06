package frc.robot.auto.modes;

import frc.robot.RobotMap.AutonomousConstants;
import frc.robot.commands.drivetrain.DriveToPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import harkerrobolib.auto.AutoMode;
import harkerrobolib.commands.ThrowExceptionCommand;

public class Baseline extends AutoMode {
	public Baseline(Location loc) {
		super(loc, loc);
	}

	@Override
	public Command getLeftCommands(Location endLoc) {
		return new DriveToPosition (AutonomousConstants.BASELINE_DISTANCE);
	}

	@Override
	public Command getRightCommands(Location endLoc) {
		return new DriveToPosition (AutonomousConstants.BASELINE_DISTANCE);
	}
}
