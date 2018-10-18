package org.usfirst.frc.team1072.robot.auto.modes;

import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import harkerrobolib.auto.AutoMode;

public class Baseline extends AutoMode {
	public Baseline(StartLocation loc) {
		super(loc);
	}

	@Override
	public Command getCenterCommands() {
		return new CommandGroup();
	}

	@Override
	public Command getLeftCommands() {
		return new DriveToPosition (AutonomousConstants.BASELINE_DISTANCE);
	}

	@Override
	public Command getRightCommands() {
		return getLeftCommands();
	}
}
