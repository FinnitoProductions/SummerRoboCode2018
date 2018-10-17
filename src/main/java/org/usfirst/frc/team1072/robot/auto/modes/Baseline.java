package org.usfirst.frc.team1072.robot.auto.modes;

import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPosition;

import harkerrobolib.auto.AutoMode;

public class Baseline extends AutoMode {
	@Override
	public void addCommands() {
		addSequential (new DriveToPosition(AutonomousConstants.BASELINE_DISTANCE));
	}
	
}
