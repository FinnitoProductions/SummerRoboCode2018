package org.usfirst.frc.team1072.robot.auto.modes;

import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import harkerrobolib.auto.AutoMode;
import harkerrobolib.commands.ThrowException;

public class Baseline extends AutoMode {
	public Baseline(StartLocation loc) {
		super(loc, new DriveToPosition (AutonomousConstants.BASELINE_DISTANCE), 
				new ThrowException("Center Auton Not Defined"), 
				new DriveToPosition (AutonomousConstants.BASELINE_DISTANCE));
	}
}
