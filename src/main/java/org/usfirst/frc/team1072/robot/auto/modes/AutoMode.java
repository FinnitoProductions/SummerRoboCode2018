package org.usfirst.frc.team1072.robot.auto.modes;

import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class AutoMode extends CommandGroup {
	public AutoMode() {
		addCommands();
	}
	public abstract void addCommands();
}
