package org.usfirst.frc.team1072.robot.auto.modes;

import java.security.InvalidParameterException;

import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.auto.paths.LeftToLeftScaleSide;
import org.usfirst.frc.team1072.robot.commands.auton.Delay;
import org.usfirst.frc.team1072.robot.commands.auton.FollowPathRio;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPosition;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;
import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidDirection;

import edu.wpi.first.wpilibj.command.Command;
import harkerrobolib.auto.AutoMode;
import harkerrobolib.auto.CommandGroupWrapper;

public class CompatibleScale extends AutoMode {

	public CompatibleScale(StartLocation loc) {
		super(loc);
	}

	@Override
	public Command getCenterCommands() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Command getLeftCommands() {
        return new CommandGroupWrapper().sequential (new FollowPathRio (new LeftToLeftScaleSide()))
        .sequential (new MoveElevatorMotionMagic(Elevator.SCALE_HIGH_HEIGHT))
        .sequential (new DriveToPosition(2.75))
        .sequential(new SetSolenoid (SolenoidDirection.DECOMPRESS))
        .sequential (new IntakeOuttakeTimed(AutonomousConstants.SCALE_OUTTAKE_TIME, IntakeType.OUTTAKE))
        .sequential(new Delay(1))
        .sequential (new DriveToPosition(-1.5));
	}

	@Override
	public Command getRightCommands() {
		// TODO Auto-generated method stub
		return null;
	}


}
