package frc.robot.auto.modes;

import frc.robot.RobotMap.AutonomousConstants;
import frc.robot.auto.paths.LeftToLeftScaleSide;
import frc.robot.commands.auton.Delay;
import frc.robot.commands.auton.FollowPathRio;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.intake.IntakeOuttakeTimed;
import frc.robot.commands.intake.SetSolenoid;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.IntakeType;
import frc.robot.subsystems.Pneumatics.SolenoidDirection;

import edu.wpi.first.wpilibj.command.Command;
import harkerrobolib.auto.AutoMode;
import harkerrobolib.auto.SequentialCommandGroup;

public class CompatibleScale extends AutoMode {

	public CompatibleScale(Location loc) {
		super(loc, loc);
	}

	@Override
	public Command getLeftCommands(Location endLoc) {
		return new SequentialCommandGroup (new FollowPathRio (new LeftToLeftScaleSide()),
		        new MoveElevatorMotionMagic(Elevator.SCALE_HIGH_HEIGHT),
		        new DriveToPosition(2.75),
		        new SetSolenoid (SolenoidDirection.DECOMPRESS),
		        new IntakeOuttakeTimed(AutonomousConstants.SCALE_OUTTAKE_TIME, IntakeType.OUTTAKE),
		        new Delay(1),
		        new DriveToPosition(-1.5));
	}
}
