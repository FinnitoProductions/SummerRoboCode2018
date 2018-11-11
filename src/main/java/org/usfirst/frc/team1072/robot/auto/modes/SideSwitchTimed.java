//package org.usfirst.frc.team1072.robot.auto.modes;
//
//import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveWithVelocityTimed;
//import org.usfirst.frc.team1072.robot.commands.drivetrain.TurnToAngleTimed;
//import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
//import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed;
//import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
//import org.usfirst.frc.team1072.robot.subsystems.Elevator;
//import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.TurnDirection;
//import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;
//import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidDirection;
//
//import edu.wpi.first.wpilibj.command.Command;
//import harkerrobolib.auto.AutoMode;
//import harkerrobolib.auto.ParallelCommandGroup;
//import harkerrobolib.auto.SequentialCommandGroup;
//
//public class SideSwitchTimed extends AutoMode {
//
//	public static final double DRIVE_FORWARD_TIME_ONE = 1.0;
//	public static final double TURN_TIME_ONE = 1.0;
//	public static final double DRIVE_FORWARD_TIME_TWO = 0.5;
//
//	public SideSwitchTimed(StartLocation loc) {
//		super(loc, new SequentialCommandGroup(new DriveWithVelocityTimed(DRIVE_FORWARD_TIME_ONE),
//				new TurnToAngleTimed(TURN_TIME_ONE, TurnDirection.RIGHT),
//				new ParallelCommandGroup(new DriveWithVelocityTimed(DRIVE_FORWARD_TIME_TWO), new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT_AUTON)),
//				new ParallelCommandGroup(new SetSolenoid(SolenoidDirection.DECOMPRESS), new IntakeOuttakeTimed(0.5, IntakeType.OUTTAKE))),
//				AutoMode.Companion.getCenterCommandDefault(),
//				new SequentialCommandGroup(new DriveWithVelocityTimed(DRIVE_FORWARD_TIME_ONE),
//						new TurnToAngleTimed(TURN_TIME_ONE, TurnDirection.LEFT),
//						new ParallelCommandGroup(new DriveWithVelocityTimed(DRIVE_FORWARD_TIME_TWO), new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT_AUTON)),
//						new ParallelCommandGroup(new SetSolenoid(SolenoidDirection.DECOMPRESS), new IntakeOuttakeTimed(0.5, IntakeType.OUTTAKE))));
//		// TODO Auto-generated constructor stub
//	}
//
//}
