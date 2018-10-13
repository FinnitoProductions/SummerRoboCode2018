package org.usfirst.frc.team1072.robot.auto.modes;

import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.commands.auton.AutonomousCommand;
import org.usfirst.frc.team1072.robot.commands.auton.FollowPath;
import org.usfirst.frc.team1072.robot.commands.auton.PauseUntilPathBegins;
import org.usfirst.frc.team1072.robot.commands.auton.PauseUntilReachingPosition;
import org.usfirst.frc.team1072.robot.commands.auton.PrebufferPathPoints;
import org.usfirst.frc.team1072.robot.commands.auton.PauseUntilPathBegins.PauseType;
import org.usfirst.frc.team1072.robot.commands.drivetrain.CombinedPositionAnglePID;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;
import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterSwitch extends AutoMode {
	private boolean onLeft;
	private FollowPath fpc1, fpc2, fpc5, fpc6, fpc7, fpc8, fpc9;
    private CombinedPositionAnglePID fpc3, fpc4;
    
	public CenterSwitch (boolean onLeft) {
		this.onLeft = onLeft;
		
		fpc1 = AutonomousCommand.setupPathFollowerArc(onLeft ? AutonomousConstants.CLH_P1_LEFT : "", onLeft ? AutonomousConstants.CLH_P1_RIGHT : "", 
                false, null).zeroPigeonAtStart(false).resetSensors(true);
        fpc2 = AutonomousCommand.setupPathFollowerArc(onLeft? AutonomousConstants.CLH_P2_LEFT_REV : "", onLeft ? AutonomousConstants.CLH_P2_RIGHT_REV : "", true, 
                fpc1).zeroPigeonAtStart(false).resetSensors(false);
        fpc3 = new CombinedPositionAnglePID(2.95, 0);
        fpc4 = new CombinedPositionAnglePID(-3.25, 0);
        fpc5 = AutonomousCommand.setupPathFollowerArc
                (onLeft ? AutonomousConstants.CLH_P5_LEFT : "", onLeft ? AutonomousConstants.CLH_P5_RIGHT : "", false, fpc2)
                .zeroPigeonAtStart(false).resetSensors(true);
	}

	@Override
	public void addCommands() {
		addSequential (getFirstCube());
		addSequential (getSecondCube());
		addSequential (scoreSecondCube());
	}
	
	public CommandGroup getFirstCube() {
		CommandGroup firstCube = new CommandGroup();
	        CommandGroup firstPath = new CommandGroup();
	            firstPath.addSequential(new PrebufferPathPoints(fpc1));
	            CommandGroup preBufferAndPaths = new CommandGroup();
	                preBufferAndPaths.addParallel(fpc1);
	                CommandGroup preBuffer = new CommandGroup();
	                    preBuffer.addSequential(new PrebufferPathPoints(fpc2));
	                    preBuffer.addSequential(new PrebufferPathPoints(fpc5));
	                preBufferAndPaths.addParallel(preBuffer);
	            firstPath.addSequential(preBufferAndPaths);
	        firstCube.addParallel(firstPath);
	        CommandGroup raiseElevatorFirstCube = new CommandGroup();
	            raiseElevatorFirstCube.addSequential(new PauseUntilPathBegins(fpc1, PauseType.END_OF_PATH, 1.9, fpc1.getTotalTime()));
	            raiseElevatorFirstCube.addSequential(new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT_AUTON));
	        firstCube.addParallel(raiseElevatorFirstCube);
	        CommandGroup outtakeFirstCube = new CommandGroup();
	            outtakeFirstCube.addSequential(new PauseUntilPathBegins(fpc1, PauseType.END_OF_PATH, 0.15, fpc1.getTotalTime()));
	            outtakeFirstCube.addSequential(new SetSolenoid(SolenoidDirection.DECOMPRESS));
	            outtakeFirstCube.addSequential(new IntakeOuttakeTimed(0.17, IntakeType.OUTTAKE));
	        firstCube.addParallel(outtakeFirstCube);
	    addSequential(firstCube);
	    return firstCube;
	}
	
	private CommandGroup getSecondCube() {
		CommandGroup getSecondCube = new CommandGroup();
			CommandGroup pathGroupSecondCube = new CommandGroup();
				pathGroupSecondCube.addSequential(fpc2);
				CommandGroup startPath3LowerIntake = new CommandGroup();
					startPath3LowerIntake.addParallel(fpc3);
					startPath3LowerIntake.addParallel(new SetSolenoid(SolenoidDirection.DOWN));
				pathGroupSecondCube.addSequential(startPath3LowerIntake);
			getSecondCube.addParallel(pathGroupSecondCube);
			CommandGroup lowerElevatorSecondCube = new CommandGroup();
				lowerElevatorSecondCube
						.addSequential(new PauseUntilPathBegins(fpc2, PauseType.START_OF_PATH, 0.5, fpc2.getTotalTime()));
				lowerElevatorSecondCube.addSequential(new MoveElevatorMotionMagic(Elevator.INTAKE_HEIGHT));
			getSecondCube.addParallel(lowerElevatorSecondCube);
			CommandGroup intakeSecondCube = new CommandGroup();
				intakeSecondCube.addSequential(new PauseUntilReachingPosition(fpc3, 0.45));
				intakeSecondCube.addSequential(new IntakeOuttakeTimed(0.4, IntakeType.INTAKE));
			getSecondCube.addParallel(intakeSecondCube);
		addSequential(getSecondCube);
		return getSecondCube;
	}

	private CommandGroup scoreSecondCube () {
        CommandGroup scoreSecondCube = new CommandGroup();
            CommandGroup pathGroupOuttakeSecondCube = new CommandGroup();
                pathGroupOuttakeSecondCube.addSequential(fpc4);
                pathGroupOuttakeSecondCube.addSequential(fpc5);
            scoreSecondCube.addParallel(pathGroupOuttakeSecondCube);
            CommandGroup intakeSecondCubeDuringPath = new CommandGroup();
                intakeSecondCubeDuringPath.addSequential(new IntakeOuttakeTimed(0.4, IntakeType.INTAKE));
                intakeSecondCubeDuringPath.addSequential(new SetSolenoid(SolenoidDirection.COMPRESS));
            scoreSecondCube.addParallel(intakeSecondCubeDuringPath);
            CommandGroup outtakeSecondCube = new CommandGroup();
                outtakeSecondCube.addSequential(new PauseUntilPathBegins(fpc5, PauseType.END_OF_PATH, 
                        2, fpc5.getTotalTime()));
                outtakeSecondCube.addSequential(new SetSolenoid(SolenoidDirection.COMPRESS));
                outtakeSecondCube.addSequential(new SetSolenoid(SolenoidDirection.UP));
                outtakeSecondCube.addSequential(new MoveElevatorMotionMagic
                        (Elevator.SWITCH_HEIGHT_AUTON));
                outtakeSecondCube.addSequential(new SetSolenoid(SolenoidDirection.DECOMPRESS));
                outtakeSecondCube.addSequential(new IntakeOuttakeTimed(0.34, IntakeType.OUTTAKE));
            scoreSecondCube.addParallel(outtakeSecondCube);
        addSequential(scoreSecondCube);
        return scoreSecondCube;
	}
	
}
