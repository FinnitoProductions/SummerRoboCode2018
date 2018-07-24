package org.usfirst.frc.team1072.robot.commands.auton;

import java.io.File;
import java.io.FileNotFoundException;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.auton.PauseUntilPathBegins.PauseType;
import org.usfirst.frc.team1072.robot.commands.drivetrain.CombinedPositionAnglePID;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveToPosition;
import org.usfirst.frc.team1072.robot.commands.drivetrain.InitializeDrivetrain;
import org.usfirst.frc.team1072.robot.commands.drivetrain.PositionCommand;
import org.usfirst.frc.team1072.robot.commands.drivetrain.TurnToAngle;
import org.usfirst.frc.team1072.robot.commands.elevator.InitializeElevator;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.InitializeIntake;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed.IntakeType;
import org.usfirst.frc.team1072.robot.commands.util.PrintValueCommand;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * A set of commands to be called during the autonomous period.
 * @author Finn Frankis
 * @version 6/20/18
 */
public class AutonomousCommand extends CommandGroup
{
    private int numPoints = 0;
    
    
    
    //private final String CENTER_LEFT_HEAD_ON_ONE_CUBE_SLOW_LEFT_PART2 = "/home/summer2018/paths/center_left_headonTest(switch_1)/center_left_headonTest(switch_1)_left_detailed.csv";
    
    /**
     * Constructs a new command
     * @param subsystems the list of subsystems
     */
    public AutonomousCommand(Subsystem[] subsystems)
    {
        for (Subsystem s : subsystems)
            requires(s);

        initSubsystems();
        //testThirdCube();
        
        switchAuton(true);
    }

    private void testThirdCube()
    {
        PositionCommand fpc6 = new CombinedPositionAnglePID(-3.4, 0).setAllowableError(650), 
                fpc7 = new CombinedPositionAnglePID(4.0, -44).setAllowableError(650),
                fpc8 = new DriveToPosition(-0.5).setAllowableError(650), 
                fpc9 = new CombinedPositionAnglePID(3.2, 40).setAllowableError(1000);
        TurnToAngle turn6 = new TurnToAngle(-52.25, 0.8), turn8 = new TurnToAngle(0, 1.5);
        CommandGroup thirdCube = new CommandGroup();
            CommandGroup intakeCube = new CommandGroup();
                intakeCube.addSequential(new InstantCommand() {
                    public void initialize()
                    {
                        System.out.println("REACHED COMMAND BODY");
                    }
                });
                
            thirdCube.addParallel(intakeCube);
            CommandGroup thirdCubePaths = new CommandGroup();
                thirdCubePaths.addSequential(fpc6);
                thirdCubePaths.addSequential(turn6);
                thirdCubePaths.addSequential(fpc7);
                thirdCubePaths.addSequential(fpc8);
                thirdCubePaths.addSequential(turn8);
                thirdCubePaths.addSequential(fpc9);
            thirdCube.addParallel(thirdCubePaths);
            CommandGroup lowerIntakeThirdCube = new CommandGroup();
                lowerIntakeThirdCube.addSequential(new PauseUntilReachingPosition(fpc7, 0.01));
                lowerIntakeThirdCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
                lowerIntakeThirdCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
                lowerIntakeThirdCube.addSequential(new IntakeOuttakeTimed(1.1, IntakeType.INTAKE));
                lowerIntakeThirdCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
                lowerIntakeThirdCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
                lowerIntakeThirdCube.addSequential(new PauseUntilReachingPosition(fpc9, 0.35));
                lowerIntakeThirdCube.addSequential(new MoveElevatorMotionMagic
                        (ElevatorConstants.SWITCH_HEIGHT_THIRD_CUBE));
                lowerIntakeThirdCube.addSequential(new IntakeOuttakeTimed(1.1, IntakeType.OUTTAKE));
            thirdCube.addParallel(lowerIntakeThirdCube);
        addSequential(thirdCube);
    }
    
    /**
     * Initializes subsystems in parallel.
     */
    private void initSubsystems()
    {
        CommandGroup initSubsystems = new CommandGroup();
            initSubsystems.addParallel(new InitializeDrivetrain());
            initSubsystems.addParallel(new InitializeElevator());
            initSubsystems.addParallel(new InitializeIntake());
        addSequential(initSubsystems);
    }
    /**
     * The command to be performed for a one-cube switch autonomous.
     * @param onLeft true if the switch is on the left; false otherwise
     */
    private void switchAuton (boolean onLeft)
    {
        FollowPath fpc1, fpc2, fpc5;
        CombinedPositionAnglePID fpc3, fpc4, fpc6, fpc7, fpc8, fpc9;
        TurnToAngle turn6, turn7;
        //DriveToPositionCommand fpc3;
        fpc1 = setupPathFollowerArc(AutonomousConstants.CLH_P1_LEFT, AutonomousConstants.CLH_P1_RIGHT, 
                false, null).zeroPigeonAtStart(false).resetSensors(true);
        fpc2 = setupPathFollowerArc(AutonomousConstants.CLH_P2_LEFT_REV, AutonomousConstants.CLH_P2_RIGHT_REV, true, 
                fpc1).zeroPigeonAtStart(false).resetSensors(false);
        fpc3 = new CombinedPositionAnglePID(3.1, 0);
        fpc4 = new CombinedPositionAnglePID(-3, 0);
        fpc5 = setupPathFollowerArc
                (AutonomousConstants.CLH_P5_LEFT, AutonomousConstants.CLH_P5_RIGHT, false, fpc2)
                .zeroPigeonAtStart(false).resetSensors(true);
      
        
        addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
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
                raiseElevatorFirstCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.SWITCH_HEIGHT_AUTON));
            firstCube.addParallel(raiseElevatorFirstCube);
            CommandGroup outtakeFirstCube = new CommandGroup();
                outtakeFirstCube.addSequential(new PauseUntilPathBegins(fpc1, PauseType.END_OF_PATH, 0.15, fpc1.getTotalTime()));
                outtakeFirstCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeFirstCube.addSequential(new IntakeOuttakeTimed(0.17, IntakeType.OUTTAKE));
            firstCube.addParallel(outtakeFirstCube);
        addSequential(firstCube);
        
        CommandGroup getSecondCube = new CommandGroup();
            CommandGroup pathGroupSecondCube = new CommandGroup();
                pathGroupSecondCube.addSequential(fpc2);
                CommandGroup startPath3LowerIntake = new CommandGroup();
                    startPath3LowerIntake.addParallel(fpc3);
                    startPath3LowerIntake.addParallel(new SetSolenoid(IntakeConstants.UPDOWN_KEY,
                            IntakeConstants.DOWN));
                pathGroupSecondCube.addSequential(startPath3LowerIntake);
            getSecondCube.addParallel(pathGroupSecondCube);
            // this command group is not called
            CommandGroup lowerElevatorSecondCube = new CommandGroup();
                lowerElevatorSecondCube.addSequential(
                        new PauseUntilPathBegins(fpc2, PauseType.START_OF_PATH, 0.5, fpc2.getTotalTime()));          
                lowerElevatorSecondCube.addSequential(new MoveElevatorMotionMagic
                        (ElevatorConstants.INTAKE_HEIGHT));
            getSecondCube.addParallel(lowerElevatorSecondCube);
            CommandGroup intakeSecondCube = new CommandGroup();
                intakeSecondCube.addSequential(
                        new PauseUntilReachingPosition(fpc3, 0.45));
                intakeSecondCube.addSequential(new IntakeOuttakeTimed(0.4, IntakeType.INTAKE));
           getSecondCube.addParallel(intakeSecondCube);
         addSequential(getSecondCube);

        CommandGroup scoreSecondCube = new CommandGroup();
            CommandGroup pathGroupOuttakeSecondCube = new CommandGroup();
                pathGroupOuttakeSecondCube.addSequential(fpc4);
                pathGroupOuttakeSecondCube.addSequential(fpc5);
            scoreSecondCube.addParallel(pathGroupOuttakeSecondCube);
            CommandGroup intakeSecondCubeDuringPath = new CommandGroup();
                intakeSecondCubeDuringPath.addSequential(new IntakeOuttakeTimed(0.5, IntakeType.INTAKE));
                intakeSecondCubeDuringPath.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
            scoreSecondCube.addParallel(intakeSecondCubeDuringPath);
            CommandGroup elevatorRaiseSecondCube = new CommandGroup();
                elevatorRaiseSecondCube.addSequential(new PauseUntilPathBegins(fpc5, PauseType.END_OF_PATH, 
                        1.9, fpc5.getTotalTime()));
                elevatorRaiseSecondCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY,
                            IntakeConstants.UP));
                elevatorRaiseSecondCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.SWITCH_HEIGHT_AUTON));
            scoreSecondCube.addParallel(elevatorRaiseSecondCube);
        addSequential(scoreSecondCube);


        
        CommandGroup getThirdCube = new CommandGroup();
                CommandGroup outtakeSecondCube = new CommandGroup();

                outtakeSecondCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeSecondCube.addSequential(new IntakeOuttakeTimed(0.15, IntakeType.OUTTAKE));
            getThirdCube.addParallel(outtakeSecondCube);
                //pathGroupIntakeThirdCube.addSequential(fpc7);
            //getThirdCube.addParallel(pathGroupIntakeThirdCube);
            /*CommandGroup lowerElevatorThirdCube = new CommandGroup();
                lowerElevatorThirdCube.addSequential(
                        new PauseUntilPathBeginsCommand(fpc6, PauseType.END_OF_PATH, 0.7, fpc7.getTotalTime()));
                lowerElevatorThirdCube.addSequential(new MoveElevatorMotionMagicCommand(ElevatorConstants.INTAKE_HEIGHT));
            getThirdCube.addParallel(lowerElevatorThirdCube);
            CommandGroup intakeThirdCube = new CommandGroup();
                intakeThirdCube.addParallel(
                        new PauseUntilPathBeginsCommand(fpc7, PauseType.END_OF_PATH, 0.7, fpc7.getTotalTime()));
                CommandGroup thirdCubePneumatics = new CommandGroup();
                thirdCubePneumatics.addParallel(new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY,
                        IntakeConstants.DOWN));
                thirdCubePneumatics.addParallel(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
            intakeThirdCube.addSequential(thirdCubePneumatics);
            getThirdCube.addSequential(new IntakeOuttakeTimedCommand(0.6, IntakeConstants.INTAKE_BOOL));
            getThirdCube.addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                    IntakeConstants.COMPRESS));*/
        addSequential(getThirdCube);
        
        /*CommandGroup scoreThirdCube = new CommandGroup();
            CommandGroup pathGroupScoreThirdCube = new CommandGroup();
                pathGroupScoreThirdCube.addSequential(fpc8);
                pathGroupScoreThirdCube.addSequential(fpc9);
            scoreThirdCube.addParallel(pathGroupScoreThirdCube);
            CommandGroup raiseElevatorThirdCube = new CommandGroup();
                raiseElevatorThirdCube.addSequential(new PauseUntilPathBeginsCommand(fpc9, PauseType.END_OF_PATH, 0.9, fpc9.getTotalTime()));
                raiseElevatorThirdCube.addSequential(new MoveElevatorMotionMagicCommand(ElevatorConstants.SWITCH_HEIGHT_AUTON));
            scoreThirdCube.addParallel(raiseElevatorThirdCube);
                CommandGroup outtakeThirdCube = new CommandGroup();
                outtakeThirdCube.addSequential(new PauseUntilPathBeginsCommand(fpc1, PauseType.END_OF_PATH, 0.15, fpc1.getTotalTime()));
                outtakeThirdCube.addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeThirdCube.addSequential(new IntakeOuttakeTimedCommand(0.15, RobotMap.IntakeConstants.OUTTAKE_BOOL));
        // addSequential(scoreThirdCube);
      */
       
                           
        
        
        
        
    }
    
    private void scaleAuton (boolean onLeft)
    {
        /*DriveToPosition dtp1, dtp2;
        TurnToAngle tta1;
        CommandGroup scoreFirstCube = new CommandGroup();
            scoreFirstCube.addParallel(dtp1 = new DriveToPosition(AutonomousConstants.DISTANCE_TO_SCALE));
            CommandGroup raiseElevatorFirstCube = new CommandGroup();
                raiseElevatorFirstCube.addSequential(new PauseUntilReachingPosition(dtp1, 0.4));
                raiseElevatorFirstCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
                raiseElevatorFirstCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.SCALE_HIGH_HEIGHT));
            scoreFirstCube.addParallel(raiseElevatorFirstCube);
            CommandGroup outtakeFirstCube = new CommandGroup();
                outtakeFirstCube.addSequential(new PauseUntilReachingPosition(dtp1, 0.9));
                outtakeFirstCube.addSequential(new IntakeOuttakeTimed(0.6, IntakeConstants.OUTTAKE_BOOL));
            scoreFirstCube.addParallel(outtakeFirstCube);
        addSequential(scoreFirstCube);
        
        CommandGroup getSecondCube = new CommandGroup();
            CommandGroup path2 = new CommandGroup();
                path2.addSequential(tta1 = new TurnToAngle(AutonomousConstants.ANGLE_FROM_SCALE_TO_CUBES));
                path2.addSequential(dtp2 = new DriveToPosition(AutonomousConstants.DISTANCE_TO_SCALE));
            getSecondCube.addParallel(path2);
            CommandGroup lowerElevatorSecondCube = new CommandGroup();
                lowerElevatorSecondCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.INTAKE_HEIGHT));
            getSecondCube.addParallel(lowerElevatorSecondCube);
            CommandGroup intakeSecondCube = new CommandGroup();
                intakeSecondCube.addSequential(new PauseUntilReachingPosition(dtp2, 0.7));
                intakeSecondCube.addSequential(new IntakeOuttakeTimed(0.6, IntakeConstants.INTAKE_BOOL));
            getSecondCube.addParallel(intakeSecondCube);
        addSequential(getSecondCube);*/
    }
    /**
     * Reads in a trajectory (from Jaci's pathfinder) given the filename as a CSV.
     * @param filename the file name
     * @return the Trajectory
     * @throws FileNotFoundException
     */
    public static Trajectory readTrajectory(String filename) throws FileNotFoundException
    {
        File f = new File(filename);
        if (f.exists() && f.isFile() && filename.endsWith(".csv"))
        {
            try
            {
                return Pathfinder.readFromCSV(f);
            }
            catch (Exception e)
            {
                throw new FileNotFoundException("Pathfinder failed to read trajectory: " + filename);
            }
        }
        else
        {
            throw new FileNotFoundException("Trajectory: " + filename + ", does not exist or is not a csv file");
        }
    }

//    @SuppressWarnings("unused")
//    private FollowPath setupPathFollower(String leftFileName, String rightFileName, boolean reverse)
//    {
//        FollowPath fpc = new FollowPath();
//
//        Trajectory leftPath1 = null;
//        Trajectory rightPath1 = null;
//
//        try
//        {
//            leftPath1 = readTrajectory(leftFileName);
//            rightPath1 = readTrajectory(rightFileName);
//        }
//        catch (Exception e)
//        {
//            e.printStackTrace();
//            System.out.println("FILE. NOT. FOUND.");
//        }
//        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon(), reverse);
//        fpc.addProfile(rightPath1, Robot.dt.getRightTalon(), reverse);
//        numPoints = (leftPath1.segments.length + rightPath1.segments.length)/2;
//        fpc.setTotalTime(numPoints * RobotMap.TIME_PER_TRAJECTORY_POINT_MS);
//        return fpc;
//    }
    
    /**
     * Sets up a motion profile arc path.
     * @param leftFileName the file name of the left path 
     * @param rightFileName the file name of the right path
     * @param reverse if true: perform the trajectory in reverse order; if false: perform it normally
     * @return the new follow path command
     */
    private FollowPath setupPathFollowerArc(String leftFileName, String rightFileName, boolean reverse, FollowPath prevPath)
    {
        double endAngleLeft;
        double endAngleRight;
        if (prevPath == null)
        {
            endAngleLeft = 0;
            endAngleRight = 0;
        }
        else
        {
            endAngleLeft = prevPath.getControllerTrajectory
                    (Robot.dt.getLeftTalon()).segments[prevPath.getControllerTrajectory
                                                       (Robot.dt.getLeftTalon()).segments.length-1].heading;
            endAngleRight = prevPath.getControllerTrajectory
                    (Robot.dt.getRightTalon()).segments[prevPath.getControllerTrajectory
                                                       (Robot.dt.getLeftTalon()).segments.length-1].heading;                                                      
        }
        
        FollowPathArc fpc = new FollowPathArc();

        Trajectory leftPath1 = null;
        Trajectory rightPath1 = null;

        try
        {
            leftPath1 = readTrajectory(leftFileName);
            rightPath1 = readTrajectory(rightFileName);
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("FILE. NOT. FOUND.");
        }
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon(), reverse, endAngleLeft);
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon(), reverse, endAngleRight);

        numPoints = (leftPath1.segments.length + rightPath1.segments.length)/2;
        fpc.setTotalTime(numPoints * RobotMap.TIME_PER_TRAJECTORY_POINT_MS);
        return fpc;
    }
}
