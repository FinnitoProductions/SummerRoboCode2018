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
import org.usfirst.frc.team1072.robot.commands.drivetrain.TurnToAngle;
import org.usfirst.frc.team1072.robot.commands.elevator.InitializeElevator;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.InitializeIntake;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeTimed;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
        System.out.println("ALL SUBSYSTEMS INITIALIZED " + Robot.getCurrentTimeMs());
        /*addSequential(setupPathFollower("/home/summer2018/paths/test_5ft/test_5ft_left_detailed.csv", 
                "/home/summer2018/paths/test_5ft/test_5ft_right_detailed.csv"));*/
        /*addSequential(setupPathFollower("/home/summer2018/paths/curved_path/curved_path_left_detailed.csv", 
                "/home/summer2018/paths/curved_path/curved_path_right_detailed.csv"));*/
        initSubsystems();

        /*addSequential(new CombinedPositionAnglePID(0, 
                90));*/
        //addSequential(new TurnRobotToAngleCommand(90));
        switchAuton(true);

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
        FollowPath fpc1, fpc2, fpc3, fpc4, fpc5, fpc6, fpc7, fpc8, fpc9;
        //DriveToPositionCommand fpc3;
        fpc1 = setupPathFollowerArc(AutonomousConstants.CLH_P1_LEFT, AutonomousConstants.CLH_P1_RIGHT, 
                false, null).zeroPigeonAtStart(false).resetSensors(true);
        fpc2 = setupPathFollowerArc(AutonomousConstants.CLH_P2_LEFT_REV, AutonomousConstants.CLH_P2_RIGHT_REV, true, 
                fpc1).zeroPigeonAtStart(false).resetSensors(false);
        //fpc3 = setupPathFollowerArc(AutonomousPaths.CLH_P3_LEFT, AutonomousPaths.CLH_P3_RIGHT, false);
        //fpc4 = setupPathFollowerArc(AutonomousPaths.CLH_P4_LEFT_REV, AutonomousPaths.CLH_P4_RIGHT_REV, true);
        fpc5 = setupPathFollowerArc
                (AutonomousConstants.CLH_P5_LEFT, AutonomousConstants.CLH_P5_RIGHT, false, fpc2)
                .zeroPigeonAtStart(false).resetSensors(true);
        //fpc6 = setupPathFollowerArc(AutonomousPaths.CLH_P6_LEFT_REV, AutonomousPaths.CLH_P6_RIGHT_REV, true);
        //fpc7 = setupPathFollowerArc(AutonomousPaths.CLH_P7_LEFT, AutonomousPaths.CLH_P7_RIGHT, false);
        //fpc8 = setupPathFollowerArc(AutonomousPaths.CLH_P8_LEFT_REV, AutonomousPaths.CLH_P8_RIGHT_REV, true);
        //fpc9 = setupPathFollowerArc(AutonomousPaths.CLH_P9_LEFT, AutonomousPaths.CLH_P9_RIGHT, false);
       

        
        //addSequential(new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
        //addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        //addSequential(new DriveToPositionCommand(3));
        addSequential(new PrebufferPathPoints(fpc1));
        CommandGroup path1 = new CommandGroup();
            path1.addParallel(fpc1);
            path1.addParallel(new PrebufferPathPoints(fpc2));
            path1.addParallel(new PrebufferPathPoints(fpc5));
        addSequential(path1);

        addSequential(fpc2);
        addSequential(new CombinedPositionAnglePID(4, 0));
        addSequential(new CombinedPositionAnglePID(-4, 0));
        
        addSequential(fpc5);
        /*addSequential(new PrebufferPathPointsCommand(fpc2));
        CommandGroup path2 = new CommandGroup();
        path2.addParallel(fpc2);
        path2.addParallel(new PrebufferPathPointsCommand(fpc3));
        addSequential(path2);*/
        //addSequential(new PrebufferPathPointsCommand(fpc2));
        
        /*CommandGroup path3 = new CommandGroup();
        path3.addParallel(fpc3);
        path3.addParallel(new PrebufferPathPointsCommand(fpc4));
        addSequential(path3);
        CommandGroup path4 = new CommandGroup();
        path4.addParallel(fpc4);
        path4.addParallel(new PrebufferPathPointsCommand(fpc5));
        addSequential(path4);
        addSequential(fpc5);*/
        
        
    
        /*CommandGroup firstCube = new CommandGroup();
            CommandGroup firstPath = new CommandGroup();
                firstPath.addSequential(new PrebufferPathPointsCommand(fpc1));
                CommandGroup preBufferAndPaths = new CommandGroup();
                    //preBufferAndPaths.addParallel(fpc1);
                    CommandGroup preBuffer = new CommandGroup();
                        preBuffer.addSequential(new PrebufferPathPointsCommand(fpc2));
                    preBufferAndPaths.addParallel(preBuffer);
                firstPath.addSequential(preBufferAndPaths);
            firstCube.addParallel(firstPath);
            CommandGroup raiseElevatorFirstCube = new CommandGroup();
                raiseElevatorFirstCube.addSequential(new PauseUntilPathBeginsCommand(fpc1, PauseType.END_OF_PATH, 1.1, fpc1.getTotalTime()));
                raiseElevatorFirstCube.addSequential(new MoveElevatorMotionMagicCommand(1000));//ElevatorConstants.SWITCH_HEIGHT_AUTON));
            firstCube.addParallel(raiseElevatorFirstCube);
            CommandGroup outtakeFirstCube = new CommandGroup();
                outtakeFirstCube.addSequential(new PauseUntilPathBeginsCommand(fpc1, PauseType.END_OF_PATH, 0.15, fpc1.getTotalTime()));
                outtakeFirstCube.addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeFirstCube.addSequential(new IntakeOuttakeTimedCommand(0.17, RobotMap.IntakeConstants.OUTTAKE_BOOL));
            firstCube.addParallel(outtakeFirstCube);
        addSequential(firstCube);
        
            
 /*new DriveToPositionCommand(
                new Position(PositionUnit.FEET, 3.85, 
                DrivetrainConstants.WHEELDIAMETER).getEncoderUnits(), 50);*/
        /*CommandGroup getSecondCube = new CommandGroup();
            CommandGroup pathGroupSecondCube = new CommandGroup();
                //pathGroupSecondCube.addSequential(fpc2);
                //pathGroupSecondCube.addSequential(fpc3);
            getSecondCube.addParallel(pathGroupSecondCube);
            // this command group is not called
            CommandGroup lowerElevatorSecondCube = new CommandGroup();
                lowerElevatorSecondCube.addSequential(
                        new PauseUntilPathBeginsCommand(fpc2, PauseType.START_OF_PATH, 0.5, fpc2.getTotalTime()));
                lowerElevatorSecondCube.addSequential(new MoveElevatorMotionMagicCommand
                        (RobotMap.ElevatorConstants.INTAKE_HEIGHT));
                lowerElevatorSecondCube.addSequential(new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
            getSecondCube.addParallel(lowerElevatorSecondCube);
            CommandGroup intakeSecondCube = new CommandGroup();
                intakeSecondCube.addSequential(
                        new PauseUntilPathBeginsCommand(fpc3, PauseType.END_OF_PATH, 0.5, fpc3.getTotalTime()));
                intakeSecondCube.addSequential(new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY,
                        IntakeConstants.DOWN));
                intakeSecondCube.addSequential(new IntakeOuttakeTimedCommand(0.6, IntakeConstants.INTAKE_BOOL));
                intakeSecondCube.addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.COMPRESS));
           getSecondCube.addParallel(intakeSecondCube);
         //addSequential(getSecondCube);
        
         

        CommandGroup scoreSecondCube = new CommandGroup();
            CommandGroup pathGroupOuttakeSecondCube = new CommandGroup();
                pathGroupOuttakeSecondCube.addSequential(fpc4);
                pathGroupOuttakeSecondCube.addSequential(fpc5);
            scoreSecondCube.addParallel(pathGroupOuttakeSecondCube);
            CommandGroup intakeSecondCubeDuringPath = new CommandGroup();
                intakeSecondCubeDuringPath.addSequential(new IntakeOuttakeTimedCommand(fpc4.getTotalTime(), IntakeConstants.INTAKE_BOOL));
            scoreSecondCube.addParallel(intakeSecondCubeDuringPath);
            CommandGroup elevatorRaiseSecondCube = new CommandGroup();
                elevatorRaiseSecondCube.addSequential(new PauseUntilPathBeginsCommand(fpc5, PauseType.END_OF_PATH, 
                        1.1, fpc5.getTotalTime()));
                elevatorRaiseSecondCube.addSequential(new MoveElevatorMotionMagicCommand(ElevatorConstants.SWITCH_HEIGHT_AUTON));
            scoreSecondCube.addParallel(elevatorRaiseSecondCube);
            CommandGroup outtakeSecondCube = new CommandGroup();
                outtakeSecondCube.addSequential(
                        new PauseUntilPathBeginsCommand(fpc5, PauseType.END_OF_PATH, 0.8, fpc5.getTotalTime()));
                outtakeSecondCube.addSequential(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeSecondCube.addSequential(new IntakeOuttakeTimedCommand(0.15, RobotMap.IntakeConstants.OUTTAKE_BOOL));
            scoreSecondCube.addParallel(outtakeSecondCube);
        //addSequential(scoreSecondCube);


        
        CommandGroup getThirdCube = new CommandGroup();
            CommandGroup pathGroupIntakeThirdCube = new CommandGroup();
                pathGroupIntakeThirdCube.addSequential(fpc6);
                pathGroupIntakeThirdCube.addSequential(fpc7);
            getThirdCube.addParallel(pathGroupIntakeThirdCube);
            CommandGroup lowerElevatorThirdCube = new CommandGroup();
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
                    IntakeConstants.COMPRESS));
        //addSequential(getThirdCube);
        
        CommandGroup scoreThirdCube = new CommandGroup();
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
        DriveToPosition dtp1, dtp2;
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
        addSequential(getSecondCube);
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
