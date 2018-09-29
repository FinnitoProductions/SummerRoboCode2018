package org.usfirst.frc.team1072.robot.commands.auton;

import java.io.File;
import java.io.FileNotFoundException;

import org.omg.CORBA.TRANSACTION_UNAVAILABLE;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.AutonomousConstants;
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
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * A set of commands to be called during the autonomous period.
 * @author Finn Frankis
 * @version 6/20/18
 */
public class AutonomousCommand extends CommandGroup
{    
    public static final boolean ON_LEFT = true;
    public static final boolean ON_RIGHT = false;
    public enum AutonType {
        BASELINE, CENTER_SWITCH, ONE_CUBE_CENTER, ONE_CUBE_SIDE, LEFT_SWITCH, RIGHT_SWITCH;
    }
    
    public enum RobotLocation {
        LEFT, CENTER, RIGHT;
    }

    /**
     * Constructs a new command
     * @param subsystems the list of subsystems
     */
    public AutonomousCommand(RobotLocation location, Subsystem[] subsystems, String fieldData)
    {
        for (Subsystem s : subsystems)
            requires(s);

        initSubsystems();

        if (location == RobotLocation.LEFT) {
            if (fieldData.equals("LLL"))
                sideScale(ON_LEFT);
            else if (fieldData.equals("RLR")) 
                sideScale(ON_LEFT);
            else if (fieldData.equals("RRR"))
                baseline();
            else if (fieldData.equals("LRL"))
                oneCubeSide(ON_LEFT);
        }
        else if (location == RobotLocation.CENTER) {
            if (fieldData.equals("LLL"))
                oneCubeCenter(ON_LEFT);
            else if (fieldData.equals("RLR")) 
                oneCubeCenter(ON_RIGHT);
            else if (fieldData.equals("RRR"))
                oneCubeCenter(ON_RIGHT);
            else if (fieldData.equals("LRL"))
                oneCubeSide(ON_LEFT);
        }
        else if (location == RobotLocation.RIGHT) {
            if (fieldData.equals("LLL"))
                baseline();
            else if (fieldData.equals("RLR")) 
                oneCubeSide(ON_RIGHT);
            else if (fieldData.equals("RRR"))
                sideScale(ON_RIGHT);
            else if (fieldData.equals("LRL"))
                sideScale(ON_RIGHT);
        }
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
    
    private void sideScale (boolean onLeft) {
        addSequential (new CombinedPositionAnglePID(AutonomousConstants.SCALE_DISTANCE_FEET, 0));
        addSequential(new TurnToAngle((onLeft ? 1 : -1) * 90));
        addSequential (new MoveElevatorMotionMagic(ElevatorConstants.SCALE_HIGH_HEIGHT));
        addSequential(new SetSolenoid (IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
        addSequential (new IntakeOuttakeTimed(AutonomousConstants.SCALE_OUTTAKE_TIME, IntakeType.OUTTAKE));
    }

    private void baseline () {
        addSequential (new CombinedPositionAnglePID(AutonomousConstants.BASELINE_DISTANCE, 0));
    }

    private void oneCubeSide (boolean onLeft) {
        FollowPath path = setupPathFollowerArc (AutonomousConstants.LLS_P1_SWITCH_LEFT, AutonomousConstants.LLS_P1_SWITCH_RIGHT, false, null).zeroPigeonAtStart(false).resetSensors(true);
        CommandGroup driveFirstCube = new CommandGroup();
            if (onLeft)
                driveFirstCube.addSequential (path);
            else
                driveFirstCube.addSequential (setupPathFollowerArc (AutonomousConstants.RRS_P1_SWITCH_LEFT, AutonomousConstants.RRS_P1_SWITCH_RIGHT, false, null).zeroPigeonAtStart(false).resetSensors(true));
        addParallel(driveFirstCube);
        CommandGroup raiseElevatorFirstCube = new CommandGroup();
                raiseElevatorFirstCube.addSequential(new PauseUntilPathBegins(path, PauseType.END_OF_PATH, 1.9, path.getTotalTime()));
                raiseElevatorFirstCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.SWITCH_HEIGHT_AUTON));
        addParallel(raiseElevatorFirstCube);
        CommandGroup outtakeFirstCube = new CommandGroup();
            outtakeFirstCube.addSequential(new PauseUntilPathBegins(path, PauseType.END_OF_PATH, 0.15, path.getTotalTime()));
            outtakeFirstCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                    IntakeConstants.DECOMPRESS));
            outtakeFirstCube.addSequential(new IntakeOuttakeTimed(0.17, IntakeType.OUTTAKE));
        addParallel(outtakeFirstCube);
    }
    private void oneCubeCenter (boolean onLeft) {
        FollowPath fpc1 = setupPathFollowerArc(AutonomousConstants.CLH_P1_LEFT, AutonomousConstants.CLH_P1_RIGHT, 
        false, null).zeroPigeonAtStart(false).resetSensors(true);
        CommandGroup firstCube = new CommandGroup();
            firstCube.addParallel(fpc1);
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
    }
    /**
     * The command to be performed for a switch autonomous.
     * @param onLeft true if the switch is on the left; false otherwise
     */
    private void switchAuton (boolean onLeft)
    {
        FollowPath fpc1, fpc2, fpc5, fpc6, fpc7, fpc8, fpc9;
        CombinedPositionAnglePID fpc3, fpc4;
        //DriveToPositionCommand fpc3;
        fpc1 = setupPathFollowerArc(AutonomousConstants.CLH_P1_LEFT, AutonomousConstants.CLH_P1_RIGHT, 
                false, null).zeroPigeonAtStart(false).resetSensors(true);
        fpc2 = setupPathFollowerArc(AutonomousConstants.CLH_P2_LEFT_REV, AutonomousConstants.CLH_P2_RIGHT_REV, true, 
                fpc1).zeroPigeonAtStart(false).resetSensors(false);
        fpc3 = new CombinedPositionAnglePID(2.95, 0);
        fpc4 = new CombinedPositionAnglePID(-3.25, 0);
        fpc5 = setupPathFollowerArc
                (AutonomousConstants.CLH_P5_LEFT, AutonomousConstants.CLH_P5_RIGHT, false, fpc2)
                .zeroPigeonAtStart(false).resetSensors(true);
        //addSequential (new PrintTimeToConsole());
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
                intakeSecondCubeDuringPath.addSequential(new IntakeOuttakeTimed(0.4, IntakeType.INTAKE));
                intakeSecondCubeDuringPath.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
            scoreSecondCube.addParallel(intakeSecondCubeDuringPath);
            CommandGroup outtakeSecondCube = new CommandGroup();
                outtakeSecondCube.addSequential(new PauseUntilPathBegins(fpc5, PauseType.END_OF_PATH, 
                        2, fpc5.getTotalTime()));
                outtakeSecondCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
                outtakeSecondCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY,
                    IntakeConstants.UP));
                outtakeSecondCube.addSequential(new MoveElevatorMotionMagic
                        (ElevatorConstants.SWITCH_HEIGHT_AUTON));
                outtakeSecondCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY,
                        IntakeConstants.DECOMPRESS));
                outtakeSecondCube.addSequential(new IntakeOuttakeTimed(0.34, IntakeType.OUTTAKE));
            scoreSecondCube.addParallel(outtakeSecondCube);
        addSequential(scoreSecondCube); 
        getThirdCube();
    }
    
    /**
     * Method to intake and score the third cube for a center switch autonomous.
     */
    private void getThirdCube()
    {
        PositionCommand fpc6 = new CombinedPositionAnglePID(-3.9, 0).setAllowableError(300), 
                fpc7 = new CombinedPositionAnglePID(5, -45).setAllowableError(300),
                fpc8 = new DriveToPosition(-1.5).setAllowableError(650), 
                fpc9 = new DriveToPosition(2.3).setAllowableError(1000);
        TurnToAngle turn6 = new TurnToAngle(-45, 0.8), turn8 = new TurnToAngle(20, 1.25);
        CommandGroup thirdCube = new CommandGroup();
            CommandGroup thirdCubePaths = new CommandGroup();
                thirdCubePaths.addSequential(fpc6);
                thirdCubePaths.addSequential(turn6);
                thirdCubePaths.addSequential(fpc7);
                thirdCubePaths.addSequential(new Delay(.5));
                thirdCubePaths.addSequential(fpc8);
                thirdCubePaths.addSequential(turn8);
                thirdCubePaths.addSequential(fpc9);
            thirdCube.addParallel(thirdCubePaths);
            CommandGroup raiseElevatorThirdCube = new CommandGroup();
                raiseElevatorThirdCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
                raiseElevatorThirdCube.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.INTAKE_HEIGHT));
                raiseElevatorThirdCube.addSequential(new PauseUntilReachingPosition(fpc7, 0.2));
                raiseElevatorThirdCube.addSequential(new IntakeOuttakeTimed(1.1, IntakeType.INTAKE));
                
                raiseElevatorThirdCube.addSequential(new PauseUntilReachingPosition(fpc8, .7));
                
                raiseElevatorThirdCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
                raiseElevatorThirdCube.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
                //raiseElevatorThirdCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
                raiseElevatorThirdCube.addSequential(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
                CommandGroup raiseElevatorOuttake = new CommandGroup();
                    raiseElevatorOuttake.addParallel(new MoveElevatorMotionMagic
                            (ElevatorConstants.SWITCH_HEIGHT_THIRD_CUBE));
                    raiseElevatorOuttake.addParallel(new IntakeOuttakeTimed(.7, IntakeType.OUTTAKE));
                raiseElevatorThirdCube.addSequential(raiseElevatorOuttake);
            thirdCube.addParallel(raiseElevatorThirdCube);
           addSequential(thirdCube);
    }
    
    /**
     * Reads in a trajectory (from Jaci's pathfinder) given the filename as a CSV.
     * @param filename the file name
     * @return the Trajectory
     * @throws FileNotFoundException if the file is not successfully found
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
    
    /**
     * Sets up a motion profile arc path.
     * @param leftFileName the file name of the left path 
     * @param rightFileName the file name of the right path
     * @param reverse if true: perform the trajectory in reverse order; if false: perform it normally
     * @param prevPath the path which comes before this one (in a series of multiple paths)
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
            ;
        }
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon(), reverse, endAngleLeft);
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon(), reverse, endAngleRight);

        int numPoints = (leftPath1.segments.length + rightPath1.segments.length)/2;
        fpc.setTotalTime(numPoints * RobotMap.TIME_PER_TRAJECTORY_POINT_MS);
        return fpc;
    }
}
