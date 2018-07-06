package org.usfirst.frc.team1072.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.TimedCommand;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * A set of commands to be called during the autonomous period.
 * @author Finn Frankis
 * @version 6/20/18
 */
public class AutonomousCommand extends CommandGroup
{
    private int numPoints = 0;
    
    private final String CENTER_RIGHT_HEAD_ON_ONE_CUBE_LEFT = "/home/summer2018/paths/center_right_headon(switch_1)/center_right_headon(switch_1)_left_detailed.csv";
    private final String CENTER_RIGHT_HEAD_ON_ONE_CUBE_RIGHT = "/home/summer2018/paths/center_right_headon(switch_1)/center_right_headon(switch_1)_right_detailed.csv";
    
    private final String PATH_TEST_LEFT = "/home/summer2018/paths/simplepathtest/simplepathtest_left_detailed.csv";
    private final String PATH_TEST_RIGHT = "/home/summer2018/paths/simplepathtest/simplepathtest_right_detailed.csv";
    
    public AutonomousCommand(Subsystem[] subsystems)
    {
        for (Subsystem s : subsystems)
            requires(s);

        /*addSequential(setupPathFollower("/home/summer2018/paths/test_5ft/test_5ft_left_detailed.csv", 
                "/home/summer2018/paths/test_5ft/test_5ft_right_detailed.csv"));*/
        /*addSequential(setupPathFollower("/home/summer2018/paths/curved_path/curved_path_left_detailed.csv", 
                "/home/summer2018/paths/curved_path/curved_path_right_detailed.csv"));*/
        
        oneCubeSwitch(true);

    }

    private void oneCubeSwitch (boolean onLeft)
    {
        FollowPathCommand fpc1;
        //addParallel(new MoveElevatorMotionMagicCommand(numPoints * RobotMap.TIME_PER_TRAJECTORY_POINT_MS - 1000, RobotMap.EL_SWITCH_HEIGHT));
        if (onLeft)
        {
             fpc1 = setupPathFollowerArc(PATH_TEST_LEFT, PATH_TEST_RIGHT, false);//CENTER_RIGHT_HEAD_ON_ONE_CUBE_LEFT, CENTER_RIGHT_HEAD_ON_ONE_CUBE_RIGHT, false);/*"/home/summer2018/paths/test_switch_auton/test_switch_auton_left_detailed.csv", 
                    //"/home/summer2018/paths/test_switch_auton/test_switch_auton_right_detailed.csv");*/ /**/
        }
        else
        {
            fpc1 = setupPathFollowerArc("/home/summer2018/paths/test_switch_auton/right_switch_auton_left_detailed.csv", 
                    "/home/summer2018/paths/test_switch_auton/right_switch_auton_right_detailed.csv", false);
        }
        addSequential (fpc1);
        /*(addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_UP));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_COMPRESS));
        

        
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_DECOMPRESS));
        
        addSequential(new IntakeOuttakeTimedCommand(2, RobotMap.OUTTAKE_BOOL));*/

        //addSequential(new DriveToPositionCommand(Robot.dt.getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX), Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)));
        //addSequential(new TurnRobotToAngleCommand(90));
        
                
        
        
        
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

    @SuppressWarnings("unused")
    private FollowPathCommand setupPathFollower(String leftFileName, String rightFileName, boolean reverse)
    {
        FollowPathCommand fpc = new FollowPathCommand();

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
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon(), reverse);
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon(), reverse);
        numPoints = (leftPath1.segments.length + rightPath1.segments.length)/2;
        return fpc;
    }
    
    /**
     * Sets up a motion profile arc path.
     * @param leftFileName the file name of the left path 
     * @param rightFileName the file name of the right path
     * @param reverse if true: perform the trajectory in reverse order; if false: perform it normally
     * @return the new follow path command
     */
    private FollowPathCommand setupPathFollowerArc(String leftFileName, String rightFileName, boolean reverse)
    {
        FollowPathArcCommand fpc = new FollowPathArcCommand();

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
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon(), reverse);
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon(), reverse);
        numPoints = (leftPath1.segments.length + rightPath1.segments.length)/2;
        return fpc;
    }
}
