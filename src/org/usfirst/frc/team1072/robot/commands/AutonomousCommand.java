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

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class AutonomousCommand extends CommandGroup
{
    public AutonomousCommand(Subsystem[] subsystems)
    {
        for (Subsystem s : subsystems)
            requires(s);

        /*addSequential(setupPathFollower("/home/summer2018/paths/test_5ft/test_5ft_left_detailed.csv", 
                "/home/summer2018/paths/test_5ft/test_5ft_right_detailed.csv"));*/
        /*addSequential(setupPathFollower("/home/summer2018/paths/curved_path/curved_path_left_detailed.csv", 
                "/home/summer2018/paths/curved_path/curved_path_right_detailed.csv"));*/
        
        
        oneCubeSwitch(true);
        
        
        
        /*addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_DECOMPRESS));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN));
        
        addSequential(new IntakeOuttakeTimedCommand(2, RobotMap.INTAKE_BOOL));
        
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_COMPRESS));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_UP));
        
        addSequential(new MoveElevatorMotionMagicCommand(RobotMap.EL_SCALE_LOW_HEIGHT));
        
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_DECOMPRESS));
        
        addSequential(new IntakeOuttakeTimedCommand(2, RobotMap.OUTTAKE_BOOL));*/
    }

    private void oneCubeSwitch (boolean onLeft)
    {
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_COMPRESS));
        
        FollowPathCommand fpc1;
        if (onLeft)
        {
             fpc1 = setupPathFollower("/home/summer2018/paths/test_switch_auton/test_switch_auton_left_detailed.csv", 
                    "/home/summer2018/paths/test_switch_auton/test_switch_auton_right_detailed.csv");
            addSequential(fpc1);
        }
        else
        {
            fpc1 = setupPathFollower("/home/summer2018/paths/test_switch_auton/right_switch_auton_left_detailed.csv", 
                    "/home/summer2018/paths/test_switch_auton/right_switch_auton_right_detailed.csv");
            addSequential(fpc1);
        }
        /*addParallel(new MoveElevatorMotionMagicCommand((fpc1.getTotalTime(Robot.dt.getLeftTalon()) + fpc1.getTotalTime(Robot.dt.getLeftTalon()))/2  - 1000, 
                RobotMap.EL_SWITCH_HEIGHT));

        
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_UPDOWN_KEY, RobotMap.INTAKE_DOWN));
        addSequential(new SetSolenoidCommand(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, RobotMap.INTAKE_DECOMPRESS));
        
        addSequential(new IntakeOuttakeTimedCommand(2, RobotMap.OUTTAKE_BOOL));*/
    }
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

    private FollowPathCommand setupPathFollower(String leftFileName, String rightFileName)
    {
        FollowPathCommand fpc = new FollowPathCommand();

        // file names should be constants (in future)
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
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon());
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon());
        return fpc;
    }

}
