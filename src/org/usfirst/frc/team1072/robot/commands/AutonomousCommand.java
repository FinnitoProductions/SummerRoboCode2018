package org.usfirst.frc.team1072.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class AutonomousCommand extends CommandGroup
{
    public AutonomousCommand ()
    {
        FollowPathCommand fpc = new FollowPathCommand();
        
        // file names should be constants (in future)
        Trajectory leftPath1 = null;
        Trajectory rightPath1 = null;
        
        try
        {
            leftPath1 = readTrajectory("/home/summer2018/paths/test_5ft/test_5ft_left_detailed.csv");
            rightPath1 = readTrajectory("/home/summer2018/paths/test_5ft/test_5ft_right_detailed.csv"); 
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("FILE. NOT. FOUND.");
        }
        System.out.println("read succeess");
        fpc.addProfile(leftPath1, Robot.dt.getLeftTalon());
        fpc.addProfile(rightPath1, Robot.dt.getRightTalon());
        addSequential(fpc);
        System.out.println("logging command");
        
        
        addSequential(new MoveElevatorMotionMagicCommand(RobotMap.EL_SCALE_LOW_HEIGHT));
        addSequential(new IntakeOuttakeCubeCommand(RobotMap.NO_MANUAL_INTAKE, RobotMap.INTAKE_BOOL));
    }
    
    public static Trajectory readTrajectory(String filename) throws FileNotFoundException {
        File f = new File(filename);
        if(f.exists() && f.isFile() && filename.endsWith(".csv")) {
            try {
                return Pathfinder.readFromCSV(f);
            } catch(Exception e) {
                throw new FileNotFoundException("Pathfinder failed to read trajectory: " + filename);
            }
        } else {
            throw new FileNotFoundException("Trajectory: " + filename + ", does not exist or is not a csv file");
        }
           }
    
    
}
