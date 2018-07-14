package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.usfirst.frc.team1072.robot.Robot;

import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

/**
 * 
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class PrebufferPathPointsCommand extends Command
{
    private FollowPathCommand fpc;
    private HashMap<IMotorController, Trajectory> masterControllers;
    
    public PrebufferPathPointsCommand(FollowPathCommand fpc)
    {
        masterControllers = new HashMap<IMotorController, Trajectory>();
        masterControllers.put(Robot.dt.getRightTalon(), fpc.getControllerTrajectory(Robot.dt.getRightTalon()));
        if (!(fpc instanceof FollowPathArcCommand))
        {
            masterControllers.put(Robot.dt.getLeftTalon(), fpc.getControllerTrajectory(Robot.dt.getLeftTalon()));
        }
    }
    
    
    public void execute()
    {
        for (IMotorController imc : masterControllers.keySet())
            fpc.loadTrajectoryToTalon(masterControllers.get(imc), imc);
        
    }
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }
}
