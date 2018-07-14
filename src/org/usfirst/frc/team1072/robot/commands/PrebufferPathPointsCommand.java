package org.usfirst.frc.team1072.robot.commands;

import java.util.HashMap;

import org.usfirst.frc.team1072.robot.Robot;

import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

/**
 * Prebuffers the points in a path to improve autonomous speed.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class PrebufferPathPointsCommand extends Command
{
    private FollowPathCommand fpc;
    private HashMap<IMotorController, Trajectory> masterControllers;
    private boolean isFinished;
    
    /**
     * Constructs a new PrebufferPathPointsCommand.
     * @param fpc the path to be buffered
     */
    public PrebufferPathPointsCommand(FollowPathCommand fpc)
    {
        masterControllers = new HashMap<IMotorController, Trajectory>();
        masterControllers.put(Robot.dt.getRightTalon(), fpc.getControllerTrajectory(Robot.dt.getRightTalon()));
        if (!(fpc instanceof FollowPathArcCommand))
        {
            masterControllers.put(Robot.dt.getLeftTalon(), fpc.getControllerTrajectory(Robot.dt.getLeftTalon()));
        }
    }
    
    /**
     * Executes the command by pushing points to the RoboRio buffer.
     */
    public void execute()
    {
        for (IMotorController imc : masterControllers.keySet())
        {
            fpc.loadTrajectoryToTalon(masterControllers.get(imc), imc);
        }
        isFinished = true;
    }
    /**
     * Determines whether the command has finished.
    *  @return true if all the points have been pushed to the RoboRio buffer
    */
    @Override
    protected boolean isFinished()
    {
        isFinished = true;
        for (IMotorController imc : masterControllers.keySet())
            isFinished = isFinished && fpc.getControllerTrajectoryLoaded(imc);
        return isFinished;
    }
}
