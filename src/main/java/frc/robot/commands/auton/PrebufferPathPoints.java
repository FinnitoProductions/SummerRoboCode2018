package frc.robot.commands.auton;

import java.util.HashMap;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

/**
 * Prebuffers the points in a path to improve autonomous speed.
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class PrebufferPathPoints extends Command
{
    /**
     * The path which this command will prebuffer.
     */
    private FollowPath fpc;
    
    /**
     * The map containing all master (aka: not follower) controllers which should be 
     * prebuffered.
     */
    private HashMap<IMotorController, Trajectory> masterControllers;
   
    /**
     * Whether or not the prebuffering has completed.
     */
    private boolean isFinished;
    
    /**
     * Constructs a new PrebufferPathPointsCommand.
     * @param fpc the path to be buffered
     */
    public PrebufferPathPoints(FollowPath fpc)
    {
        this.fpc = fpc;
        masterControllers = new HashMap<IMotorController, Trajectory>();
        masterControllers.put(Robot.dt.getRightMaster(), fpc.getControllerTrajectory(Robot.dt.getRightMaster()));
        if (!(fpc instanceof FollowPathArc))
        {
            masterControllers.put(Robot.dt.getLeftMaster(), fpc.getControllerTrajectory(Robot.dt.getLeftMaster()));
        }
    }
    
    /**
     * Executes the command by pushing points to the RoboRio buffer.
     */
    public void execute()
    {
        ;
        for (IMotorController imc : masterControllers.keySet())
        {
            fpc.loadTrajectoryToTalon(masterControllers.get(imc), imc);
        }
        ;
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
