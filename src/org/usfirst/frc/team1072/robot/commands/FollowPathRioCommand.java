package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Stack;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * 
 * @author Finn Frankis
 * @version 6/30/18
 */
public class FollowPathRioCommand extends Command
{
    private static final int NUM_TALONS = 2;
    private final int minPointsInController = 3;
    private Notifier pointProcessor;
    private PointProcessor pp;
  
    // store trajectory at index 0, status at index 1
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private Map<IMotorController, Object[]> controllers;
    
    private int pathState;
    
    
    private double totalTime;
    private double startTime;   
    
    boolean hasStarted;
    
    public FollowPathRioCommand()
    {
        controllers = new HashMap<IMotorController, Object[]>();
        //pp = new PointProcessor();
        pointProcessor = new Notifier(pp);
    }

    public void initialize()
    {
        for (IMotorController imc : controllers.keySet())
        {
            ((TalonSRX) imc).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID, RobotMap.TIMEOUT);
            imc.setSelectedSensorPosition(RobotMap.DT_MOTION_PROFILE_PID, 0, RobotMap.TIMEOUT);
        }
        startTime = -1;
        pathState = 0;
        totalTime = 0;
    }
    
    public void execute()
    {
        boolean isReady = true;
        if (controllers.keySet().size() == NUM_TALONS)
        {
            Iterator<IMotorController> it = controllers.keySet().iterator();
            for (int i = 0; i < NUM_TALONS; i++)
            {
                if (getControllerTrajectory(it.next()) == null)
                    isReady = false;
            }
            
            
        }

        if (startTime == -1 && isReady)
        {
            startTime = Timer.getFPGATimestamp() * RobotMap.MS_PER_SEC;
            Robot.dt.getPIDController().setContinuous(false);
            Robot.dt.getPIDController().setAbsoluteTolerance(RobotMap.DT_POS_ALLOWABLE_ERROR);
            Robot.dt.enablePID();
           

            
        }
    }
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath)
    {
        if (reversePath)
            t = reverseTrajectory(t);
        processTrajectory(t);
        controllers.put(controller, new Object[] {t, null, new Double(0)});
    }
    
    /**
     * Modifies the trajectory to have the correct units.
     * @param t the trajectory to modify
     */
    private void processTrajectory (Trajectory t)
    {
        for (Segment s : t.segments)
        {
            s.position = s.position / (RobotMap.WHEELDIAMETER * Math.PI / 12) * RobotMap.TICKS_PER_REV; // convert revolutions to encoder units
            s.velocity = s.velocity / (RobotMap.WHEELDIAMETER * Math.PI / 12)  * RobotMap.TICKS_PER_REV / 10; // convert revs/100ms to seconds;
            s.heading = s.heading * RobotMap.DEGREES_PER_RADIAN; // convert radians to degrees
        }

    }
    
    public double getStartTime()
    {
        return startTime;
    }
    
    public Map<IMotorController, Object[]> getControllers()
    {
        return controllers;
    }
    
    class PointProcessor implements java.lang.Runnable {
        
        Segment[] leftPoints;
        Segment[] rightPoints;
        
        public PointProcessor()
        {
            Iterator<IMotorController> it = controllers.keySet().iterator();
            leftPoints = getControllerTrajectory(it.next()).segments;
            rightPoints = getControllerTrajectory(it.next()).segments;
        }
        
        @Override
        public void run()
        {
           double currentTime =  Timer.getFPGATimestamp() * RobotMap.MS_PER_SEC;
           double timeElapsed = currentTime - startTime;
           int startPoint = (int) (timeElapsed/RobotMap.TIME_PER_TRAJECTORY_POINT_MS);
           int endPoint = startPoint + 1;
           if (endPoint >= leftPoints.length)
           {
               
           }
           else
           {
               // interpolate to find point for current time
               double interpStartTime = startPoint * RobotMap.TIME_PER_TRAJECTORY_POINT_MS;
               double interpEndTime = endPoint * RobotMap.TIME_PER_TRAJECTORY_POINT_MS;
               
               double startPosLeft = leftPoints[startPoint].position;
               double endPosLeft = leftPoints[endPoint].position;
               double slopeLeft = (endPosLeft - startPosLeft) / (interpEndTime - interpStartTime);
               
               double startPosRight = rightPoints[startPoint].position;
               double endPosRight = rightPoints[endPoint].position;
               double slopeRight = (endPosRight - startPosRight) / (interpEndTime - interpStartTime);
               
               double targetPosLeft = slopeLeft * (currentTime - interpStartTime) + startPosLeft;
               double targetPosRight = slopeRight * (currentTime - interpStartTime) + startPosRight;
               
               Robot.dt.setSetpoint((targetPosLeft + targetPosRight)/2);
               
               
           }
           
        }
        
    }
        
    

        
    

    @Override
    protected boolean isFinished()
    {
        return pathState == 3;
    }
    
    @Override
    public void cancel()
    {
        disable();
    }
    @Override
    protected void end()
    {
        disable();
    }
    
    @Override
    protected void interrupted()
    {
        disable();
    }
    
    public void disable() {
        System.out.println("DISABLING");
        Robot.dt.disablePID();
        for (IMotorController imc : controllers.keySet())
        {
            imc.clearMotionProfileTrajectories();
            imc.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
        }
    }
    
    
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        return (MotionProfileStatus) controllers.get(controller)[1];//STAT_INDEX];
    }
    
    public Trajectory getControllerTrajectory (IMotorController controller)
    {
        return (Trajectory) controllers.get(controller)[TRAJ_INDEX];
    }
    
    public double getTotalTime(IMotorController controller)
    {
        return (double) controllers.get(controller)[2];
    }
    
    /**
     * Reverses a given trajectory (assists in creating backward paths).
     * @param t the trajectory to reverse 
     * @return the reversed trajectory
     */
    private Trajectory reverseTrajectory(Trajectory t)
    {
        Stack<Segment> segments = new Stack<Segment>();
        for (Segment s : t.segments)
        {
            s.velocity *= -1;
            s.position *= -1;
            segments.push(s);
        }
        for (int i = 0; i < segments.size(); i++)
            t.segments[i] = segments.pop();
        
        return t;
    }
}
