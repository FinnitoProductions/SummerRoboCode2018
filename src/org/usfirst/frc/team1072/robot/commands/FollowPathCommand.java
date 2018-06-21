package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * 
 * @author Finn Frankis
 * @version 6/20/18
 */
public class FollowPathCommand extends Command
{
    private int minPointsInController = 5;
    private ProcessBuffer p;
    private Notifier notif;
  
    // store trajectory at index 0, status at index 1
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private Map<IMotorController, Object[]> controllers = new HashMap<IMotorController, Object[]>();
    
    private int pathState = 0;
    
    public FollowPathCommand()
    {
        p = new ProcessBuffer();
        notif = new Notifier(p);
        notif.startPeriodic(RobotMap.PERIOD_IN_MS / RobotMap.MS_PER_SEC);
    }
    
    public void execute()
    {
        for (IMotorController controller : controllers.keySet())
        {
            MotionProfileStatus motionStatus = new MotionProfileStatus();
            controller.getMotionProfileStatus(motionStatus);
            controllers.get(controller)[STAT_INDEX] = motionStatus;
        }
        
        switch(pathState)
        {
            // ready to begin loading
            case 0:
            {
                for (IMotorController controller : controllers.keySet())
                {
                    controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
                    loadTrajectoryToTalon(getControllerTrajectory(controller), controller);
                }
                pathState = 1;
            }
            
            // ready to begin 
            case 1:
            {
                // once enough points have been buffered, begin sequence
                boolean allReady = false;
                for (IMotorController controller : controllers.keySet())
                {
                    allReady = allReady || getControllerStatus(controller).btmBufferCnt > minPointsInController;
                }
                
                if (allReady)
                {
                    for (IMotorController controller : controllers.keySet())
                    {
                        controller.selectProfileSlot(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID);
                        controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
                        pathState = 2;
                    }
                }
            }
            
            // check up on profile to see if done
            case 2: 
            {
                boolean isFinished = false;
                for (IMotorController controller : controllers.keySet())
                {
                        isFinished = isFinished || 
                                getControllerStatus(controller).activePointValid &&
                                getControllerStatus(controller).isLast;
                }
                
                if (isFinished) 
                    pathState = 3;
            }
           
        }
        
            
        
        
        
            
    }
    public void addProfile (Trajectory t, IMotorController controller)
    {
        controller.changeMotionControlFramePeriod(RobotMap.PERIOD_IN_MS);
        p.addController(controller);
        
        controllers.put(controller, new Object[2]);
        loadTrajectoryToTalon(t, controller);
    }
    
    private void loadTrajectoryToTalon(Trajectory t, IMotorController controller)
    {
        if (getControllerStatus(controller) != null && getControllerStatus(controller).isUnderrun)
            controller.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
        // clears existing trajectories
        controller.clearMotionProfileTrajectories();
        
        // sets up a base period which will be ADDED TO the time of each trajectory point (usually zero)
        controller.configMotionProfileTrajectoryPeriod(RobotMap.AUTON_BASE_PERIOD, RobotMap.TIMEOUT);
        
        // constructs Talon-readable trajectory points out of each segment
        Segment[] segs = t.segments;
        for (int i = 0; i < segs.length; i++)
        {
            TrajectoryPoint tp = new TrajectoryPoint();
            tp.position = segs[i].position / (RobotMap.WHEELDIAMETER * Math.PI)* RobotMap.TICKS_PER_REV; // convert revolutions to encoder units
            tp.velocity = segs[i].velocity / (RobotMap.WHEELDIAMETER * Math.PI) * RobotMap.TICKS_PER_REV / 10; // convert revs/100ms to seconds;
            tp.headingDeg = segs[i].heading * RobotMap.DEGREES_PER_RADIAN; // convert radians to degrees
            tp.timeDur = TrajectoryDuration.valueOf((int)segs[i].dt); // convert to correct units
            tp.profileSlotSelect0 = 0;
            tp.profileSlotSelect1 = 1;
            tp.zeroPos = false;
            if (i == 0)
                tp.zeroPos = true; // specify that this is the first point
            else if (i == (segs.length-1))
                tp.isLastPoint = true; // specify that this is the last point
            
            controller.pushMotionProfileTrajectory(tp); // push point to talon
        }
    }
    
    
    class ProcessBuffer implements java.lang.Runnable {
        private List<IMotorController> controllers;
        
        public ProcessBuffer()
        {
            controllers = new ArrayList<IMotorController>();
        }
        
        public void addController (IMotorController control)
        {
            controllers.add(control);
        }
        
        public void run()
        {
            for (IMotorController i : controllers)
                i.processMotionProfileBuffer();
        }
        
    }


    @Override
    protected boolean isFinished()
    {
        return pathState == 3;
    }
    
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        System.out.println(controllers.get(controller));
        System.out.println(controllers.keySet());
        return (MotionProfileStatus) controllers.get(controller)[1];//STAT_INDEX];
    }
    
    private Trajectory getControllerTrajectory (IMotorController controller)
    {
        return (Trajectory) controllers.get(controller)[TRAJ_INDEX];
    }
}
