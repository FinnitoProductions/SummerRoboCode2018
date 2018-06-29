package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * 
 * @author Finn Frankis
 * @version 6/20/18
 */
public class FollowPathCommand extends Command
{
    private final int minPointsInController = 3;
    private ProcessBuffer p;
    private Notifier notif;
    private Notifier sensorSumPeriodic;
  
    // store trajectory at index 0, status at index 1
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private Map<IMotorController, Object[]> controllers;
    
    private int pathState;
    
    private boolean enableNotifier;
    private int outerPort;
    
    
    private double totalTime;
    
    public FollowPathCommand()
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        outerPort = -1;
        enableNotifier = false;
    }
    
    public FollowPathCommand(int outerPort, boolean enableNotifier)
    {
        this.enableNotifier = enableNotifier;
        this.outerPort = outerPort;
        
    }
    
    public void initialize()
    {
        for (IMotorController imc : controllers.keySet())
        {
            ((TalonSRX) imc).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID, RobotMap.TIMEOUT);
            imc.setSelectedSensorPosition(RobotMap.DT_MOTION_PROFILE_PID, 0, RobotMap.TIMEOUT);
        }
        pathState = 0;
        totalTime = 0;
        System.out.println("initializing command");
        double period = 1.0 * RobotMap.PERIOD_IN_MS / RobotMap.MS_PER_SEC / 2 / 2.5;
        notif = new Notifier(p);
        notif.startPeriodic(period);
        if (enableNotifier)
            sensorSumPeriodic.startPeriodic(RobotMap.TALON_ENCODER_SUM_PERIOD_MS / RobotMap.MS_PER_SEC);
        
        System.out.println("Initialized with period " + period);
    }
    
    public void execute()
    {
        for (IMotorController controller : controllers.keySet())
        {
            MotionProfileStatus motionStatus = new MotionProfileStatus();
            controller.getMotionProfileStatus(motionStatus);
            controllers.get(controller)[STAT_INDEX] = motionStatus;
        }
        
        System.out.println("SWITCHING NOW " + pathState);
        switch(pathState)
        {
            // ready to begin loading
            case 0:
            {
                System.out.println("CASE 0");
                 // halve for optimal communication
                for (IMotorController controller : controllers.keySet())
                {
                    controller.selectProfileSlot(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID);
                    controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
                    loadTrajectoryToTalon(getControllerTrajectory(controller), controller);
                    MotionProfileStatus status = new MotionProfileStatus();
                    controller.getMotionProfileStatus(status);
                    System.out.println(controller.getDeviceID() + " Buffer After Pushed: " + status.btmBufferCnt);
                    
                }
                System.out.println("Loaded points correctly.");
                pathState = 1;
                break;
            }
            
            // ready to begin 
            case 1:
            {
                System.out.println("CASE 1");
                // once enough points have been buffered, begin sequence
                boolean allReady = true;
                for (IMotorController controller : controllers.keySet())
                {
                    allReady = allReady && getControllerStatus(controller).btmBufferCnt > minPointsInController;
                }
                
                System.out.println("All Ready? " + allReady);
                if (allReady)
                {
                  
                    System.out.println("I am ready.");
                    for (IMotorController controller : controllers.keySet())
                    {
                        
                        System.out.println("Enabling profile.");
                        controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
                        
                    }
                    pathState = 2;
                    SmartDashboard.putNumber("ACTIVE LEFT POS: ", controllers.keySet().iterator().next().getActiveTrajectoryPosition());
                    SmartDashboard.putNumber("ACTIVE LEFT VEL: ", controllers.keySet().iterator().next().getActiveTrajectoryVelocity());
                    SmartDashboard.putNumber("ACTIVE LEFT HEAD: ", controllers.keySet().iterator().next().getActiveTrajectoryHeading());
                }
                break;
            }
            
            // check up on profile to see if done
            case 2: 
            {
                System.out.println("CASE 2");
                boolean isFinished = true;
                System.out.println();
                for (IMotorController controller : controllers.keySet())
                {
                        MotionProfileStatus status = new MotionProfileStatus();
                        controller.getMotionProfileStatus(status);
                        System.out.println("Buffer Count: " + status.btmBufferCnt);
                        isFinished = isFinished && status.btmBufferCnt == 0;
                        
                        /*if (getControllerStatus(controller).isUnderrun)
                        {
                            controller.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
                            System.out.println(controller.getDeviceID() + " IS UNDERRUN");
                        }*/
                }
                

                
                if (isFinished) 
                {
                    System.out.println("FINISHED");
                    pathState = 3;
                }
                break;
            }
           
        }           
    }
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath)
    {
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.PERIOD_IN_MS / 2));
        p.addController(controller);
        if (reversePath)
            t = reverseTrajectory(t);
        controllers.put(controller, new Object[] {t, null, new Double(0)});
    }
    
    private void loadTrajectoryToTalon(Trajectory t, IMotorController controller)
    {
        if (t != null)
        {
            if (getControllerStatus(controller) != null && getControllerStatus(controller).isUnderrun)
            {
                controller.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
                System.out.println("IS UNDERRUN");
            }
            
            // clears existing trajectories
            controller.clearMotionProfileTrajectories();
            
            // sets up a base period which will be ADDED TO the time of each trajectory point (usually zero)
            //controller.configMotionProfileTrajectoryPeriod(RobotMap.AUTON_BASE_PERIOD, RobotMap.TIMEOUT);
            controller.configMotionProfileTrajectoryPeriod(RobotMap.PERIOD_IN_MS, RobotMap.TIMEOUT);
            // constructs Talon-readable trajectory points out of each segment
            Segment[] segs = t.segments;
            controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
            for (int i = 0; i < segs.length; i++)
            {
                TrajectoryPoint tp = new TrajectoryPoint();
                tp.position = segs[i].position / (RobotMap.WHEELDIAMETER * Math.PI / 12) * RobotMap.TICKS_PER_REV; // convert revolutions to encoder units
                tp.velocity = segs[i].velocity / (RobotMap.WHEELDIAMETER * Math.PI / 12)  * RobotMap.TICKS_PER_REV / 10; // convert revs/100ms to seconds;
                tp.headingDeg = segs[i].heading * RobotMap.DEGREES_PER_RADIAN; // convert radians to degrees
                tp.timeDur = TrajectoryDuration.valueOf((int)segs[i].dt); // convert to correct units
                tp.profileSlotSelect0 = RobotMap.DT_MOTION_PROFILE_PID;
                
                if (outerPort >= 0) 
                {
                    tp.profileSlotSelect1 = outerPort;
                    tp.auxiliaryPos = tp.headingDeg;
                }
                tp.zeroPos = false;
                if (i == 0)
                    tp.zeroPos = true; // specify that this is the first point
                else if (i == (segs.length-1))
                    tp.isLastPoint = true;
                
                controller.pushMotionProfileTrajectory(tp); // push point to talon
                controllers.get(controller)[2] = new Double(segs[i].dt * 1000) + new Double((double) controllers.get(controller)[2]);
            }
        }
        else
        {
            System.out.println("TRAJECTORY WAS NULL!");
        }
    }
    
    
    class ProcessBuffer implements java.lang.Runnable {
        private List<IMotorController> motorControllers;
        
        public ProcessBuffer()
        {
            motorControllers = new ArrayList<IMotorController>();
        }
        
        public void addController (IMotorController control)
        {
            motorControllers.add(control);
        }
        
        public void run()
        {
            for (int i = 0; i < motorControllers.size(); i++)
            {
                MotionProfileStatus status = new MotionProfileStatus();
                motorControllers.get(i).getMotionProfileStatus(status);
                //System.out.println("i= " + i + " BUFFER: " + status.btmBufferCnt);
                motorControllers.get(i).processMotionProfileBuffer();
            }
        }
        
    }

    class TalonEncoderSumLoop implements java.lang.Runnable {

        @Override
        public void run()
        {
            // TODO Auto-generated method stub
            int total = 0;
            int count = 0;
            for (IMotorController imc : controllers.keySet())
            {
                total += imc.getSelectedSensorPosition(RobotMap.PRIMARY_PID);
                count++;
            }
            
            int avg = total / count;
            for (IMotorController imc : controllers.keySet())
            {
                imc.setSelectedSensorPosition(RobotMap.DT_MOTION_PROFILE_PID, avg, RobotMap.TIMEOUT);
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
        notif.stop();
        sensorSumPeriodic.stop();
        for (IMotorController imc : controllers.keySet())
        {
            imc.clearMotionProfileTrajectories();
            imc.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
            // imc.set(ControlMode.MotionProfile, MotionProfile.);
        }
    }
    
    
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        return (MotionProfileStatus) controllers.get(controller)[1];//STAT_INDEX];
    }
    
    private Trajectory getControllerTrajectory (IMotorController controller)
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
