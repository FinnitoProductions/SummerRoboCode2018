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
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
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
  
    // store trajectory at index 0, status at index 1
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private Map<IMotorController, Object[]> controllers;
    
    private int pathState;
    

    private int outerPort;
    
    
    private double totalTime;
    
    public FollowPathCommand()
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        outerPort = -1;
    }
    
    public FollowPathCommand(int outerPort)
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        this.enableNotifier = enableNotifier;
        this.outerPort = outerPort;
        
    }
    
    public void initialize()
    {
        for (IMotorController imc : controllers.keySet())
        {
            imc.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            imc.setSelectedSensorPosition(0, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        }
        
        pathState = 0;
        totalTime = 0;
        double period = 1.0 * RobotMap.TIME_PER_TRAJECTORY_POINT_MS / RobotMap.MS_PER_SEC / 2;
        notif = new Notifier(p);
        notif.startPeriodic(period);
        
        Robot.dt.getLeftTalon().setSensorPhase(RobotMap.DT_LEFT_TALON_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(RobotMap.DT_RIGHT_TALON_PHASE);
        
        if (outerPort == -1) // no auxiliary/arc
        {
            for (IMotorController imc : controllers.keySet())
            {
                Robot.dt.selectProfileSlots(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
            }
        }
        else
        {
            Robot.dt.getLeftTalon().follow(Robot.dt.getRightTalon(), FollowerType.AuxOutput1);
            
            Robot.dt.getRightTalon().selectProfileSlot(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
            Robot.dt.getRightTalon().selectProfileSlot(RobotMap.DT_ANGLE_PID, RobotMap.AUXILIARY_PID_INDEX);
            
            Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getLeftTalon().getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_1, RobotMap.TIMEOUT);
            

            
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
            
            Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(RobotMap.PIGEON_REMOTE_SENSOR_TYPE, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(0.5,
                    RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT); // set to average
            
        }
    }
    
    public void execute()
    {
        for (IMotorController controller : controllers.keySet())
        {
            MotionProfileStatus motionStatus = new MotionProfileStatus();
            controller.getMotionProfileStatus(motionStatus);
            controllers.get(controller)[STAT_INDEX] = status;

        switch(pathState)
        {
            // ready to begin loading
            case 0:
            {
<<<<<<< HEAD
                IMotorController controller = Robot.dt.getRightTalon();
                controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
                loadTrajectoryToTalon(getControllerTrajectory(controller), controller);

=======
                 // halve for optimal communication
                    IMotorController controller = Robot.dt.getRightTalon();
                    controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
                    loadTrajectoryToTalon(getControllerTrajectory(controller), controller);
                    MotionProfileStatus status = new MotionProfileStatus();
                    controller.getMotionProfileStatus(status);
                    System.out.println(controller.getDeviceID() + " Buffer After Pushed: " + status.btmBufferCnt);
                    
                
                System.out.println("Loaded points correctly.");
>>>>>>> parent of 1b81f40... Fixed compilation errors
                pathState = 1;
                break;
            }
            
            // ready to begin 
            case 1:
            {
                // once enough points have been buffered, begin sequence
                boolean allReady = getControllerStatus(Robot.dt.getRightTalon()).btmBufferCnt > minPointsInController;

                System.out.println("All Ready? " + allReady);
                if (allReady)
                {
                  
                    System.out.println("I am ready.");
                    IMotorController controller = Robot.dt.getRightTalon();
                        
                        System.out.println("Enabling profile.");
                        controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
                        
                    pathState = 2;
                }
                break;
            }
            
            // check up on profile to see if done
            case 2:
            {
            
                IMotorController controller = Robot.dt.getRightTalon();
                MotionProfileStatus status = new MotionProfileStatus();
                controller.getMotionProfileStatus(status);
                System.out.println("Buffer Count: " + status.btmBufferCnt);
                boolean isFinished = status.btmBufferCnt == 0;    
                
                if (isFinished) 
                {
                    System.out.println("FINISHED");
                    pathState = 3;
                }
                break;
        }
    }
    
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath)
    {
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 2));
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
            
            controller.configMotionProfileTrajectoryPeriod(RobotMap.TIME_PER_TRAJECTORY_POINT_MS, RobotMap.TIMEOUT);
            // constructs Talon-readable trajectory points out of each segment
            Segment[] segs = t.segments;
            controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
            for (int i = 0; i < segs.length; i++)
            {
                TrajectoryPoint tp = new TrajectoryPoint();
                tp.position = segs[i].position / (RobotMap.WHEELDIAMETER * Math.PI / 12) * RobotMap.TICKS_PER_REV; // convert revolutions to encoder units
                tp.velocity = segs[i].velocity / (RobotMap.WHEELDIAMETER * Math.PI / 12)  * RobotMap.TICKS_PER_REV / 10; // convert revs/100ms to seconds;
                
                tp.timeDur = TrajectoryDuration.valueOf((int)segs[i].dt); // convert to correct units
                tp.profileSlotSelect0 = RobotMap.DT_MOTION_PROFILE_PID;
                
                if (outerPort >= 0) 
                {
                    tp.profileSlotSelect1 = outerPort;
                    tp.auxiliaryPos = Robot.radiansToPigeonUnits(segs[i].heading - 2 * Math.PI);
                }
                else
                    //tp.headingDeg = segs[i].heading * RobotMap.DEGREES_PER_RADIAN; // convert radians to degrees
                tp.zeroPos = false;
                if (i == 0)
                    tp.zeroPos = true; // specify that this is the first point
                else if (i == (segs.length-1))
                    tp.isLastPoint = true;
                
                controller.pushMotionProfileTrajectory(tp); // push point to talon
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
                Robot.dt.getRightTalon().processMotionProfileBuffer();
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
        //for (IMotorController imc : controllers.keySet())
        //{
            //imc.clearMotionProfileTrajectories();
            Robot.dt.getRightTalon().set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
            //imc.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
            
        //}
    }
    
    
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        return (MotionProfileStatus) controllers.get(controller)[1];//STAT_INDEX];
    }
    
    private Trajectory getControllerTrajectory (IMotorController controller)
    {
        return (Trajectory) controllers.get(controller)[TRAJ_INDEX];
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
