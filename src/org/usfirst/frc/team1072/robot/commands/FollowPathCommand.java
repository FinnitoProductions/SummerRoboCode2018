package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Angle;
import org.usfirst.frc.team1072.util.Angle.AngleUnit;
import org.usfirst.frc.team1072.util.Position;
import org.usfirst.frc.team1072.util.Position.PositionUnit;
import org.usfirst.frc.team1072.util.Speed;
import org.usfirst.frc.team1072.util.Speed.SpeedUnit;
import org.usfirst.frc.team1072.util.Time;
import org.usfirst.frc.team1072.util.Time.TimeUnit;

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
        this.outerPort = outerPort;
        
    }
    
    public void initialize()
    {

            
        
        
        pathState = 0;
        totalTime = 0;
        double period = new Time(TimeUnit.MILLISECONDS, RobotMap.TIME_PER_TRAJECTORY_POINT_MS).getSeconds() / 2;

        notif = new Notifier(p);
        notif.startPeriodic(period);
        
        Robot.dt.getLeftTalon().setSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(DrivetrainConstants.RIGHT_TALON_PHASE);
        
        if (outerPort == -1) // no auxiliary/arc
        {
            Robot.dt.selectProfileSlots(DrivetrainConstants.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
        }
        else
        {
            Robot.dt.getLeftTalon().follow(Robot.dt.getRightTalon(), FollowerType.AuxOutput1);
            
            System.out.println("SELECTING PROFILE SLOTS");
            
            
            Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
            Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.ANGLE_PID, RobotMap.AUXILIARY_PID_INDEX);
            
            Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                    RemoteSensorSource.Pigeon_Yaw, 
                    RobotMap.REMOTE_SLOT_0, 
                    RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getLeftTalon().getDeviceID(), 
                    RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_SLOT_1, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
            
            Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0/*PigeonConstants.REMOTE_SENSOR_SLOT* TEMPORARY*/, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(1,
                    RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT); // set to average
            
            Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            //Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        }
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
                IMotorController controller = Robot.dt.getRightTalon();
                controller.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
                loadTrajectoryToTalon(getControllerTrajectory(controller), controller);

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
                tp.position = new Position(PositionUnit.FEET, segs[i].position, DrivetrainConstants.WHEELDIAMETER).getEncoderUnits(); // convert revolutions to encoder units
                //System.out.println()
                tp.velocity = new Speed(SpeedUnit.FEET_PER_SECOND, segs[i].velocity, DrivetrainConstants.WHEELDIAMETER).getEncoderUnits(); // convert fps to encoder units
                
                tp.timeDur = TrajectoryDuration.valueOf(RobotMap.TIME_PER_TRAJECTORY_POINT_MS); // convert to correct units
                tp.profileSlotSelect0 = DrivetrainConstants.MOTION_PROFILE_PID;
                
                if (outerPort >= 0) 
                {
                    tp.profileSlotSelect1 = outerPort;
                    tp.auxiliaryPos = new Angle(AngleUnit.RADIANS, segs[i].heading - 2 * Math.PI).getPigeonUnits();
                    tp.position = tp.position + new Position(PositionUnit.FEET, getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].position, RobotMap.DrivetrainConstants.WHEELDIAMETER).getEncoderUnits();
                }
                tp.headingDeg = 0; // convert radians to degrees
                tp.zeroPos = false;

                if (i == (segs.length-1))
                    tp.isLastPoint = true;
                if (i == 0)
                {
                    System.out.println("POSITION: " + tp.position);
                    System.out.println("VELOCITY " + tp.velocity);
                    System.out.println("AUXILIARY POS: " + tp.auxiliaryPos);
                    System.out.println("TIME: " + tp.timeDur);
                    System.out.println("INNER PORT: " + tp.profileSlotSelect0);
                    System.out.println("OUTER PORT: " + tp.profileSlotSelect1);
                    System.out.println("SEGMENT TIME: " + segs[i].dt);
                }
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
            Robot.dt.getRightTalon().set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
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