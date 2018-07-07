package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
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
 * A command to follow a unidirectional motion profile.
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
    private Set<IMotorController> masterControllers;
    
    private int pathState;
    

    private int outerPort;
    private ControlMode controlMode;
    
    /**
     * Constructs a new command. 
     */
    public FollowPathCommand()
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        masterControllers = new HashSet<IMotorController>();
        
        outerPort = -1;
        controlMode = ControlMode.MotionProfile;
    }
    
    /**
     * Constructs a new command.
     * @param outerPort the outer PID slot
     */
    public FollowPathCommand(int outerPort)
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        masterControllers = new HashSet<IMotorController>();
        this.outerPort = outerPort;
        controlMode = ControlMode.MotionProfileArc;
    }
    
    /**
     * Initializes the command (called each time the command is started) by setting up sensors and the notifier.
     */
    public void initialize()
    {
        pathState = 0;
        
        masterControllers.add(Robot.dt.getRightTalon());
        if (controlMode.equals(ControlMode.MotionProfile))
            masterControllers.add(Robot.dt.getLeftTalon());
        
        double period = new Time(TimeUnit.MILLISECONDS, RobotMap.TIME_PER_TRAJECTORY_POINT_MS).getSeconds() / 2;

        notif = new Notifier(p);
        notif.startPeriodic(period);
        
        Robot.dt.getLeftTalon().setSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(DrivetrainConstants.RIGHT_TALON_PHASE);
        
        if (controlMode.equals(ControlMode.MotionProfile)) // no auxiliary/arc
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
            System.out.println("SENSOR SUM");
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(PigeonConstants.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(0.5,
                    RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT); // set to average
            
            Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            //Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            SmartDashboard.putNumber("Right Talon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        }
    }
    
    /**
     * Executes the command periodically after being started.
     */
    public void execute()
    {
        SmartDashboard.putNumber("Right Talon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        MotionProfileStatus motionStatus = new MotionProfileStatus();
        for (IMotorController imc : masterControllers)
        {
            imc.getMotionProfileStatus(motionStatus);
            controllers.get(imc)[STAT_INDEX] = motionStatus;
        }
        switch(pathState)
        {
            // ready to begin loading trajectories
            case 0:
            {
                for (IMotorController controller : masterControllers)
                {
                    controller.set(controlMode, SetValueMotionProfile.Disable.value);
                    loadTrajectoryToTalon(getControllerTrajectory(controller), controller);
                }

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

                    
                    System.out.println("Enabling profile.");
                    for (IMotorController controller : masterControllers)
                    {
                        controller.set(controlMode, SetValueMotionProfile.Enable.value);
                    }
                        
                    pathState = 2;
                }
                break;
            }
            
            // check up on profile to see if done
            case 2:
            {

                IMotorController controller = masterControllers.iterator().next();
                MotionProfileStatus status = new MotionProfileStatus();
                controller.getMotionProfileStatus(status);
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
    
    /**
     * Adds a controller and its trajectory to the map.
     * @param t the trajectory which this controller should follow
     * @param controller the controller which is being added
     * @param reversePath true if the path should be formed in reverse order; false otherwise
     */
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath)
    {
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 2));
        if (reversePath)
            t = reverseTrajectory(t);
        controllers.put(controller, new Object[] {t, null, new Double(0)});        
    }
    
    /**
     * Loads a given set of trajectory points to a controller.
     * @param t the trajectory to be loaded
     * @param controller the controller onto which the points should be loaded
     */
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
            controller.set(controlMode, SetValueMotionProfile.Disable.value);
            for (int i = 0; i < segs.length; i++)
            {
                TrajectoryPoint tp = new TrajectoryPoint();
                tp.position = new Position(PositionUnit.FEET, segs[i].position, DrivetrainConstants.WHEELDIAMETER).getEncoderUnits(); // convert revolutions to encoder units
                //System.out.println()
                tp.velocity = new Speed(SpeedUnit.FEET_PER_SECOND, segs[i].velocity, 
                        DrivetrainConstants.WHEELDIAMETER).getEncoderUnits(); // convert fps to encoder units
                
                tp.timeDur = TrajectoryDuration.valueOf(0); // convert to correct units
                tp.profileSlotSelect0 = DrivetrainConstants.MOTION_PROFILE_PID;
                
                if (controlMode.equals(ControlMode.MotionProfileArc))
                {
                    tp.profileSlotSelect1 = outerPort;
                    tp.auxiliaryPos = new Angle(AngleUnit.RADIANS, segs[i].heading).getPigeonUnits();
                    tp.position = (tp.position + 
                            new Position(PositionUnit.FEET, getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].position, 
                                    RobotMap.DrivetrainConstants.WHEELDIAMETER).getEncoderUnits())/2;
                    tp.velocity = (tp.velocity + new Speed(SpeedUnit.FEET_PER_SECOND,
                            getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].velocity, 
                            DrivetrainConstants.WHEELDIAMETER).getEncoderUnits())/2;
                }
                //tp.headingDeg = new Angle(AngleUnit.RADIANS, segs[i].heading).getDegrees(); // convert radians to degrees
                tp.zeroPos = false;

                System.out.println("AUXILIARY POSITION " + tp.auxiliaryPos);
                if (i == (segs.length-1))
                    tp.isLastPoint = true;
                if (i == 0)
                {
                    System.out.println("POSITION: " + tp.position);


                    System.out.println("TIME: " + tp.timeDur);
                    System.out.println("INNER PORT: " + tp.profileSlotSelect0);
                    System.out.println("OUTER PORT: " + tp.profileSlotSelect1);

                }
                controller.pushMotionProfileTrajectory(tp); // push point to talon
            }
        }
        else
        {
            System.out.println("TRAJECTORY WAS NULL!");
        }
    }
    
    /**
     * A class to periodically buffer the Talon points.
     * @author Finn Frankis
     * @version 6/20/18
     */
    class ProcessBuffer implements java.lang.Runnable {       
        public void run()
        {
            if (controlMode.equals(ControlMode.MotionProfile))
                Robot.dt.getLeftTalon().processMotionProfileBuffer();
            Robot.dt.getRightTalon().processMotionProfileBuffer();
        }
        
    }

    @Override
    /**
     * Determines whether the command has completed operation.
     */
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
    
    /**
     * To be called when the command is either cancelled, interrupted, or ended.
     */
    public void disable() {
        System.out.println("DISABLING");
        notif.stop();

        Robot.dt.getRightTalon().clearMotionProfileTrajectories();
        Robot.dt.getRightTalon().set(controlMode, SetValueMotionProfile.Hold.value);
        Robot.dt.getRightTalon().clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
    }
    
    /**
     * Gets the controller status of a given controller in the map.
     * @param controller the controller for which the status will be retrieved
     * @return the status of this controller
     */
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        return (MotionProfileStatus) controllers.get(controller)[1];//STAT_INDEX];
    }
    
    /**
     * Gets the trajectory of a given controller in the map.
     * @param controller the controller for which the status will be retrieved
     * @return the trajectory of this controller
     */
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