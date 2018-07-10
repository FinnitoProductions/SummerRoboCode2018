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
import org.usfirst.frc.team1072.util.ConversionFactors;
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
import edu.wpi.first.wpilibj.Timer;
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
    private final int minPointsInController = 13;
    private ProcessBuffer p;
    private Notifier notif;
  
    // store trajectory at index 0, status at index 1
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private Map<IMotorController, Object[]> controllers;
    
    private int pathState;
    

    private int outerPort;
    
    
    private double totalTime;
    
    /**
     * Constructs a new command. 
     */
    public FollowPathCommand()
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        outerPort = -1;
        requires(Robot.dt);
    }
    
    /**
     * Constructs a new command.
     * @param outerPort the outer PID slot
     */
    public FollowPathCommand(int outerPort)
    {
        p = new ProcessBuffer();
        controllers = new HashMap<IMotorController, Object[]>();
        this.outerPort = outerPort;
        requires(Robot.dt);
        
    }
    
    /**
     * Initializes the command (called each time the command is started) by setting up sensors and the notifier.
     */
    public void initialize()
    {
        System.out.println("INITIALIZED FIRST CALLED " + Robot.getCurrentTimeMs());
        pathState = 0;
        totalTime = 0;
        double period = new Time(TimeUnit.MILLISECONDS, RobotMap.TIME_PER_TRAJECTORY_POINT_MS).getSeconds() / 2;

        notif = new Notifier(p);
        notif.startPeriodic(period);
        System.out.println("NOTIFIER STARTED " + Robot.getCurrentTimeMs());
        
        Robot.dt.getLeftTalon().setSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(DrivetrainConstants.RIGHT_TALON_PHASE);
        
        if (outerPort == -1) // no auxiliary/arc
        {
            Robot.dt.selectProfileSlots(DrivetrainConstants.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
        }
        else
        {
            System.out.println("CONFIGURING TALONS " + Robot.getCurrentTimeMs());
            Robot.dt.getLeftTalon().follow(Robot.dt.getRightTalon(), FollowerType.AuxOutput1);
            

            
            
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
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(PigeonConstants.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(0.5,
                    RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT); // set to average
            
            Robot.dt.getRightTalon().set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
            
            Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            

            Robot.dt.getRightTalon().configAllowableClosedloopError(DrivetrainConstants.ANGLE_PID, PigeonConstants.ANGLE_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            
            Robot.dt.getLeftTalon().configAllowableClosedloopError(DrivetrainConstants.MOTION_PROFILE_PID, DrivetrainConstants.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configAllowableClosedloopError(DrivetrainConstants.MOTION_PROFILE_PID, DrivetrainConstants.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            //Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            System.out.println("TALONS CONFIGURED " + Robot.getCurrentTimeMs());
        }
    }
    
    /**
     * Executes the command periodically after being started.
     */
    public void execute()
    {
        SmartDashboard.putNumber("Right Talon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("Right Talon Setpoint", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX) + Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
        if (getControllerStatus(Robot.dt.getRightTalon()) != null)
                SmartDashboard.putNumber("Right Talon Points", getControllerStatus(Robot.dt.getRightTalon()).btmBufferCnt);
        SmartDashboard.putNumber("Right Talon Position Status", Robot.dt.getRightTalon().getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Talon Output Percent", Robot.dt.getRightTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("Right Talon Pigeon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.AUXILIARY_PID_INDEX));

        System.out.println("STARTING EXECUTE " + Robot.getCurrentTimeMs());
        MotionProfileStatus motionStatus = new MotionProfileStatus();
        IMotorController imc = Robot.dt.getRightTalon();
        imc.getMotionProfileStatus(motionStatus);
        controllers.get(imc)[STAT_INDEX] = motionStatus;
        System.out.println("STATUS SET " + Robot.getCurrentTimeMs());
        switch(pathState)
        {
            // ready to begin loading trajectories
            case 0:
            {
                IMotorController controller = Robot.dt.getRightTalon();
                controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
                System.out.println("TRAJECTORY LOADED " + Robot.getCurrentTimeMs());
                loadTrajectoryToTalon(getControllerTrajectory(controller), controller);

                pathState = 1;
                break;
            }
            
            // ready to begin 
            case 1:
            {
                // once enough points have been buffered, begin sequence
                boolean allReady = getControllerStatus(Robot.dt.getRightTalon()).btmBufferCnt > minPointsInController;

                if (allReady)
                {
                  
                    IMotorController controller = Robot.dt.getRightTalon();
                        
                    System.out.println("ENABLING PROFILE " + 1000 * (Timer.getFPGATimestamp() - Robot.startTime));
                    controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
                        
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
                //System.out.println("Buffer Count: " + status.btmBufferCnt);
                boolean isFinished = status.btmBufferCnt == 0;    
                
                if (isFinished) 
                {
                    System.out.println("FINISHED");
                    pathState = 3;
                }
                break;
            }
            case 3:
            {
                Robot.dt.getRightTalon().set(ControlMode.MotionProfileArc, SetValueMotionProfile.Hold.value);
            }
            break;
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
        System.out.println("ADDING PROFILE " + Robot.getCurrentTimeMs());
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 2));
        if (reversePath)
            t = reverseTrajectory(t);
        controllers.put(controller, new Object[] {t, null, new Double(0)});
    }
    
    /**
     * Loads a given set of trajectory points to a controller.
     * @param t the trajectory to be loaded
     * @param controller the controller onto which the points should be loaded
     * @throws InterruptedException 
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
            controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
            System.out.println("STARTING LOAD TRAJECTORY " + Robot.getCurrentTimeMs());
            for (int i = 0; i < segs.length; i++)
            {
                TrajectoryPoint tp = new TrajectoryPoint();
                tp.position = segs[i].position * ConversionFactors.INCHES_PER_FOOT // convert to inches
                        / (DrivetrainConstants.WHEELDIAMETER * Math.PI) // convert to revolutions
                        * ConversionFactors.TICKS_PER_REV;; // convert revolutions to encoder units
                //System.out.println()
                tp.velocity = segs[i].velocity / 10.0 // convert to feet per 100 ms
                        * ConversionFactors.INCHES_PER_FOOT // convert to inches per 100 ms
                        / (DrivetrainConstants.WHEELDIAMETER * Math.PI) // convert to revolutions per 100ms
                        * ConversionFactors.TICKS_PER_REV; // convert to ticks per 100ms
                        // convert fps to encoder units


                
                tp.timeDur = TrajectoryDuration.valueOf(0); // convert to correct units
                tp.profileSlotSelect0 = DrivetrainConstants.MOTION_PROFILE_PID;
                
                if (outerPort >= 0) 
                {
                    tp.profileSlotSelect1 = outerPort;
                    tp.auxiliaryPos = segs[i].heading * ConversionFactors.PIGEON_UNITS_PER_ROTATION/ConversionFactors.RADIANS_PER_ROTATION;
                    tp.position = (tp.position + 
                            (getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].position * ConversionFactors.INCHES_PER_FOOT // convert to inches
                            / (RobotMap.DrivetrainConstants.WHEELDIAMETER* Math.PI) // convert to revolutions
                            * ConversionFactors.TICKS_PER_REV))/2;
                    tp.velocity = (tp.velocity + (getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].velocity / 10.0 // convert to feet per 100 ms
                            * ConversionFactors.INCHES_PER_FOOT // convert to inches per 100 ms
                            / (DrivetrainConstants.WHEELDIAMETER * Math.PI) // convert to revolutions per 100ms
                            * ConversionFactors.TICKS_PER_REV))/2; // convert to ticks per 100ms)
 
                    SmartDashboard.putNumber("Right Talon Traj Position", tp.position);
                }
                //tp.headingDeg = new Angle(AngleUnit.RADIANS, segs[i].heading).getDegrees(); // convert radians to degrees
                tp.zeroPos = false;

                //System.out.println("AUXILIARY POSITION " + tp.auxiliaryPos);
                if (i == (segs.length-1))
                    tp.isLastPoint = true;
                if (i == 0)
                {
                    //System.out.println("POSITION: " + tp.position);


                    //System.out.println("TIME: " + tp.timeDur);
                    //System.out.println("INNER PORT: " + tp.profileSlotSelect0);
                    //System.out.println("OUTER PORT: " + tp.profileSlotSelect1);

                }
                controller.pushMotionProfileTrajectory(tp); // push point to talon
            }
        }
        else
        {
            System.out.println("TRAJECTORY WAS NULL!");
        }
        System.out.println("DONE LOADING TRAJECTORY " + Robot.getCurrentTimeMs());
    }
    
    /**
     * A class to periodically buffer the Talon points.
     * @author Finn Frankis
     * @version 6/20/18
     */
    class ProcessBuffer implements java.lang.Runnable {       
        public void run()
        {
            Robot.dt.getRightTalon().processMotionProfileBuffer();
        }
        
    }

    @Override
    /**
     * Determines whether the command has completed operation.
     */
    protected boolean isFinished()
    {
        return false; //pathState == 3;
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

        Robot.dt.getRightTalon().set(ControlMode.MotionProfileArc, SetValueMotionProfile.Hold.value);
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