package org.usfirst.frc.team1072.robot.commands.auton;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.Pigeon;
import harkerrobolib.util.Conversions;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
//import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * A command to follow a unidirectional motion profile.
 * @author Finn Frankis
 * @version 6/20/18
 */
public class FollowPath extends Command
{
    /**
     * The minimum required points to be buffered on the Talon level before the path can begin.
     */
    private final int minPointsInController = 18;
    
    /**
     * The runnable command which sends points from the RoboRio buffer to the Talon buffer.
     */
    private ProcessBuffer p;
    
    /**
     * The notifier calling the ProcessBuffer periodically.
     */
    private Notifier notif;
  
    /**
     * The index of the trajectory in the controllers array.
     */
    private final int TRAJ_INDEX = 0;
    
    /**
     * The index of the motion profile status in the controllers array.
     */
    private final int STAT_INDEX = 1;
    
    /**
     * The index of the boolean determing whether the trajectory has been loaded in the controllers array.
     */
    private static final int TRAJ_LOADED_INDEX = 2;
    
    /**
     * The map containing all controllers used in this path and pointing to the relevant objects
     * mentioned above.
     */
    private Map<IMotorController, Object[]> controllers;
    
    /**
     * The current state of this path.
     */
    private int pathState;
    
    /**
     * The outer PID slot for this profile, if it is an arc.
     */
    private int outerPort;
    
    /**
     * The total time which this path would be expected to take, in an ideal case.
     */
    private double totalTime;
    
    /**
     * Whether or not the auxiliary sensor should be zeroed at initialization.
     */
    private boolean zeroAux;
    
    /**
     * Whether or not the sensors should be reconfigured at initializtion.
     */
    private boolean resetSensors;
    
    /**
     * Constructs a new command. 
     */
    public FollowPath()
    {
        p = new ProcessBuffer();
        notif = new Notifier(p);
        controllers = new HashMap<IMotorController, Object[]>();
        outerPort = -1;
        requires(Robot.dt);
        totalTime = -1;
    }
    
    /**
     * Constructs a new command.
     * @param outerPort the outer PID slot
     */
    public FollowPath(int outerPort)
    {
        p = new ProcessBuffer();
        notif = new Notifier(p);
        controllers = new HashMap<IMotorController, Object[]>();
        this.outerPort = outerPort;
        resetSensors = true;
        requires(Robot.dt);
        totalTime = -1;
    }
    
    /**
     * Determines whether the pigeon should be zeroed at the start of the path.
     * @param zeroPigeon whether or not to zero the pigeon
     * @return this command, to allow the command to be called at initialization 
     * (ex: FollowPath fp = new FollowPath().zeroPigeonAtStart(true);)
     */
    public FollowPath zeroPigeonAtStart(boolean zeroPigeon)
    {
        zeroAux = zeroPigeon;
        return this;
    }
    
    /**
     * Determines whether the sensors should be reset when the path begins.
     * @param zeroSensors whether or not to reset the sensors
     * @return this command, to allow the command to be called at initialization 
     * (ex: FollowPath fp = new FollowPath().zeroPigeonAtStart(true);)
     */
    public FollowPath resetSensors (boolean zeroSensors)
    {
        resetSensors = zeroSensors;
        return this;
    }
    
    /**
     * Sets the total time of the path (to be used if externally calculated).
     * @param newTotalTime the value with which tita
     */
    public void setTotalTime (double newTotalTime)
    {
        totalTime = newTotalTime;
    }
    
    /**
     * Gets the total time which the path is expected to take.
     * @return the total time of the path in seconds
     */
    public double getTotalTime()
    {
        return totalTime;
    }
    
    /**
     * Initializes the command (called each time the command is started) by setting up sensors and the notifier.
     */
    public void initialize()
    {
        ;
        ;
        pathState = 0;
        totalTime = 0;
        double period = RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 1000 / 2;

        notif.startPeriodic(period);
        ;
        
        Robot.dt.setTalonSensorPhase(Drivetrain.LEFT_TALON_PHASE, 
                Drivetrain.RIGHT_TALON_PHASE);
        
        Robot.dt.configureMotionProfileDriveClosedLoop();
        if (outerPort == -1) // no auxiliary/arc
        {
            Robot.dt.selectProfileSlots(Drivetrain.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
        }
        else
        {
            ;
            //;
            
            Robot.dt.configureMotionProfileAngleClosedLoop();
            if (resetSensors)
            {
                Robot.dt.getLeftMaster().follow(Robot.dt.getRightMaster(), FollowerType.AuxOutput1);
                Robot.dt.getLeftMaster().configAuxPIDPolarity(false);
                
                Robot.dt.getRightMaster().selectProfileSlot(Drivetrain.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
                Robot.dt.getRightMaster().selectProfileSlot(Drivetrain.ANGLE_PID, RobotMap.AUXILIARY_PID_INDEX);
                
                Robot.dt.getRightMaster().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                        RemoteSensorSource.Pigeon_Yaw, 
                        RobotMap.REMOTE_SLOT_0, 
                        RobotMap.TIMEOUT);
                
                Robot.dt.getRightMaster().configRemoteFeedbackFilter(Robot.dt.getLeftMaster().getDeviceID(), 
                        RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_SLOT_1, RobotMap.TIMEOUT);
                
    
                Robot.dt.getRightMaster().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
                Robot.dt.getRightMaster().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
                
                Robot.dt.getLeftMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    
                Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                
                Robot.dt.getRightMaster().configSelectedFeedbackSensor(Drivetrain.Pigeon.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
                
                ;
                if (zeroAux)
                {
                    Robot.dt.getPigeon().setYaw(0);
                }
                //;
                Robot.dt.getRightMaster().configSelectedFeedbackCoefficient(0.5,
                        RobotMap.PRIMARY_PID_INDEX); // set to average
                
    
                Robot.dt.getRightMaster().configAllowableClosedloopError(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.ANGLE_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
                
                Robot.dt.getLeftMaster().configAllowableClosedloopError(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROFILE_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
                Robot.dt.getRightMaster().configAllowableClosedloopError(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROFILE_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            }
            ;
        }
    }
    
    /**
     * Executes the command periodically after being started.
     */
    public void execute()
    {
        //;
        MotionProfileStatus motionStatus = new MotionProfileStatus();
        IMotorController imc = Robot.dt.getRightMaster();
        imc.getMotionProfileStatus(motionStatus);
        controllers.get(imc)[STAT_INDEX] = motionStatus;
        //;
        SmartDashboard.putNumber("Left Encoder - Right Encoder", Robot.dt.getLeftMaster().getSelectedSensorPosition(0) - Robot.dt.getRightMaster().getSelectedSensorPosition(0));
        switch(pathState)
        {
            // ready to begin loading trajectories
            case 0:
            {
                IMotorController controller = Robot.dt.getRightMaster();
                controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
                ;
                if (!getControllerTrajectoryLoaded(controller))
                {
                    loadTrajectoryToTalon(getControllerTrajectory(controller), controller);
                }
                
                pathState = 1;
                break;
            }
            
            // ready to begin 
            case 1:
            {
                // once enough points have been buffered, begin sequence
                boolean allReady = getControllerStatus(Robot.dt.getRightMaster()).btmBufferCnt > minPointsInController;
                //;
                if (allReady)
                {
                  
                    IMotorController controller = Robot.dt.getRightMaster();
                        
                    ;
                    controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
                    
                    Robot.dt.getLeftMaster().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);
                    
                    Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    Robot.dt.getRightMaster().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);
                    
                    Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    Robot.dt.getRightMaster().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);

                    ;
                    controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
                        
                    pathState = 2;
                    ;
                }
                break;
            }
            
            // check up on profile to see if done
            case 2:
            {
                
                IMotorController controller = Robot.dt.getRightMaster();
                MotionProfileStatus status = new MotionProfileStatus();
                controller.getMotionProfileStatus(status); 

                controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
                //;
                boolean isFinished = status.isLast == true;
                
                if (isFinished) 
                {
                    ;
                    pathState = 3;
                }
                break;
            }
            case 3:
            {
                Robot.dt.getRightMaster().set(ControlMode.MotionProfileArc, SetValueMotionProfile.Hold.value);
                ;
            }
            break;
        }
    }
    
    /**
     * Adds a controller and its trajectory to the map.
     * @param t the trajectory which this controller should follow
     * @param controller the controller which is being added
     * @param reversePath true if the path should be formed in reverse order; false otherwise
     * @param endAngle the angle at which this trajectory ends
     */
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath, double endAngle)
    {
        ;
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 2));
        if (reversePath)
            t = reverseTrajectory(t);
        t = addAngleOffset(t, endAngle);
        controllers.put(controller, new Object[] {t, null, false});
        ;
    }
    
    /**
     * Adds an offset angle to a given trajectory based on a previous path.
     * @param t the trajectory which the offset will be added to
     * @param endAngle the final angle of the previous path (to offset this one)
     * @return the trajectory, after the offset has been added
     */
    private Trajectory addAngleOffset(Trajectory t, double endAngle)
    {
        Segment[] segs = t.segments;
        double factorToAdd = endAngle - t.segments[0].heading;
        ;
        for (int i = 0; i < segs.length; i++)
        {
            segs[i].heading += factorToAdd;
        }
        ;
        return t; 
    }

    /**
     * Loads a given set of trajectory points to a controller.
     * @param t the trajectory to be loaded
     * @param controller the controller onto which the points should be loaded
     */
    public void loadTrajectoryToTalon(Trajectory t, IMotorController controller)
    {
        if (t != null)
        {
            if (getControllerStatus(controller) != null && getControllerStatus(controller).isUnderrun)
            {
                controller.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
                ;
            }
            
            if (!getControllerTrajectoryLoaded(controller))
            {
                controller.configMotionProfileTrajectoryPeriod(RobotMap.TIME_PER_TRAJECTORY_POINT_MS, RobotMap.TIMEOUT);
                // constructs Talon-readable trajectory points out of each segment
                Segment[] segs = t.segments;
    
                ;
                double velocityAddFactor = Drivetrain.MOT_PROF_ADD_TO_VEL_INIT;
                for (int i = 0; i < segs.length; i++)
                {
                    TrajectoryPoint tp = new TrajectoryPoint();
                    tp.position = segs[i].position * Conversions.INCHES_PER_FOOT // convert to inches
                            / (Drivetrain.WHEELDIAMETER * Math.PI) // convert to revolutions
                            * Conversions.TICKS_PER_REV;; // convert revolutions to encoder units
                    //
                    tp.velocity = segs[i].velocity;// convert to ticks per 100ms
                            // convert fps to encoder units
    
    
                    
                    //tp.timeDur = TrajectoryDuration.valueOf(0); // time to ADD to each point convert to correct units
                    tp.profileSlotSelect0 = Drivetrain.MOTION_PROFILE_PID;
                    
                    if (outerPort >= 0) 
                    {
                        tp.profileSlotSelect1 = outerPort;
                        tp.auxiliaryPos = segs[i].heading * Conversions.PIGEON_UNITS_PER_ROTATION/Conversions.RADIANS_PER_ROTATION;
                        tp.position = (tp.position + 
                                (getControllerTrajectory(Robot.dt.getLeftMaster()).segments[i].position * Conversions.INCHES_PER_FOOT // convert to inches
                                / (Drivetrain.WHEELDIAMETER* Math.PI) // convert to revolutions
                                * Conversions.TICKS_PER_REV))/2;
                        tp.velocity = (tp.velocity + (getControllerTrajectory(Robot.dt.getLeftMaster()).segments[i].velocity)) / 2; // convert to ticks per 100ms)
    
    
                    }
                    if (segs.length > 1 && i == 0 && tp.velocity == 0)
                    {
                        tp.velocity += Math.signum(segs[i+1].velocity) * velocityAddFactor;
                    }
                    else
                    {
                        tp.velocity += Math.signum(tp.velocity) * velocityAddFactor;
                    }
                    //ramp down factor to add to velocity after each point
                    velocityAddFactor -= 
                            Drivetrain.MOT_PROF_ADD_TO_VEL_INIT 
                            * (RobotMap.TIME_PER_TRAJECTORY_POINT_MS / Drivetrain.TIME_TO_OVERCOME_S_FRICTION_MS);
                    velocityAddFactor = Math.max(0, velocityAddFactor);
                    //tp.headingDeg = new Angle(AngleUnit.RADIANS, segs[i].heading).getDegrees(); // convert radians to degrees
                    tp.velocity = tp.velocity / 10.0 // convert to feet per 100 ms
                    * Conversions.INCHES_PER_FOOT // convert to inches per 100 ms
                    / (Drivetrain.WHEELDIAMETER * Math.PI) // convert to revolutions per 100ms
                    * Conversions.TICKS_PER_REV; 
                    tp.zeroPos = false;
    
                    //;
                    /*System.out.print("v: " + 
                    Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, tp.velocity, SpeedUnit.FEET_PER_SECOND)
                    + ", pos: " + 
                    Conversions.convertPosition(PositionUnit.ENCODER_UNITS, tp.position, PositionUnit.FEET) + " ");*/
                    if (i == 0)
                        ;
                    if (i == (segs.length-1))
                    {
                        tp.isLastPoint = true;
                        ;
                    }
                    
                    controller.pushMotionProfileTrajectory(tp); // push point to talon
                }
                setControllerTrajectoryLoaded(controller, true);
            }
            else
                ;
        }
        else
        {
            ;
        }
        ;
    }
    
    /**
     * A class to periodically buffer the Talon points.
     * @author Finn Frankis
     * @version 6/20/18
     */
    class ProcessBuffer implements java.lang.Runnable {       
        public void run()
        {
            Robot.dt.getRightMaster().processMotionProfileBuffer();
        }
        
    }

    @Override
    /**
     * Determines whether the command has completed operation.
     */
    protected boolean isFinished()
    {
        boolean isFinished = pathState == 3 &&
                Math.abs(Robot.dt.getRightMaster().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < Drivetrain.MOTION_PROFILE_ALLOWABLE_ERROR;
        return isFinished;
    }
    
    @Override
    /**
     * To be called if the command is forcibly cancelled by the user.
     */
    public void cancel()
    {
        disable();
    }
    
    @Override
    /**
     * To be called when the command ends peacefully (isFinished returns true).
     */
    protected void end()
    {
        ;
        /*Robot.dt.getRightTalon().setSelectedSensorPosition(
                -(int)(Conversions.convertPosition(PositionUnit.FEET, getControllerTrajectory(Robot.dt.getRightTalon()).segments[
                getControllerTrajectory(Robot.dt.getRightTalon()).segments.length-1].position, PositionUnit.ENCODER_UNITS)
                        - Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)),
                RobotMap.PRIMARY_PID_INDEX, 
                RobotMap.TIMEOUT);*/
        ;
        
        System.out.println("MOT PROF IS OVER!");
        disable();
        
    }
    
    @Override
    /**
     * To be called when the command is interrupted by another one in the scheduler requiring similar subsystems.
     */
    protected void interrupted()
    {
        disable();
    }
    
    /**
     * To be called when the command is stopping (regardless of why it is forced to stop).
     */
    public void disable() {
        ;
        notif.stop();

        Robot.dt.getRightMaster().set(ControlMode.MotionProfileArc, SetValueMotionProfile.Hold.value);
        Robot.dt.getRightMaster().clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
    }
    
    /**
     * Gets the controller status of a given controller in the map.
     * @param controller the controller for which the status will be retrieved
     * @return the status of this controller
     */
    private MotionProfileStatus getControllerStatus (IMotorController controller)
    {
        return (MotionProfileStatus) controllers.get(controller)[STAT_INDEX];
    }
    
    /**
     * Gets the trajectory of a given controller in the map.
     * @param controller the controller for which the status will be retrieved
     * @return the trajectory of this controller
     */
    public Trajectory getControllerTrajectory (IMotorController controller)
    {
        return (Trajectory) controllers.get(controller)[TRAJ_INDEX];
    }
    
    /**
     * Determines whether a given controller's trajectory has been successfully loaded. For use 
     * when prebuffering may or may not occur on time.
     * @param controller the controller to be checked
     * @return true if the controller trajectory has been loaded, false otherwise
     */
    public boolean getControllerTrajectoryLoaded (IMotorController controller)
    {
        return (boolean) controllers.get(controller)[TRAJ_LOADED_INDEX];
    }
    
    /**
     * Sets whether a given controller's trajectory has been successfully loaded.
     * @param controller the controller for which the trajectory has or hasn't been loaded
     * @param value the value to set the trajectory 
     */
    public void setControllerTrajectoryLoaded (IMotorController controller, boolean value)
    {
        controllers.get(controller)[TRAJ_LOADED_INDEX] = value;
    }
    
    /**
     * Reverses a given trajectory (assists in creating backward paths).
     * @param t the trajectory to reverse 
     * @return the reversed trajectory
     */
    private Trajectory reverseTrajectory(Trajectory t)
    {   
        ;
        for (Segment s : t.segments)
        {
            s.velocity *= -1;
            s.position -= t.segments[t.segments.length - 1].position;
        }
        List<Segment> list = Arrays.asList(t.segments);
        Collections.reverse(list);
        
        t.segments = (Segment[]) list.toArray();
        
        ;
        return t;

    }
    
    /**
     * Determines whether the path has been enabled and all initialization has been completed.
     * @return true if setup is complete; false otherwise
     */
    public boolean isSetupComplete()
    {
        return pathState >= 2;
    }
         
    /**
     * Wrapper method for isFinished() to allow it to be publicly accessible.
     * @return true if the command has finished; false otherwise
     */
    public boolean getFinished()
    {
        return isFinished();
    }
    
 

    /**
     * Returns the current state of this path.
     * @return the current state of this path (0 if just initialized, 1 if still buffering points, 2 if 
     * in execution mode, and 3 if the path is complete)
     */
    public int getPathState()
    {
        // TODO Auto-generated method stub
        return pathState;
    }
}