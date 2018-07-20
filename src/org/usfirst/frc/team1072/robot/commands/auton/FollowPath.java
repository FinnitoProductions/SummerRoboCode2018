package org.usfirst.frc.team1072.robot.commands.auton;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.AngleUnit;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;
import org.usfirst.frc.team1072.util.Conversions.SpeedUnit;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
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
    private final int minPointsInController = 18;
    private ProcessBuffer p;
    private Notifier notif;
  
    // store trajectory at index 0, status at index 1, and whether the trajectory has been loaded at index 2
    private final int TRAJ_INDEX = 0;
    private final int STAT_INDEX = 1;
    private static final int TRAJ_LOADED_INDEX = 2;
    private Map<IMotorController, Object[]> controllers;
    
    private int pathState;
    

    private int outerPort;
    
    
    private double totalTime;
    
    private boolean zeroAux;
    
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
        requires(Robot.dt);
        totalTime = -1;
    }
    
    /**
     * Determines whether the pigeon should be zeroed at the start of the path.
     * @param zeroPigeon whether or not to zero the pigeon
     * @return this command, to allow the command to be called at initialization (ex: FollowPath fp = new FollowPath().zeroPigeonAtStart(true);)
     */
    public FollowPath zeroPigeonAtStart(boolean zeroPigeon)
    {
        zeroAux = zeroPigeon;
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
        System.out.println("INITIALIZED" + Robot.getCurrentTimeMs());
        System.out.println("INITIALIZED FIRST CALLED " + Robot.getCurrentTimeMs());
        pathState = 0;
        totalTime = 0;
        double period = RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 1000 / 2;

        notif.startPeriodic(period);
        System.out.println("NOTIFIER STARTED " + Robot.getCurrentTimeMs());
        
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, 
                DrivetrainConstants.RIGHT_TALON_PHASE);
        
        Robot.dt.configureMotionProfileDriveClosedLoop();
        if (outerPort == -1) // no auxiliary/arc
        {
            Robot.dt.selectProfileSlots(DrivetrainConstants.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
        }
        else
        {
            System.out.println("CONFIGURING TALONS " + Robot.getCurrentTimeMs());
            System.out.println("VERY START AUX ANGLE: " + Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));
            Robot.dt.configureMotionProfileAngleClosedLoop();
            Robot.dt.getLeftTalon().follow(Robot.dt.getRightTalon(), FollowerType.AuxOutput1);
            Robot.dt.getLeftTalon().configAuxPIDPolarity(false, RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.MOTION_PROFILE_PID, RobotMap.PRIMARY_PID_INDEX);
            Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.ANGLE_PID, RobotMap.AUXILIARY_PID_INDEX);
            
            Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                    RemoteSensorSource.Pigeon_Yaw, 
                    RobotMap.REMOTE_SLOT_0, 
                    RobotMap.TIMEOUT);
            
            Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getLeftTalon().getDeviceID(), 
                    RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_SLOT_1, RobotMap.TIMEOUT);
            

            
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
            
            Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);

            Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
            
            double prevPigeonValue = 
                    Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX);
            Robot.dt.getRightTalon().configSelectedFeedbackSensor(PigeonConstants.REMOTE_SENSOR_SLOT, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
            if (!zeroAux)
            {
                Robot.dt.addPigeonYaw(prevPigeonValue);
            }
            Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(0.5,
                    RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT); // set to average
            

            Robot.dt.getRightTalon().configAllowableClosedloopError(DrivetrainConstants.ANGLE_PID, PigeonConstants.ANGLE_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            
            Robot.dt.getLeftTalon().configAllowableClosedloopError(DrivetrainConstants.MOTION_PROFILE_PID, DrivetrainConstants.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
            Robot.dt.getRightTalon().configAllowableClosedloopError(DrivetrainConstants.MOTION_PROFILE_PID, DrivetrainConstants.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        
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
        SmartDashboard.putNumber("Right Talon Output Percent", Robot.dt.getRightTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("Left Talon Output Percent", Robot.dt.getLeftTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("Right Talon Pigeon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.AUXILIARY_PID_INDEX));

        //System.out.println("STARTING EXECUTE " + Robot.getCurrentTimeMs());
        MotionProfileStatus motionStatus = new MotionProfileStatus();
        IMotorController imc = Robot.dt.getRightTalon();
        imc.getMotionProfileStatus(motionStatus);
        controllers.get(imc)[STAT_INDEX] = motionStatus;
        //System.out.println("STATUS SET " + Robot.getCurrentTimeMs());
        switch(pathState)
        {
            // ready to begin loading trajectories
            case 0:
            {
                IMotorController controller = Robot.dt.getRightTalon();
                controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
                System.out.println("TRAJECTORY LOADED " + Robot.getCurrentTimeMs());
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
                boolean allReady = getControllerStatus(Robot.dt.getRightTalon()).btmBufferCnt > minPointsInController;
                //System.out.println("Buffer Count: " + getControllerStatus(Robot.dt.getRightTalon()).btmBufferCnt);
                if (allReady)
                {
                  
                    IMotorController controller = Robot.dt.getRightTalon();
                        
                    System.out.println("ENABLING PROFILE " + Robot.getCurrentTimeMs());
                    controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
                    
                    Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    
                    Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    
                    Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                    Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);

                    System.out.println("AUX ANGLE BEFORE ENABLE: " + Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));
                    controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
                        
                    pathState = 2;
                    System.out.println("MOT PROF ENABLED " + Robot.getCurrentTimeMs());
                }
                break;
            }
            
            // check up on profile to see if done
            case 2:
            {
                
                IMotorController controller = Robot.dt.getRightTalon();
                MotionProfileStatus status = new MotionProfileStatus();
                controller.getMotionProfileStatus(status); 

                controller.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
                //System.out.println("Buffer Count: " + status.btmBufferCnt);
                boolean isFinished = status.isLast == true;
                
                if (isFinished) 
                {
                    System.out.println("PATH FINISHED" + Robot.getCurrentTimeMs());
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
    public void addProfile (Trajectory t, IMotorController controller, boolean reversePath, double endAngle)
    {
        System.out.println("ADDING PROFILE " + Robot.getCurrentTimeMs());
        controller.changeMotionControlFramePeriod(Math.max(1, RobotMap.TIME_PER_TRAJECTORY_POINT_MS / 2));
        if (reversePath)
            t = reverseTrajectory(t);
        t = addAngleOffset(t, endAngle);
        controllers.put(controller, new Object[] {t, null, false});
        System.out.println("DONE ADDING PROFILE");
    }
    
    /**
     * @param t
     * @param endAngle
     * @return
     */
    private Trajectory addAngleOffset(Trajectory t, double endAngle)
    {
        Segment[] segs = t.segments;
        double factorToAdd = endAngle - t.segments[0].heading;
        for (int i = 0; i < segs.length; i++)
        {
            segs[i].heading += factorToAdd;
        }
        return t; 
    }

    /**
     * Loads a given set of trajectory points to a controller.
     * @param t the trajectory to be loaded
     * @param controller the controller onto which the points should be loaded
     * @throws InterruptedException 
     */
    public void loadTrajectoryToTalon(Trajectory t, IMotorController controller)
    {
        if (t != null)
        {
            if (getControllerStatus(controller) != null && getControllerStatus(controller).isUnderrun)
            {
                controller.clearMotionProfileHasUnderrun(RobotMap.TIMEOUT);
                System.out.println("IS UNDERRUN");
            }
            
            if (!getControllerTrajectoryLoaded(controller))
            {
                controller.configMotionProfileTrajectoryPeriod(RobotMap.TIME_PER_TRAJECTORY_POINT_MS, RobotMap.TIMEOUT);
                // constructs Talon-readable trajectory points out of each segment
                Segment[] segs = t.segments;
    
                System.out.println("STARTING LOAD TRAJECTORY " + Robot.getCurrentTimeMs());
                double velocityAddFactor = DrivetrainConstants.MOT_PROF_ADD_TO_VEL_INIT;
                for (int i = 0; i < segs.length; i++)
                {
                    TrajectoryPoint tp = new TrajectoryPoint();
                    tp.position = segs[i].position * Conversions.INCHES_PER_FOOT // convert to inches
                            / (DrivetrainConstants.WHEELDIAMETER * Math.PI) // convert to revolutions
                            * Conversions.TICKS_PER_REV;; // convert revolutions to encoder units
                    //System.out.println()
                    tp.velocity = segs[i].velocity;// convert to ticks per 100ms
                            // convert fps to encoder units
    
    
                    
                    tp.timeDur = TrajectoryDuration.valueOf(0); // time to ADD to each point convert to correct units
                    tp.profileSlotSelect0 = DrivetrainConstants.MOTION_PROFILE_PID;
                    
                    if (outerPort >= 0) 
                    {
                        tp.profileSlotSelect1 = outerPort;
                        tp.auxiliaryPos = segs[i].heading * Conversions.PIGEON_UNITS_PER_ROTATION/Conversions.RADIANS_PER_ROTATION;
                        tp.position = (tp.position + 
                                (getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].position * Conversions.INCHES_PER_FOOT // convert to inches
                                / (RobotMap.DrivetrainConstants.WHEELDIAMETER* Math.PI) // convert to revolutions
                                * Conversions.TICKS_PER_REV))/2;
                        tp.velocity = (tp.velocity + (getControllerTrajectory(Robot.dt.getLeftTalon()).segments[i].velocity)) / 2; // convert to ticks per 100ms)
    
    
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
                            DrivetrainConstants.MOT_PROF_ADD_TO_VEL_INIT 
                            * (RobotMap.TIME_PER_TRAJECTORY_POINT_MS / DrivetrainConstants.TIME_TO_OVERCOME_S_FRICTION_MS);
                    velocityAddFactor = Math.max(0, velocityAddFactor);
                    //tp.headingDeg = new Angle(AngleUnit.RADIANS, segs[i].heading).getDegrees(); // convert radians to degrees
                    tp.velocity = tp.velocity / 10.0 // convert to feet per 100 ms
                    * Conversions.INCHES_PER_FOOT // convert to inches per 100 ms
                    / (DrivetrainConstants.WHEELDIAMETER * Math.PI) // convert to revolutions per 100ms
                    * Conversions.TICKS_PER_REV; 
                    tp.zeroPos = false;
    
                    //System.out.println("AUXILIARY POSITION " + tp.auxiliaryPos);
                    /*System.out.print("v: " + 
                    Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, tp.velocity, SpeedUnit.FEET_PER_SECOND)
                    + ", pos: " + 
                    Conversions.convertPosition(PositionUnit.ENCODER_UNITS, tp.position, PositionUnit.FEET) + " ");*/
                    if (i == 0)
                        System.out.println("START ANGLE: " + tp.auxiliaryPos);
                    if (i == (segs.length-1))
                    {
                        tp.isLastPoint = true;
                        System.out.println("END ANGLE: " + tp.auxiliaryPos);
                    }
    
                    controller.pushMotionProfileTrajectory(tp); // push point to talon
                }
                setControllerTrajectoryLoaded(controller, true);
            }
            else
                System.out.println("The trajectory has already been loaded.");
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
        boolean isFinished = getControllerStatus(Robot.dt.getRightTalon()).isLast &&
                Math.abs(Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < RobotMap.MOTION_PROFILE_END_ERROR;
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
        System.out.println(this + " finished " + Robot.getCurrentTimeMs());
        /*Robot.dt.getRightTalon().setSelectedSensorPosition(
                -(int)(Conversions.convertPosition(PositionUnit.FEET, getControllerTrajectory(Robot.dt.getRightTalon()).segments[
                getControllerTrajectory(Robot.dt.getRightTalon()).segments.length-1].position, PositionUnit.ENCODER_UNITS)
                        - Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)),
                RobotMap.PRIMARY_PID_INDEX, 
                RobotMap.TIMEOUT);*/
        System.out.println("END ANGLE: " + Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.AUXILIARY_PID_INDEX));
        
        
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
    
    public boolean getControllerTrajectoryLoaded (IMotorController controller)
    {
        return (boolean) controllers.get(controller)[TRAJ_LOADED_INDEX];
    }
    
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
        System.out.println("REVERSING TRAJ");
        for (Segment s : t.segments)
        {
            s.velocity *= -1;
            s.position -= t.segments[t.segments.length - 1].position;
        }
        List<Segment> list = Arrays.asList(t.segments);
        Collections.reverse(list);
        
        t.segments = (Segment[]) list.toArray();
        
        System.out.println("TRAJ REVERSED");
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