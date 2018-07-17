package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.AngleUnit;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Combines position and angle for a smoother PID-based autonomous.
 * @author Finn Frankis
 * @version Jul 14, 2018
 */
public class CombinedPositionAnglePID extends Command
{
    private double position, angle, numExecutes, maxExecutes = 3;
    private boolean isPosition, isAngle;
    
    /**
     * Constructs a new CombinedPositionAnglePID.
     * @param position the final position for the robot in feet
     * @param angle the final angle for the robot in degrees
     */
    public CombinedPositionAnglePID (double position, double angle)
    {
        this.position = Conversions.convertPosition(PositionUnit.FEET, position, PositionUnit.ENCODER_UNITS);
        this.angle = Conversions.convertAngle(AngleUnit.DEGREES, angle, AngleUnit.PIGEON_UNITS);
    }
    
    /**
     * Initializes the command.
     */
    public void initialize()
    {
        isPosition = true;
        isAngle = false;
        initPosition();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        
        numExecutes = 0;
    }
    
    /**
     * Initializes the position part of this command.
     */
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setBothSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    }
    
    /**
     * Initializes the angle part of this command.
     */
    private void initAngle()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.ANGLE_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.configureAngleClosedLoop();
        
        Robot.dt.getLeftTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        
        Robot.dt.getLeftTalon().setSensorPhase(PigeonConstants.LEFT_SENSOR_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(PigeonConstants.RIGHT_SENSOR_PHASE);
        
        Robot.dt.setBothSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
    }

    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
            numExecutes++;
        else
        {
            numExecutes = -1;
        }
        
        if (isPosition)
        {
            Robot.dt.setBoth(ControlMode.Position, position);
            if (Robot.dt.isClosedLoopErrorWithin(RobotMap.PRIMARY_PID_INDEX, DrivetrainConstants.POS_ALLOWABLE_ERROR))
            {
                isPosition = false;
                Robot.dt.setBoth(ControlMode.Disabled, position);
            }
        }
        else
        {
            if (!isPosition && !isAngle)
            {
                isAngle = true;
                initAngle();
            }

                       
            Robot.dt.setLeft(ControlMode.MotionMagic, angle);
            Robot.dt.setRight(ControlMode.MotionMagic, angle);
        }
        Robot.dt.printClosedLoopError(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.printSensorPositions(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.printMotorOutputPercentage();
    }
    /**
    * Determines whether the command has finished.
    * @return true if the pigeon is in the correct position and the position loop is complete;
    * false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        if (numExecutes == -1 && isAngle)
        {
            boolean isFinished = Robot.dt.isClosedLoopErrorWithin(RobotMap.PRIMARY_PID_INDEX, PigeonConstants.ANGLE_ALLOWABLE_ERROR);
            if (isFinished)
            {
                Robot.dt.setBoth(ControlMode.Disabled, 0);
            }
            return isFinished;
        }
        return false;
    }
   
    /**
     * To be called when the command ends peacefully (isFinished returns true).
     */
    public void end()
    {
        Robot.dt.configureNominalPeakOutputs();
    }
    
}
