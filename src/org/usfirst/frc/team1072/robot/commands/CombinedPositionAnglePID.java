package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Angle;
import org.usfirst.frc.team1072.util.ConversionFactors;
import org.usfirst.frc.team1072.util.Angle.AngleUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Combines position and angle for a smoother PID autonomous.
 * @author Finn Frankis
 * @version Jul 14, 2018
 */
public class CombinedPositionAnglePID extends Command
{
    private double position;
    private double angle;
    private double numExecutes;
    private double maxExecutes = 3;
    
    /**
     * Constructs a new CombinedPositionAnglePID.
     * @param position the final position for the robot
     * @param angle the final angle for the robot in degrees
     */
    public CombinedPositionAnglePID (double position, double angle)
    {
        this.position = position;
        this.angle = new Angle(AngleUnit.DEGREES, angle).getPigeonUnits();
    }
    
    public void initialize()
    {
        initAngle();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        
        numExecutes = 0;
    }
    
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setBothSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    }
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
        
        Robot.dt.setBothSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
    }

    
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
            numExecutes++;
        else
        {
            numExecutes = -1;
        }
        
        //Robot.dt.setBoth(ControlMode.Position, position);
        
        Robot.dt.getLeftTalon().setSensorPhase(PigeonConstants.LEFT_SENSOR_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(PigeonConstants.RIGHT_SENSOR_PHASE);
        
        
        Robot.dt.setLeft(ControlMode.MotionMagic, -1 * 90 * 
                ConversionFactors.PIGEON_UNITS_PER_ROTATION/ConversionFactors.DEGREES_PER_ROTATION);
        Robot.dt.setRight(ControlMode.MotionMagic, 90 * 
                ConversionFactors.PIGEON_UNITS_PER_ROTATION/ConversionFactors.DEGREES_PER_ROTATION);
        
        Robot.dt.printClosedLoopError(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.printSensorPositions(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.printMotorOutputPercentage();
    }
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        /*if (numExecutes == -1)
        {
            boolean isFinished = Math.abs(Robot.dt.getLeftTalon().
                    getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < DrivetrainConstants.POS_ALLOWABLE_ERROR
                && Math.abs(Robot.dt.getRightTalon().
                    getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < DrivetrainConstants.POS_ALLOWABLE_ERROR;
            if (isFinished)
            {
                Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, 0);
                Robot.dt.getRightTalon().set(ControlMode.PercentOutput, 0);
            }
            return isFinished;
        }*/
        return false;
    }
    
}
