package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.AngleUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Turns the robot to a given angle.
 * @author Finn Frankis
 * @version Jul 14, 2018
 */
public class TurnToAngle extends Command
{
    private double angle;
    private double numExecutes;
    private double maxExecutes = 25;
    
    /**
     * Constructs a new TurnRobotToAngleCommand.
     * @param position the final position for the robot
     * @param angle the final angle for the robot in degrees
     */
    public TurnToAngle (double angle)
    {
       
        this.angle = Conversions.convertAngle(AngleUnit.DEGREES, angle, AngleUnit.PIGEON_UNITS);
    }
    
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        initAngle();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        
        numExecutes = 0;
    }
    
    /**
     * Initializes the angle-specific part of this command.
     */
    private void initAngle()
    {
        Robot.dt.getLeftTalon().setSensorPhase(PigeonConstants.LEFT_SENSOR_PHASE);
        Robot.dt.getRightTalon().setSensorPhase(PigeonConstants.RIGHT_SENSOR_PHASE);
        
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
        
        
        Robot.dt.configBothFeedbackSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
    }

    /**
     * Executes this command periodically.
     */
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
            numExecutes++;
        else
        {
            numExecutes = -1;
        }
        
        System.out.println("EXECUTING " + numExecutes);
        
        
        Robot.dt.setLeft(ControlMode.MotionMagic, -1 * angle);
        Robot.dt.setRight(ControlMode.MotionMagic, angle);
    }
    /**
    * Determines whether the commmand has finished.
    * @return true if the error is within ANGLE_ALLOWABLE_ERROR
    */
    @Override
    protected boolean isFinished()
    {
        if (numExecutes == -1)
        {
            return 
                    Robot.dt.isClosedLoopErrorWithin
                    (RobotMap.PRIMARY_PID_INDEX, PigeonConstants.ANGLE_ALLOWABLE_ERROR);
        }
        return false;
    }
    
    /**
     * To be called when the command ends peacefully (isFinished returns true).
     */
    public void end()
    {
        Robot.dt.configureNominalPeakOutputs();
        Robot.dt.setBoth(ControlMode.PercentOutput, 0);
    }
    
}
