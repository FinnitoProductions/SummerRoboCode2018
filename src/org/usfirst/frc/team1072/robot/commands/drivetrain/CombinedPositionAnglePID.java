package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.AngleUnit;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Combines position and angle for a smoother PID-based autonomous.
 * @author Finn Frankis
 * @version Jul 14, 2018
 */
public class CombinedPositionAnglePID extends Command
{
    private double position, angle;
    private int numExecutes, maxExecutes = 15;
    /**
     * Constructs a new CombinedPositionAnglePID.
     * @param position the final position for the robot in feet
     * @param angle the final angle for the robot in feet
     */
    public CombinedPositionAnglePID (double position, double angle)
    {
        this.position = Conversions.convertPosition(PositionUnit.FEET, position, PositionUnit.ENCODER_UNITS);
        this.angle = Conversions.convertAngle(AngleUnit.DEGREES, angle, AngleUnit.PIGEON_UNITS);
    }
    
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        initPosition();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getRightTalon().set(ControlMode.Position, position, DemandType.AuxPID, angle);
        numExecutes = 0;
    }
    
    /**
     * Initializes the position part of this command (to be used if combined with a subsequent turn).
     */
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.ANGLE_PID, RobotMap.AUXILIARY_PID_INDEX);
        Robot.dt.getLeftTalon().follow(Robot.dt.getRightTalon(), FollowerType.AuxOutput1);
        
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, RobotMap.REMOTE_SLOT_0, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getLeftTalon().getDeviceID(), 
                RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_SLOT_1, RobotMap.TIMEOUT);

        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, 
                RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(
                FeedbackDevice.SensorSum, 
                RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 
                RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.getRightTalon().configSelectedFeedbackCoefficient(0.5, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, DrivetrainConstants.RIGHT_TALON_PHASE);
        
    }
    
    /**
     * Executes this command periodically.
     */
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
        {
            System.out.println("INCREMENTING");
            numExecutes++;
        }
        else
        {
            System.out.println("DONE");
            numExecutes = -1;
        }
        System.out.println("EXECUTING");
        Robot.dt.getRightTalon().set(ControlMode.Position, position, DemandType.AuxPID, angle);
    }
    /**
    * Determines whether this command has finished.
    * @return true if the error is within POS_ALLOWABLE_ERROR; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        if (numExecutes == -1)
        {
            return Math.abs(Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX))
                    < DrivetrainConstants.POS_ALLOWABLE_ERROR;
        }
        System.out.println("NOT FINISHED");
        return false;
    }
    
    public void end()
    {
        Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, 0);
        Robot.dt.getRightTalon().set(ControlMode.PercentOutput, 0);
        System.out.println("FINISHING");
    }
    /**
     * Gets the total number of executes.
     * @return the total number of times the command has executed
     */
    public int getNumExecutes()
    {
        return numExecutes;
    }
    
    /**
     * Gets the maximum number of executes required to end.
     * @return the maximum number of executes required
     */
    public int getMaxExecutes()
    {
        return maxExecutes;
    }
    
    /**
     * Gets the current position of the Talons.
     * @return
     */
    public double getCurrentPosition()
    {
        return (Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) 
                + Robot.dt.getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)) / 2;
    }
    
    /**
     * Gets the position to where the robot is intended to travel.
     * @return the desired position
     */
    public double getDesiredPosition()
    {
        return position;
    }
    
}
