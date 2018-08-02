package org.usfirst.frc.team1072.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

/**
 * Drives the robot to a given position using PID.
 * @author Finn Frankis
 * @version Jun 14, 2018
 */
public class DriveToPosition extends PositionCommand
{
    /**
     * The desired position for this closed loop.
     */
    private double position;
    
    /**
     * Constructs a new DriveToPositionCommand.
     * @param position the final position for the robot in feet
     */
    public DriveToPosition (double position)
    {
        super (10, Conversions.convertPosition(PositionUnit.FEET, position, PositionUnit.ENCODER_UNITS));
        this.position = Conversions.convertPosition(PositionUnit.FEET, position, PositionUnit.ENCODER_UNITS);
    }
    
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        initPosition();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.setBoth(ControlMode.Position, position);
        Robot.dt.resetTalonCoefficients(RobotMap.PRIMARY_PID_INDEX);
    }
    
    /**
     * Initializes the position part of this command (to be used if combined with a subsequent turn).
     */
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.configBothFeedbackSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, DrivetrainConstants.RIGHT_TALON_PHASE);
        
    }
    
    /**
     * Executes this command periodically.
     */
    public void execute()
    {
        if (!passedMaxExecutes())
        {
            incrementNumExecutes();
        }
        Robot.dt.setBoth(ControlMode.Position, position);
    }
    /**
    * Determines whether this command has finished.
    * @return true if the error is within POS_ALLOWABLE_ERROR; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        if (passedMaxExecutes())
        {
            return Robot.dt.isClosedLoopErrorWithin(RobotMap.PRIMARY_PID_INDEX, getAllowableError());
        }
        return false;
    }
    
    public void end()
    {
        Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, 0);
        Robot.dt.getRightTalon().set(ControlMode.PercentOutput, 0);
    }
}
