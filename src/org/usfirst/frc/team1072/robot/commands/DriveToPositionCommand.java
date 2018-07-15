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
 * Drives the robot to a given position using PID.
 * @author Finn Frankis
 * @version Jun 14, 2018
 */
public class DriveToPositionCommand extends Command
{
    private double position;
    private double numExecutes;
    private double maxExecutes = 3;
    
    /**
     * Constructs a new DriveToPositionCommand.
     * @param position the final position for the robot
     */
    public DriveToPositionCommand (double position)
    {
        this.position = position;
    }
    
    public void initialize()
    {
        initPosition();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        
        numExecutes = 0;
    }
    
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setBothSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    }
    

    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
            numExecutes++;
        else
        {
            numExecutes = -1;
        }

        Robot.dt.setBoth(ControlMode.Position, position);
    }
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        if (numExecutes == -1)
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
        }
        return false;
    }

}
