package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Finn Frankis
 * @version Jul 14, 2018
 */
public class CombinedPositionAnglePID extends Command
{
    private double position;
    private double numExecutes;
    private double maxExecutes = 3;
    
    public CombinedPositionAnglePID (double position)
    {
        this.position = position;
    }
    
    public void initialize()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.getLeftTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
       
        //Robot.dt.setBothSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.setBothSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        numExecutes = 0;
    }
    

    
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
            numExecutes++;
        else
        {
            numExecutes = -1;
        }
        
        Robot.dt.getLeftTalon().set(ControlMode.Position, position);
        Robot.dt.getRightTalon().set(ControlMode.Position, position);
        
        SmartDashboard.putNumber("PRIMARY ERROR RIGHT", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("PRIMARY ERROR LEFT", Robot.dt.getLeftTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));

        SmartDashboard.putNumber("POSITION RIGHT", Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("POSITION LEFT", Robot.dt.getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
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
