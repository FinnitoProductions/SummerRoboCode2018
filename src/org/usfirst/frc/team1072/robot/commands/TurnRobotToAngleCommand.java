package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Angle;
import org.usfirst.frc.team1072.util.Angle.AngleUnit;
import org.usfirst.frc.team1072.util.ConversionFactors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnRobotToAngleCommand extends Command
{
    private double angle;
    
    public TurnRobotToAngleCommand()
    {
        requires (Robot.dt);
        angle = -1;
    }
    /**
     * 
     * @param angle the angle to which the robot should turn (in degrees).
     */
    public TurnRobotToAngleCommand(double angle)
    {
        requires (Robot.dt);
        this.angle = new Angle(AngleUnit.DEGREES, angle).getPigeonUnits();
    }
    
    public void initialize()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.ANGLE_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.getLeftTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
                RobotMap.PRIMARY_PID_INDEX, 
                RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
                RobotMap.PRIMARY_PID_INDEX, 
                RobotMap.TIMEOUT);
        
        System.out.println("ZEROING");
        Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
    }
    @Override
    public void execute()
    {
        Robot.dt.getLeftTalon().setSensorPhase(false);
        Robot.dt.getRightTalon().setSensorPhase(false);

        Robot.dt.getLeftTalon().set(ControlMode.Position, -1 * angle);
        Robot.dt.getRightTalon().set(ControlMode.Position, angle);
        
        SmartDashboard.putNumber("PRIMARY ERROR RIGHT", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("PRIMARY ERROR LEFT", Robot.dt.getLeftTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        
        

    }
    @Override
    protected boolean isFinished()
    {
        return 
                Math.abs(Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < PigeonConstants.ANGLE_ALLOWABLE_ERROR 
                && Math.abs(Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX)) < PigeonConstants.ANGLE_ALLOWABLE_ERROR;
        //return 
    }
    
    @Override
    public void end()
    {
    }

}
