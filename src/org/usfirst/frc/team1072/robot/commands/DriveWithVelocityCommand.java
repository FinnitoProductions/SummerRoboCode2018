package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.util.Speed;
import org.usfirst.frc.team1072.util.Speed.SpeedUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Commands the drivetrain to drive with a given velocity. Vel PID
 * 
 * @author Finn Frankis
 * @version 6/11/18
 */
public class DriveWithVelocityCommand extends Command
{
    private double driveSpeed;
    private double turnSpeed;
    private double deadband;
    
    /**
     * Creates a new DriveWithVelocityCommand object requiring the Drivetrain.
     */
    public DriveWithVelocityCommand(double deadband) 
    { 
        requires(Robot.dt); 
        this.deadband = deadband;
    }
    
    public void initialize()
    {
        Robot.dt.selectProfileSlots(RobotMap.DT_VEL_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    }
    /**
     * Executes the command to drive with a given velocity.
     * @param speed the speed at which the robot will drive
     * @param turn the amount by which the robot should turn
     */
    @Override
    public void execute() 
    { 
        OI oi = OI.getInstance();
        
        double leftX = oi.getGamepad().getLeftX();
        double leftY = oi.getGamepad().getLeftY();
        if (Math.abs(leftX) < deadband)
            leftX = 0;
        else
        {
            leftX -= Math.signum(leftX) * deadband;
            leftX /= 1- deadband;
        }
            
        
        if (Math.abs(leftY) < deadband)
            leftY = 0;
        else
        {
            leftY -= Math.signum(leftY) * deadband;
            leftY /= 1-deadband;
        }
        
        driveSpeed = new Speed(SpeedUnit.FEET_PER_SECOND, leftY * RobotMap.MAX_DRIVE_SPEED_FPS, RobotMap.WHEELDIAMETER).getEncoderUnits(); 
        turnSpeed = new Speed(SpeedUnit.FEET_PER_SECOND, -1 * leftX * RobotMap.MAX_TURN_SPEED_FPS, RobotMap.WHEELDIAMETER).getEncoderUnits();
        
        Robot.dt.arcadeDriveVelocity(
                driveSpeed, 
                turnSpeed); 
        
        SmartDashboard.putNumber("LEFT Motor Voltage", driveSpeed - Robot.dt.getLeftTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("RIGHT Motor Voltage", driveSpeed - Robot.dt.getRightTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("LEFT TALON ERROR", driveSpeed - Robot.dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
        SmartDashboard.putNumber("RIGHT TALON ERROR", driveSpeed - Robot.dt.getRightTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
    }
    
    public double getDriveSpeed()
    {
        return driveSpeed;
    }
    
    public double getTurnSpeed()
    {
        return turnSpeed;
    }
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return true; }

}
