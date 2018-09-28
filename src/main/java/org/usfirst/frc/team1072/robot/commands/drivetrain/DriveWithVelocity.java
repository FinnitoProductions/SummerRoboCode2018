package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.SpeedUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the drivetrain to drive with a given velocity. Vel PID
 * 
 * @author Finn Frankis
 * @version 6/11/18
 */
public class DriveWithVelocity extends Command
{
    /**
     * The controller deadband.
     */
    private double deadband;
    
    /**
     * Creates a new DriveWithVelocityCommand object requiring the Drivetrain.
     * @param deadband the controller deadband, or the range for which any input is ignored
     */
    public DriveWithVelocity(double deadband) 
    { 
        requires(Robot.dt); 
        this.deadband = deadband;
    }
    
    public void initialize()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.VEL_PID, RobotMap.PRIMARY_PID_INDEX);
        
        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, 
                DrivetrainConstants.RIGHT_TALON_PHASE);
        
        Robot.dt.resetTalonCoefficients(DrivetrainConstants.VEL_PID);
    }
    /**
     * Executes the command to drive with a given velocity.
     */
    @Override
    public void execute() 
    { 
        OI oi = OI.getInstance();
        
        double leftX = oi.getDriverGamepad().getLeftX();
        double leftY = oi.getDriverGamepad().getLeftY();
        if (Math.abs(leftX) < deadband)
            leftX = 0;
        else
        {
            leftX = leftX -  Math.signum(leftX) * deadband;
            leftX /= 1- deadband;
        }
            
        
        if (Math.abs(leftY) < deadband)
            leftY = 0;
        else
        {
            leftY -= Math.signum(leftY) * deadband;
            leftY /= 1-deadband;
        }
        
        double x = leftX;
        double y = leftY;
        double k = Math.max(1.0, Math.max(Math.abs(y + x * x), Math.abs(y - x * x)));
        double left = (y + x * Math.abs(x)) / k;
        double right = (y - x * Math.abs(x)) / k;

        double leftSpeed = Conversions.convertSpeed
                (SpeedUnit.FEET_PER_SECOND, 
                        left * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                        SpeedUnit.ENCODER_UNITS);
        double rightSpeed = Conversions.convertSpeed
        (SpeedUnit.FEET_PER_SECOND, 
                right * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                SpeedUnit.ENCODER_UNITS);
        
        Robot.dt.getLeftTalon().set(ControlMode.Velocity, leftSpeed);
        Robot.dt.getRightTalon().set(ControlMode.Velocity, rightSpeed);
    }
    
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return true; }

}
