package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;

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
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor
        (FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);

        Robot.dt.selectProfileSlots(Drivetrain.VEL_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.configureVelocityClosedLoop();
        
        Robot.dt.getLeftMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    
        Robot.dt.setTalonSensorPhase(Drivetrain.LEFT_TALON_PHASE, 
                Drivetrain.RIGHT_TALON_PHASE);
        
        Robot.dt.resetTalonCoefficients(Drivetrain.VEL_PID);
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
        
        /*if (Math.abs(leftX) < deadband)
            leftX = 0;
        else
        {
            leftX = leftX -  Math.signum(leftX) * deadband;
            leftX /= 1- deadband;
        }*/
            
        
        if (Math.abs(leftY) < deadband)
            leftY = 0;
        else
        {
            leftY -= Math.signum(leftY) * deadband;
            leftY /= 1-deadband;
        }
        double elevatorPercent = (1.0 * Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition
        (RobotMap.PRIMARY_PID_INDEX)) / Elevator.SCALE_HIGH_HEIGHT;
        
        
        double elevatorScale = 1;
        if (elevatorPercent > Elevator.THROTTLE_PERCENT) {
            elevatorScale = 1-elevatorPercent*(1-Elevator.MIN_THROTTLE_SPEED);
        }

        double x = 0.8 * Math.pow(Math.abs(leftX), 2) * Math.signum(leftX);
        double y = leftY;
        double k = Math.max(1.0, Math.max(Math.abs(y + x), Math.abs(y - x)));
        double left = elevatorScale * (y + x * Math.abs(x)) / k;
        double right = elevatorScale * (y - x * Math.abs(x)) / k;

        /*double leftSpeed = Conversions.convertSpeed
                (SpeedUnit.FEET_PER_SECOND, 
                        elevatorScale * left * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                        SpeedUnit.ENCODER_UNITS);
        double rightSpeed = Conversions.convertSpeed
        (SpeedUnit.FEET_PER_SECOND, 
                elevatorScale * right * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                SpeedUnit.ENCODER_UNITS);*/
        
        Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, left);
        Robot.dt.getRightMaster().set(ControlMode.PercentOutput, right);
    }
    
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return false; }

    double map (double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
}
