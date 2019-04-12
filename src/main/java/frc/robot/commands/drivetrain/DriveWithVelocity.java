package frc.robot.commands.drivetrain;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.SpeedUnit;

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
        
        double leftX = MathUtil.mapJoystickOutput(oi.getDriverGamepad().getLeftX(), OI.BLACK_XBOX_DRIVE_DEADBAND);
        double leftY = MathUtil.mapJoystickOutput(oi.getDriverGamepad().getLeftY(), OI.BLACK_XBOX_DRIVE_DEADBAND);

        // double elevatorPercent = (1.0 * Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition
        // (RobotMap.PRIMARY_PID_INDEX)) / Elevator.SCALE_HIGH_HEIGHT;
        
        
        double elevatorScale = 1;
        // if (elevatorPercent > Elevator.THROTTLE_PERCENT) {
        //     elevatorScale = 1-elevatorPercent*(1-Elevator.MIN_THROTTLE_SPEED);
        // }

        double x = 0.4 * Math.pow(Math.abs(leftX), 2) * Math.signum(leftX);
        double y = leftY;
        // double k = Math.max(1.0, Math.max(Math.abs(y + x), Math.abs(y - x)));
        double left = elevatorScale * (y * Drivetrain.MAX_DRIVE_SPEED_FPS + x * Drivetrain.MAX_TURN_SPEED_FPS);
        double right = elevatorScale * (y * Drivetrain.MAX_DRIVE_SPEED_FPS - x * Drivetrain.MAX_TURN_SPEED_FPS);

        /*double leftSpeed = Conversions.convertSpeed
                (SpeedUnit.FEET_PER_SECOND, 
                        elevatorScale * left * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                        SpeedUnit.ENCODER_UNITS);
        double rightSpeed = Conversions.convertSpeed
        (SpeedUnit.FEET_PER_SECOND, 
                elevatorScale * right * DrivetrainConstants.MAX_DRIVE_SPEED_FPS, 
                SpeedUnit.ENCODER_UNITS);*/
        
        if (Math.abs(left) < 2e-6) {
            Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, 0);
        } else {
            Robot.dt.getLeftMaster().set(ControlMode.Velocity, Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, left, SpeedUnit.ENCODER_UNITS));
        }

        if (Math.abs(right) < 2e-6) {
            Robot.dt.getRightMaster().set(ControlMode.PercentOutput, 0);
        } else {
            Robot.dt.getRightMaster().set(ControlMode.Velocity, Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, right, SpeedUnit.ENCODER_UNITS));
        }
    }
    
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return false; }

    double map (double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
}
