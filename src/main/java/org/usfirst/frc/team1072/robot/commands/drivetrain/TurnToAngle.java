package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.Pigeon;
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
    /**
     * The angle to which this command should closed loop.
     */
    private double angle;
    
    /**
     * The number of times which this command has executed.
     */
    private double numExecutes;
    
    /**
     * The minimum number of times for which this command must executed to be marked as complete. This ensures
     * that isFinished() does not immediately return true due to a delayed spike in error.
     */
    private double maxExecutes = 25;
    
    /**
     * The time at which the command began.
     */
    private double startTime;
    
    /**
     * The amount of time after which this command should stop.
     */
    private double timeout;
    
    /**
     * The previous closed loop errors for this command a given number back.
     */
    private double[] previousErrors;
    
    /**
     * The index at which the most recent error resides.
     */
    private int errorIndex;
    
    /**
     * The total number of error samples to be taken.
     */
    private int errorSamples;
    
    /**
     * Constructs a new TurnRobotToAngleCommand.
     * @param angle the final angle for the robot in degrees
     */
    public TurnToAngle (double angle)
    {
        errorIndex = 0;
        errorSamples = 6;
        this.angle = Conversions.convertAngle(AngleUnit.DEGREES, angle, AngleUnit.PIGEON_UNITS);
        this.timeout = -1.0;
    }
    
    /**
     * Constructs a new TurnToAngle.
     * @param angle the final angle for the robot in degrees
     * @param timeout the time at which the command stops, regardless of how much progress the robot has made.
     */
    public TurnToAngle (double angle, double timeout)
    {
        errorIndex = 0;
        errorSamples = 6;
        this.angle = Conversions.convertAngle(AngleUnit.DEGREES, angle, AngleUnit.PIGEON_UNITS);
        this.timeout = timeout*1000;
    }
    
    /**
     * Sets the number of errors to keep track of when determining whether the robot is near the setpoint and staying there 
     * (as opposed to passing by quickly).
     * @param numErrors the number of errors to keep track of
     * @return this command
     */
    public TurnToAngle setNumErrorsToCheck (int numErrors)
    {
        errorSamples = numErrors;
        return this;
    }
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        previousErrors = new double[errorSamples];
        for (int i = 0; i < errorSamples; i++)
            previousErrors[i] = angle;
        initAngle();
        numExecutes = 0;
    }
    
    /**
     * Initializes the angle-specific part of this command.
     */
    private void initAngle()
    {
        startTime = Robot.getCurrentTimeMs();
        ;
        Robot.dt.selectProfileSlots(Drivetrain.ANGLE_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.resetTalonCoefficients(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.configureAngleClosedLoop();
        
        Robot.dt.getLeftMaster().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, errorIndex);
        Robot.dt.getRightMaster().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, errorIndex);
        
        
        Robot.dt.configBothFeedbackSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
        
        //Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setTalonSensorPhase(Drivetrain.Pigeon.LEFT_SENSOR_PHASE, 
                Drivetrain.Pigeon.RIGHT_SENSOR_PHASE);

        Robot.dt.setLeft(ControlMode.MotionMagic, -angle);
        Robot.dt.setRight(ControlMode.MotionMagic, angle);
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
        
        ;
    }
    /**
    * Determines whether the commmand has finished.
    * @return true if the error is within ANGLE_ALLOWABLE_ERROR
    */
    @Override
    protected boolean isFinished()
    {
        double timePassed = Robot.getCurrentTimeMs() - startTime;
        if(timePassed > timeout && timeout > 0)
        {
            return true; //end early
        }
        
        previousErrors[errorIndex] = Robot.dt.getRightMaster().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX);
        errorIndex = (errorIndex + 1) % previousErrors.length;
        if (numExecutes == -1)
        {
            for (double d : previousErrors)
            {
                if (Math.abs(d) > Drivetrain.Pigeon.ANGLE_ALLOWABLE_ERROR)
                    return false;
            }
            return true;
        }
        return false;
    }
    
    /**
     * To be called when the command ends peacefully (isFinished returns true).
     */
    public void end()
    {
        ;
    }
    
}
