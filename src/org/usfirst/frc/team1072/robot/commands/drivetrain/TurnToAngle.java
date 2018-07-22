package org.usfirst.frc.team1072.robot.commands.drivetrain;

import java.util.Arrays;
import java.util.Queue;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
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
    private double angle;
    private double numExecutes;
    private double maxExecutes = 25;
    private double startTime;
    private double timeout;
    private double[] previousErrors;
    private int errorIndex;
    private int errorSamples;
    
    /**
     * Constructs a new TurnRobotToAngleCommand.
     * @param position the final position for the robot
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
        System.out.println("Angle started: " + Robot.getCurrentTimeMs());
        Robot.dt.selectProfileSlots(DrivetrainConstants.ANGLE_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.resetTalonCoefficients(RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.configureAngleClosedLoop();
        
        Robot.dt.getLeftTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configRemoteFeedbackFilter(Robot.dt.getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_SLOT_0, 
                RobotMap.TIMEOUT);
        
        
        Robot.dt.configBothFeedbackSensors(FeedbackDevice.RemoteSensor0, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setTalonSensorPhase(PigeonConstants.LEFT_SENSOR_PHASE, 
                PigeonConstants.RIGHT_SENSOR_PHASE);

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
        
        //System.out.println("EXECUTING " + numExecutes);
    }
    /**
    * Determines whether the commmand has finished.
    * @return true if the error is within ANGLE_ALLOWABLE_ERROR
    */
    @Override
    protected boolean isFinished()
    {
        System.out.println("CHECKING IF FINISHED " + numExecutes);
        previousErrors[errorIndex] = Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX);
        errorIndex = (errorIndex + 1) % previousErrors.length;
        System.out.println(Arrays.toString(previousErrors));
        double timePassed = Robot.getCurrentTimeMs() - startTime;
        if(timePassed > timeout && timeout > 0)
        {
            System.out.println("FINISHED RETURNING TRUE");
            return true; //end early
        }
        if (numExecutes == -1)
        {
            for (double d : previousErrors)
            {
                if (Math.abs(d) > PigeonConstants.ANGLE_ALLOWABLE_ERROR)
                    return false;
            }
            System.out.println("RETURNING TRUE");
            return true;
        }
        return false;
    }
    
    /**
     * To be called when the command ends peacefully (isFinished returns true).
     */
    public void end()
    {
        System.out.println("Angle ended: " + (Robot.getCurrentTimeMs()-startTime));
        
    }
    
}
