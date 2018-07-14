package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.CAN_IDs;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.IntakeOuttakeCubeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents an intake subsystem with pneumatic and motor control.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Intake extends Subsystem
{
    public static Intake intake = null;
    public static Pneumatics pn = Pneumatics.getInstance();
    
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    
    public Intake()
    {
        leftTalon = new TalonSRX(CAN_IDs.INTAKE_TALON_LEFT);
        rightTalon = new TalonSRX(CAN_IDs.INTAKE_TALON_RIGHT);
    }
    /**
     * Initializes the command using the ports for the left and right Talons.
     */
    protected void initDefaultCommand()
    {
        setDefaultCommand(new IntakeOuttakeCubeCommand());
    }

    /**
     * Intakes/outakes a cube given a left and right speed (if two joysticks)
     * @param speedLeft the percent output for the left intake
     * @param speedRight the percent output for the right intake
     */
    public void intakeOuttakeCube(double speedLeft, double speedRight)
    {
        leftTalon.set(ControlMode.PercentOutput, speedLeft);
        rightTalon.set(ControlMode.PercentOutput, speedRight);
    }
    
    /**
     * Intakes/outakes a cube given a single speed for both halves.
     * @param speed the speed at which both halves should be run
     */
    public void intakeOuttakeCube (double speed)
    {
        leftTalon.set(ControlMode.PercentOutput, speed);
        rightTalon.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Initializes the intake talons.
     */
    public void talonInit()
    {
        intakeSetNeutralMode(NeutralMode.Brake);
        intakeSetCurrentLimit(IntakeConstants.PEAK_CURRENT_LIMIT, IntakeConstants.PEAK_TIME_MS,
                IntakeConstants.CONTINUOUS_CURRENT_LIMIT);
    }

    /**
     * Sets the neutral mode for the Talons
     * 
     * @param nm the neutral mode
     */
    private void intakeSetNeutralMode(NeutralMode nm)
    {
        getLeftTalon().setNeutralMode(nm);
        getRightTalon().setNeutralMode(nm);
    }

    /**
     * Sets the current limit on the 
     * 
     * @param peakCurrentLimit
     *            the peak limit (only temporary)
     * @param peakTime
     *            the time (in ms) for the peak limit to be allowed
     * @param continuousLimit
     *            the continuous current limit (after peak has expired)
     */
    private void intakeSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        getLeftTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        getLeftTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        getLeftTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        getLeftTalon().enableCurrentLimit(true);
        getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
    }
    
    /**
     * Gets the left Talon on the 
     * @return the left Talon
     */
    public TalonSRX getLeftTalon() { return leftTalon; }

    /**
     * Gets the right Talon on the 
     * @return the right Talon
     */
    public TalonSRX getRightTalon() { return rightTalon; }

    /**
     * Gets the instance of this singleton Intake, returning a new one if one has not yet been created.
     * @return this Intake instance
     */
    public static Intake getInstance()
    {
        if (intake == null) intake = new Intake();
        return intake;
    }


}
