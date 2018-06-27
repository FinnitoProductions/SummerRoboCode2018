package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorMotionMagicCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorVelocityCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents an Elevator subsystem with control over the four motors.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Elevator extends Subsystem
{
    private static Elevator el;
    
    private VictorSPX topRightVictor;
    private VictorSPX bottomLeftVictor;
    private VictorSPX topLeftVictor;
    private TalonSRX bottomRightTalon; 
    
    /**
     * Initializes the command using the four ports in RobotMap.
     */
    public Elevator()
    {
        topRightVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_TOPRIGHT);
        topLeftVictor = new VictorSPX (RobotMap.ELEVATOR_VICTOR_TOPLEFT);
        bottomLeftVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_BOTTOMLEFT);
        bottomRightTalon = new TalonSRX(RobotMap.ELEVATOR_TALON);
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(new MoveElevatorVelocityCommand());
    }
    
    /**
     * Moves the elevator given a velocity.
     * @param speed the speed with which the elevator will be moved
     */
    public void moveElevatorVelocity(double speed)
    {
        // feed forward counterracts gravity
        //bottomRightTalon.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Moves the elevator given a position setpoint.
     * @param position the position to which the elevator will be moved
     */
    public void moveElevatorPosition(double position)
    {
        //bottomRightTalon.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Moves the elevator using motion magic. 
     * @param targetPos the position to which the elevatr will be moved
     */
    public void moveElevatorMotionMagic(double targetPos)
    {
        //bottomRightTalon.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Initializes the talons and victors on the elevator.
     */
    public void talonInit()
    {
        elSlaveVictors();
        elSetNeutralMode(NeutralMode.Brake);
        elSetCurrentLimit(RobotMap.EL_PEAK_CURRENT_LIMIT, RobotMap.EL_PEAK_TIME_MS,
                RobotMap.EL_CONTINOUS_CURRENT_LIMIT);
        elInvertControllers();
        elScaleVoltage(RobotMap.EL_NOMINAL_OUTPUT);
        elSetSoftLimit(RobotMap.EL_FORWARD_SOFT, RobotMap.EL_REVERSE_SOFT);

        elSetRampRate (RobotMap.EL_RAMP_RATE);
        elScaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        elConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        elZeroSensors();

        elConfigurePositionClosedLoop();
        
        elConfigureMotionMagic();

    }

    
    /**
     * Sets the ramp rate for closed loop elevator motion.
     * @param rampRate the time from zero to full voltage
     */
    private void elSetRampRate (double rampRate)
    {
        getBottomRightTalon().configOpenloopRamp(rampRate, RobotMap.TIMEOUT);
    }
    /**
     * Slaves the three elevator Victors to the one Talon.
     */
    private void elSlaveVictors()
    {
        TalonSRX talon = getBottomRightTalon();
        getBottomLeftVictor().follow(talon);
        getTopLeftVictor().follow(talon);
        getTopRightVictor().follow(talon);
    }

    /**
     * Inverts the motor controllers so that all align correctly.
     */
    private void elInvertControllers()
    {
        getBottomLeftVictor().setInverted(RobotMap.EL_BOTTOM_LEFT_VICTOR_INVERT);
        getBottomRightTalon().setInverted(RobotMap.EL_TALON_INVERT);
        getTopRightVictor().setInverted(RobotMap.EL_TOP_RIGHT_VICTOR_INVERT);
        getTopLeftVictor().setInverted(RobotMap.EL_TOP_LEFT_VICTOR_INVERT);
    }

    /**
     * Sets the correct neutral mode for the motor controllers on the elevator
     * 
     * @param n the neutral mode (coast or brake) to be set
     */
    private void elSetNeutralMode(NeutralMode n)
    {
        getBottomRightTalon().setNeutralMode(n);
        getTopRightVictor().setNeutralMode(n);
        getBottomLeftVictor().setNeutralMode(n);
        getTopLeftVictor().setNeutralMode(n);
    }

    /**
     * Scales the voltage on the elevator controllers given nominal voltage.
     * 
     * @param nomVoltage
     *            the nominal voltage of the battery
     */
    private void elScaleVoltage(double nomVoltage)
    {
        getBottomRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
    }

    /**
     * Sets the elevator soft limit given it as a parameter.
     * 
     * @param softLimit
     *            the soft limit to be set
     */
    private void elSetSoftLimit(int forwardSoftLimit, int reverseSoftLimit)
    {
        getBottomRightTalon().configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.TIMEOUT);
        getBottomRightTalon().configForwardSoftLimitEnable(true, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.TIMEOUT);
        getBottomRightTalon().configReverseSoftLimitEnable(true, RobotMap.TIMEOUT);
        
        getBottomRightTalon().overrideLimitSwitchesEnable(true);
    }

    /**
     * Configures the elevator position closed loop.
     * 
     * @postcondition the F/P/I/D constants have been set, as well as the nominal
     *                output in both directions
     */
    private void elConfigurePositionClosedLoop()
    {
        getBottomRightTalon().configNominalOutputForward(RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configNominalOutputReverse(-1 * RobotMap.EL_NOMINAL_OUTPUT, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configPeakOutputForward(RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configPeakOutputReverse(-1 * RobotMap.EL_PEAK_OUTPUT, RobotMap.TIMEOUT);
        
        getBottomRightTalon().config_kF(RobotMap.EL_POS_PID, RobotMap.EL_POS_KF, RobotMap.TIMEOUT);
        
        getBottomRightTalon().config_kP(RobotMap.EL_POS_PID, RobotMap.EL_POS_KP, RobotMap.TIMEOUT);
        
        getBottomRightTalon().config_kI(RobotMap.EL_POS_PID, RobotMap.EL_POS_KI, RobotMap.TIMEOUT);
        
        getBottomRightTalon().config_kD(RobotMap.EL_POS_PID, RobotMap.EL_POS_KD, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.EL_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
    }
    
    /**
     * Configures Motion Magic on the elevator, which seamlessly allows the robot to move between positions.
     */
    private void elConfigureMotionMagic()
    {
        // set motion magic port to be the velocity PID port 
        getBottomRightTalon().selectProfileSlot(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_PID);
        getBottomRightTalon().config_kF(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KF, RobotMap.TIMEOUT);
        getBottomRightTalon().config_kP(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KP, RobotMap.TIMEOUT);
        getBottomRightTalon().config_kI(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KI, RobotMap.TIMEOUT);
        getBottomRightTalon().config_kD(RobotMap.EL_VEL_PID, RobotMap.EL_VEL_KD, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configMotionCruiseVelocity(RobotMap.EL_VEL_VEL, RobotMap.TIMEOUT);
        getBottomRightTalon().configMotionAcceleration(RobotMap.EL_VEL_ACCEL, RobotMap.TIMEOUT);
        
        getBottomRightTalon().configAllowableClosedloopError(RobotMap.DT_VEL_PID, RobotMap.EL_VEL_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        
    }

    /**
     * Sets the current limit for the elevator in amps.
     * 
     * @param peakCurrentLimit
     *            the peak limit (only temporary)
     * @param peakTime
     *            the time (in ms) for which the peak limit is permitted
     * @param continuousLimit
     *            the limit after the peak time has expired.
     */
    private void elSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        getBottomRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        getBottomRightTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        getBottomRightTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        getBottomRightTalon().enableCurrentLimit(true);
    }

    /**
     * Configures the elevator sensor given the feedback device.
     * 
     * @param fd the feedback device, either quadrature or absolute
     */
    private void elConfigureSensors(FeedbackDevice fd)
    {
        getBottomRightTalon().configSelectedFeedbackSensor(fd, RobotMap.POS_PID, RobotMap.TIMEOUT);
    }

    /**
     * Zeros the elevator sensor, should always be called during initialization.
     */
    private void elZeroSensors()
    {
        getBottomRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
    }
    /**
     * Gets the top right Victor on the elevator.
     * @return the top right Victor
     */
    public VictorSPX getTopRightVictor() { return topRightVictor; }

    /**
     * Gets the bottom left Victor on the elevator.
     * @return the bottom left Victor
     */
    public VictorSPX getBottomLeftVictor() { return bottomLeftVictor; }

    /**
     * Gets the top left Victor on the elevator.
     * @return the top left Victor
     */
    public VictorSPX getTopLeftVictor() { return topLeftVictor; }

    /**
     * Gets the bottom right Talon on the elevator.
     * @return the bottom right Talon
     */
    public TalonSRX getBottomRightTalon() { return bottomRightTalon; }

    /**
     * Gets the instance of the singleton Elevator, creating a new one if necessary.
     * @return the instance of Elevator
     */
    public static Elevator getInstance()
    {
        if (el == null) el = new Elevator();
        return el;
    }
    
}
