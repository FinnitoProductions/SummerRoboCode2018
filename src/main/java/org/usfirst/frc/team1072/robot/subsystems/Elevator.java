package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.CAN_IDs;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorVelocity;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import harkerrobolib.wrappers.HSTalon;

/**
 * Represents an Elevator subsystem with control over the four motors.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Elevator extends Subsystem
{
    /**
     * The instance of this singleton Elevator.
     */
    private static Elevator el;
    
    /**
     * The top right Victor on the elevator.
     */
    private VictorSPX topRightVictor;
    
    /**
     * The bottom left Victor on the elevator.
     */
    private VictorSPX bottomLeftVictor;
    
    /**
     * The top left Victor on the elevator.
     */
    private VictorSPX topLeftVictor;
    
    /**
     * The bottom right (and only) Talon on the elevator.
     */
    private HSTalon bottomRightTalon; 
    
    /**
     * Initializes the command using the four ports in RobotMap.
     */
    public Elevator()
    {
        topRightVictor = new VictorSPX(CAN_IDs.ELEVATOR_VICTOR_TOPRIGHT);
        topLeftVictor = new VictorSPX (CAN_IDs.ELEVATOR_VICTOR_TOPLEFT);
        bottomLeftVictor = new VictorSPX(CAN_IDs.ELEVATOR_VICTOR_BOTTOMLEFT);
        bottomRightTalon = new HSTalon(CAN_IDs.ELEVATOR_TALON, RobotMap.TIMEOUT);
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(new MoveElevatorVelocity());
    }
    
    /**
     * Moves the elevator given a velocity.
     * @param speed the speed with which the elevator will be moved
     */
    public void moveElevatorVelocity(double speed)
    {
        // feed forward counterracts gravity
        bottomRightTalon.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, Elevator.POS_FGRAV);
    }
    
    /**
     * Moves the elevator given a position setpoint.
     * @param position the position to which the elevator will be moved
     */
    public void moveElevatorPosition(double position)
    {
        bottomRightTalon.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, Elevator.POS_FGRAV);
    }
    
    /**
     * Moves the elevator using motion magic. 
     * @param targetPos the position to which the elevatr will be moved
     */
    public void moveElevatorMotionMagic(double targetPos)
    {
        bottomRightTalon.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, Elevator.POS_FGRAV);
    }
    
    /**
     * Initializes the elevator talon for autonomous.
     */
    public void talonInitAutonomous()
    {
        talonInit();
    }
    
    /**
     * Initializes the elevator talon for teleoperated.
     */
    public void talonInitTeleop()
    {
        talonInit();
        elSetSoftLimit(Elevator.FORWARD_SOFT, Elevator.REVERSE_SOFT);
        elSetCurrentLimit(Elevator.PEAK_CURRENT_LIMIT, Elevator.PEAK_TIME_MS,
                Elevator.CONTINOUS_CURRENT_LIMIT);
        elSetRampRate (Elevator.RAMP_RATE);
        elConfigurePositionClosedLoop();
    }
    /**
     * Initializes the talons and victors on the elevator.
     */
    public void talonInit()
    {
        elSlaveVictors();
        elSetNeutralMode(NeutralMode.Brake);

        elInvertControllers();

        elScaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        
        elConfigureMotionMagic();

    }

    
    /**
     * Sets the ramp rate for closed loop elevator motion.
     * @param rampRate the time from zero to full voltage
     */
    private void elSetRampRate (double rampRate)
    {
        getBottomRightTalon().configOpenloopRamp(rampRate);
    }
    /**
     * Slaves the three elevator Victors to the one Talon.
     */
    private void elSlaveVictors()
    {
        getBottomLeftVictor().follow(getBottomRightTalon());
        getTopLeftVictor().follow(getBottomRightTalon());
        getTopRightVictor().follow(getBottomRightTalon());
    }

    /**
     * Inverts the motor controllers so that all align correctly.
     */
    private void elInvertControllers()
    {
        //getBottomLeftVictor().setInverted(ElevatorConstants.BOTTOM_LEFT_VICTOR_INVERT);
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
        getBottomRightTalon().configVoltageCompSaturation(nomVoltage);
    }

    /**
     * Sets the elevator soft limit given it as a parameter.
     * 
     * @param forwardSoftLimit the forward soft limit
     * @param reverseSoftLimit the reverse soft limit
     */
    private void elSetSoftLimit(int forwardSoftLimit, int reverseSoftLimit)
    {
        getBottomRightTalon().configForwardSoftLimitThreshold(forwardSoftLimit);
        getBottomRightTalon().configForwardSoftLimitEnable(true);
        
        getBottomRightTalon().configReverseSoftLimitThreshold(reverseSoftLimit);
        getBottomRightTalon().configReverseSoftLimitEnable(true);
        
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
        getBottomRightTalon().configNominalOutputForward(Elevator.NOMINAL_OUTPUT);
        
        getBottomRightTalon().configNominalOutputReverse(-1 * Elevator.NOMINAL_OUTPUT);
        
        getBottomRightTalon().configPeakOutputForward(Elevator.PEAK_OUTPUT);
        
        getBottomRightTalon().configPeakOutputReverse(-1 * Elevator.PEAK_OUTPUT);
        
        getBottomRightTalon().config_kF(Elevator.POS_PID, Elevator.POS_KF);
        
        getBottomRightTalon().config_kP(Elevator.POS_PID, Elevator.POS_KP);
        
        getBottomRightTalon().config_kI(Elevator.POS_PID, Elevator.POS_KI);
        
        getBottomRightTalon().config_kD(Elevator.POS_PID, Elevator.POS_KD);
        
        getBottomRightTalon().configAllowableClosedloopError(Drivetrain.POS_PID, Elevator.POS_ALLOWABLE_ERROR);
    }
    
    /**
     * Configures Motion Magic on the elevator, which seamlessly allows the robot to move between positions.
     */
    private void elConfigureMotionMagic()
    {
        // set motion magic port to be the velocity PID port 
        getBottomRightTalon().selectProfileSlot(Elevator.MOTION_MAGIC_PID, RobotMap.PRIMARY_PID_INDEX);
        getBottomRightTalon().config_kF(Elevator.MOTION_MAGIC_PID, Elevator.MOTION_MAGIC_KF);
        getBottomRightTalon().config_kP(Elevator.MOTION_MAGIC_PID, Elevator.MOTION_MAGIC_KP);
        getBottomRightTalon().config_kI(Elevator.MOTION_MAGIC_PID, Elevator.MOTION_MAGIC_KI);
        getBottomRightTalon().config_kD(Elevator.MOTION_MAGIC_PID, Elevator.MOTION_MAGIC_KD);
        
        getBottomRightTalon().configMotionCruiseVelocity(Elevator.MOTION_MAGIC_VEL);
        getBottomRightTalon().configMotionAcceleration(Elevator.MOTION_MAGIC_ACCEL);
        
        getBottomRightTalon().configAllowableClosedloopError(Drivetrain.VEL_PID, Elevator.MOTION_MAGIC_ALLOWABLE_ERROR);
        
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
        getBottomRightTalon().configPeakCurrentLimit(peakCurrentLimit);
        getBottomRightTalon().configPeakCurrentDuration(peakTime);
        getBottomRightTalon().configContinuousCurrentLimit(continuousLimit);
        getBottomRightTalon().enableCurrentLimit(true);
    }

    /**
     * Zeros the elevator sensor, should always be called during initialization.
     */
    private void elZeroSensors()
    {
        getBottomRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX);
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
    public HSTalon getBottomRightTalon() { return bottomRightTalon; }

    /**
     * Gets the instance of the singleton Elevator, creating a new one if necessary.
     * @return the instance of Elevator
     */
    public static Elevator getInstance()
    {
        if (el == null) el = new Elevator();
        return el;
    }
    
    /**
     * The height to which the elevator should raise for a switch autonomous (in encoder units).
     */ 
    public static final double SWITCH_HEIGHT_AUTON = 8000;
    
    /**
     * The height to which the elevator should raise for the third cube in a
     *  switch autonomous (in encoder units).
     */
    public static final double SWITCH_HEIGHT_THIRD_CUBE = 5000;
    
    /**
     * The continuous current limit when in manual control.
     */
    public static int MANUAL_CURRENT_LIMIT_CONT = 10;
    
    /**
     * The peak current limit for any control.
     */
    public static int PEAK_CURRENT_LIMIT = 25; // 25
    
    /**
     * The time (in ms) at which the peak current limit is valid.
     */
    public static int PEAK_TIME_MS = 750;
    
    /**
     * The continuous current limit for non-manual control.
     */
    public static int CONTINOUS_CURRENT_LIMIT = 15; //15
    
    /**
     * The PID slot to house motion magic constants.
     */
    public static int MOTION_MAGIC_PID = 0;
    
    /**
     * The PID slot to house position constants.
     */
    public static int POS_PID = 1;
    
    /**
     * The elevator nominal output.
     */
    public static double NOMINAL_OUTPUT = 0;
    
    /**
     * The feed forward to constantly add to the elevator to resist the effect
     * of gravity.
     */
    public static double POS_FGRAV = 0.06;
    
    /**
     * The peak output for the elevator motor controllers.
     */
    public static double PEAK_OUTPUT = 1.0;
    
    /**
     * The F constant for the position closed loop.
     */
    public static double POS_KF = 0;
    
    /**
     * The P constant for the position closed loop.
     */
    public static double POS_KP = 0.1;
    
    /**
     * The I constant for the position closed loop.
     */
    public static double POS_KI = 0.0001;
    
    /**
     * The D constant for the position closed loop.
     */
    public static double POS_KD = 18;
    
    /**
     * The allowable error for the position closed loop.
     */
    public static int POS_ALLOWABLE_ERROR = 500;
    
    /**
     * The soft limit in the forward (upward) direction.
     */
    public static int FORWARD_SOFT = 34500;
    
    /**
     * The soft limit in the reverse (downward) direction.
     */
    public static int REVERSE_SOFT = 2000;
    
    /**
     * The time (in seconds) for which the elevator should ramp up to full speed in 
     * manual control.
     */
    public static double RAMP_RATE = 0.75;
    
    // elevator max RPM: 500 RPM
    /**
     * The F constant for the motion magic closed loop.
     */
    public static double MOTION_MAGIC_KF = 0.37;
    
    /**
     * The P constant for the motion magic closed loop.
     */
    public static double MOTION_MAGIC_KP = 0.1;
    
    /**
     * The I constant for the motion magic closed loop.
     */
    public static double MOTION_MAGIC_KI = 0.001;
    
    /**
     * The D constant for the motion magic closed loop.
     */
    public static double MOTION_MAGIC_KD = 14;
    
    /**
     * The acceleration constant for the motion magic closed loop.
     */
    public static int MOTION_MAGIC_ACCEL = 3000;
    
    /**
     * The velocity constant for the motion magic closed loop.
     */
    public static int MOTION_MAGIC_VEL = 9000;
    
    /**
     * The allowable error for the motion magic closed loop.
     */
    public static int MOTION_MAGIC_ALLOWABLE_ERROR = 500;
    
    /**
     * The height (in encoder units) for optimal intaking.
     */
    public static int INTAKE_HEIGHT = 0;

    public static int RAISE_HEIGHT = 1000;
    
    /**
     * The height (in encoder units) for optimal switch scoring.
     */
    public static int SWITCH_HEIGHT = 10000;
    
    /**
     * The height (in encoder units) for optimal low scale scoring.
     */
    public static int SCALE_LOW_HEIGHT = 22500;
    
    /**
     * The height (in encoder units) for optimal high scale scoring.
     */
    public static int SCALE_HIGH_HEIGHT = 34700;
    
    /**
     * The position at which the elevator should begin to slow.
     */
    public static double SLOW_DOWN_POS = 1000;
    
    /**
     * Whether to invert the bottom left victor (those not shown can be assumed false).
     */
    public static boolean BOTTOM_LEFT_VICTOR_INVERT = true;
    
    /**
     * The sensor phase for the elevator Talon.
     */
    public static boolean TALON_PHASE = true;
    
    public static double THROTTLE_PERCENT = 0.5;

    public static double MIN_THROTTLE_SPEED = 0.25;
    
}
