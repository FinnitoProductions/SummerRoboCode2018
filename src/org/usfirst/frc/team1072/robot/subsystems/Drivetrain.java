package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.DriveWithVelocityCommand;
import org.usfirst.frc.team1072.robot.commands.TurnRobotToAngleCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents a drive train with two Talons and two Victors.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class Drivetrain extends Subsystem
{
    private static Drivetrain instance = null;
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    private VictorSPX leftVictor;
    private VictorSPX rightVictor;
    private PigeonIMU pigeon;
   
    
    private Drivetrain()
    {
        // initialize talons
        leftTalon = new TalonSRX (RobotMap.LEFT_CIM_TALON);
        rightTalon = new TalonSRX (RobotMap.RIGHT_CIM_TALON);
        leftVictor = new VictorSPX (RobotMap.LEFT_CIM_VICTOR);
        rightVictor = new VictorSPX (RobotMap.RIGHT_CIM_VICTOR);
        pigeon = new PigeonIMU(RobotMap.PIGEON_ID);
        

    }
    /**
     * Initializes the command using the four ports for the Talons/Victors.
     */
    public void initDefaultCommand()
    {
        //setDefaultCommand(new DriveWithVelocityCommand());
        //setDefaultCommand(new DriveToPositionCommand());
       setDefaultCommand(new TurnRobotToAngleCommand());
    }
    
    /**
     * Runs both motor controllers given a velocity and turn.
     * @param speed the speed at which the controllers should run
     * @param turn the amount by which each speed should be modified to account for turn
     */
    public void arcadeDriveVelocity(double speed, double turn)
    {
        // victor follows talon
        getLeftTalon().selectProfileSlot(RobotMap.DT_VEL_PID, RobotMap.PRIMARY_PID);
        getRightTalon().selectProfileSlot(RobotMap.DT_VEL_PID, RobotMap.PRIMARY_PID);
        rightTalon.set(ControlMode.Velocity, speed + turn);  
        leftTalon.set(ControlMode.Velocity, speed - turn);  

    }
    
    /**
     * Moves the robot to a given position using PID.
     * @param target the target position 
     */
    public void arcadeDrivePosition (double target)
    {
        System.out.println("DRIVING TO POSITION");
        getLeftTalon().selectProfileSlot(RobotMap.POS_PID, 0);
        getRightTalon().selectProfileSlot(RobotMap.POS_PID, 0);
        leftTalon.set(ControlMode.Position, target);
        rightTalon.set(ControlMode.Position, target);
    }
    

    
    /**
     * Performs all commands to initialize the talons.
     * 
     * @postcondition the talons are in the correct state to begin robot operation
     */
    public void talonInit()
    {
        dtSlaveVictors();
        // initTalonOutput(0);

        dtInvertControllers();
        dtSetNeutralMode(NeutralMode.Brake);

        dtSetRampTime(RobotMap.MAX_RAMP_TIME);

        dtScaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);

        dtSetSensorPhase();
        enableVelocityClosedLoop();
        dtConfigurePositionClosedLoop();
        dtConfigureMotionProfileClosedLoop();

        dtSetCurrentLimit(RobotMap.DT_PEAK_CURRENT_LIMIT, RobotMap.DT_PEAK_TIME_MS,
                RobotMap.DT_CONTINUOUS_CURRENT_LIMIT);

    }
    
    /**
     * Slaves the Victors to directly follow the behavior of their parent talons.
     */
    private void dtSlaveVictors()
    {
        getLeftVictor().follow(getLeftTalon());
        getRightVictor().follow(getRightTalon());
    }

    /**
     * Initializes the Talon output.
     * 
     * @param percentOutput
     *            the output to which the Talons should be initialized
     */
    private void dtInitTalonOutput(int percentOutput)
    {
        getLeftTalon().set(ControlMode.Velocity, percentOutput);
        getRightTalon().set(ControlMode.Velocity, percentOutput);
    }

    /**
     * Inverts the Talons and Victors to account for wiring inconsistencies (must be
     * tested).
     */
    private void dtInvertControllers()
    {
        getLeftTalon().setInverted(true);
        getLeftVictor().setInverted(true);
        
        // Invert the following direction (left Talons and Victors were wired
        // oppositely)
        getRightTalon().setInverted(false);
        getRightVictor().setInverted(false);
    }

    /**
     * Sets the neutral mode to either coast or brake.
     * 
     * @param n
     *            either coast or brake
     */
    private void dtSetNeutralMode(NeutralMode n)
    {
        getLeftTalon().setNeutralMode(n);
        getLeftVictor().setNeutralMode(n);
        getRightTalon().setNeutralMode(n);
        getRightVictor().setNeutralMode(n);
    }

    /**
     * Configure the talons to ramp gradually to peak voltage.
     * 
     * @param t
     *            the ramp time (time from 0 voltage to max)
     */
    private void dtSetRampTime(double t)
    {
        getLeftTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
        getRightTalon().configOpenloopRamp(t, RobotMap.TIMEOUT);
    }

    /**
     * Scales the voltage to modify percent output to account for the battery
     * voltage loss.
     * 
     * @param nomVoltage
     *            the assumed nominal voltage, used to scale relative to battery
     *            voltage
     */
    private void dtScaleVoltage(double nomVoltage)
    {
        getLeftTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        getRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        getLeftTalon().enableVoltageCompensation(true);
        getRightTalon().enableVoltageCompensation(true);
    }

    /**
     * Configures the encoders either to use pulse width or quadrature velocity.
     * 
     * @param f
     *            the feedback device to use to configure (either quadrature or
     *            sensor velocity)
     */
    public void velocityConfigureSensors(FeedbackDevice f)
    {
        getLeftTalon().configSelectedFeedbackSensor(f, RobotMap.DT_VEL_PID, RobotMap.TIMEOUT);
        getRightTalon().configSelectedFeedbackSensor(f, RobotMap.DT_VEL_PID, RobotMap.TIMEOUT);
    }

    /**
     * Aligns the sensor phase of the encoders to match the motions of the motors.
     */
    private void dtSetSensorPhase()
    {
        getLeftTalon().setSensorPhase(RobotMap.DT_LEFT_TALON_PHASE);
        getRightTalon().setSensorPhase(RobotMap.DT_RIGHT_TALON_PHASE);
    }

    /**
     * Configures the drivetrain velocity closed loop.
     * 
     * @postcondition nominal and peak output in both directions has been set
     */
    private void enableVelocityClosedLoop()
    {
        velocityConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        getLeftTalon().configNominalOutputForward(RobotMap.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configNominalOutputForward(RobotMap.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().configNominalOutputReverse(-1 * RobotMap.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configNominalOutputReverse(-1 * RobotMap.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().configPeakOutputForward(RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);
        getRightTalon().configPeakOutputForward(RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);

        getLeftTalon().configPeakOutputReverse(-1 * RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);
        getRightTalon().configPeakOutputReverse(-1 * RobotMap.DRIVETRAIN_SCALE, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(RobotMap.DT_VEL_PID, RobotMap.VEL_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(RobotMap.DT_VEL_PID, RobotMap.VEL_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(RobotMap.DT_VEL_PID, RobotMap.VEL_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(RobotMap.DT_VEL_PID, RobotMap.VEL_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(RobotMap.DT_VEL_PID, RobotMap.VEL_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.DT_VEL_PID, RobotMap.VEL_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(RobotMap.DT_VEL_PID, RobotMap.VEL_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.DT_VEL_PID, RobotMap.VEL_KD_RIGHT, RobotMap.TIMEOUT);
      
    }


    /**
     * Configures the drivetrain position closed loop.
     * 
     * @postcondition each talon has configured F, P, I, D and D constants, the
     *                encoders have been zeroed, and the allowable error has been
     *                set
     */
    private void dtConfigurePositionClosedLoop()
    {
        getLeftTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);
        getRightTalon().setSelectedSensorPosition(0, RobotMap.POS_PID, RobotMap.TIMEOUT);

        getLeftTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        getRightTalon().configAllowableClosedloopError(RobotMap.POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(RobotMap.POS_PID, RobotMap.POS_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(RobotMap.POS_PID, RobotMap.POS_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.POS_PID, RobotMap.POS_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.POS_PID, RobotMap.POS_KD_RIGHT, RobotMap.TIMEOUT);
    }
    
    private void dtConfigureMotionProfileClosedLoop()
    {
        //getLeftTalon().setSelectedSensorPosition(0, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
        //getRightTalon().setSelectedSensorPosition(0, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);

        getLeftTalon().configAllowableClosedloopError(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        getRightTalon().configAllowableClosedloopError(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.DT_MOTION_PROF_KD_RIGHT, RobotMap.TIMEOUT);
    }

    /**
     * Sets a current limit for the left and right talons on the drivetrain.
     * 
     * @param peakCurrentLimit
     *            the peak current limit (only temporary)
     * @param peakTime
     *            the time for which the peak current limit is allowed
     * @param continuousLimit
     *            the continuous limit (represents the limit after peakTime passes)
     */
    private void dtSetCurrentLimit(int peakCurrentLimit, int peakTime, int continuousLimit)
    {
        getLeftTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);
        getRightTalon().configPeakCurrentLimit(peakCurrentLimit, RobotMap.TIMEOUT);

        getLeftTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);
        getRightTalon().configPeakCurrentDuration(peakTime, RobotMap.TIMEOUT);

        getLeftTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);
        getRightTalon().configContinuousCurrentLimit(continuousLimit, RobotMap.TIMEOUT);

        getLeftTalon().enableCurrentLimit(true);
        getRightTalon().enableCurrentLimit(true);
    }
    
    /**
     * Gets the left Talon on the drivetrain.
     * @return the left Talon
     */
    public TalonSRX getLeftTalon() { return leftTalon; }

    /**
     * Gets the right Talon on the drivetrain.
     * @return the right Talon
     */
    public TalonSRX getRightTalon() { return rightTalon; }

    /**
     * Gets the left Victor on the drivetrain.
     * @return the left Victor
     */
    public VictorSPX getLeftVictor() { return leftVictor; }
    
    /**
     * Gets the right Victor on the drivetrain.
     * @return the right Victor.
     */
    public VictorSPX getRightVictor() { return rightVictor; }

    /**
     * Gets the instance of this Drivetrain, creating a new one if necessary.
     * @return the instance of this singleton class
     */
    public static Drivetrain getInstance()
    {
        if (instance == null) instance = new Drivetrain();
        return instance;
    }
    
    public static void resetInstance()
    {
        instance = null;
    }
    
    /**
     * Gets the pigeon for this drivetrain.
     * @return the pigeon
     */
    public PigeonIMU getPigeon()
    {
        return pigeon;
    }
    
    public double getPigeonYaw()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }
    
    public double getPigeonPitch()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[1];
    }
    
    public double getPigeonRoll()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[2];
    }

}
