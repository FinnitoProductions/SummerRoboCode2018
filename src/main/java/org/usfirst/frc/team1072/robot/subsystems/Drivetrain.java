package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.CAN_IDs;
import org.usfirst.frc.team1072.robot.commands.drivetrain.DriveWithVelocity;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.Pigeon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a drive train with two Talons and two Victors.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class Drivetrain extends Subsystem
{

	/**
     * The current instance of this singleton Drivetrain.
     */
    private static Drivetrain instance = null;
    
    /**
     * The left Talon on the drivetrain.
     */
    private TalonSRX leftTalon;
     
    /**
     * The right Talon on the drivetrain.
     */
    private TalonSRX rightTalon;
    
    /**
     * The left Victor on the drivetrain.
     */
    private VictorSPX leftVictor;
    
    /**
     * The right Victor on the drivetrain.
     */
    private VictorSPX rightVictor;
    
    /**
     * The Pigeon IMU for use on the drivetrain.
     */
    private PigeonIMU pigeon;
    
    /**
     * Initializes this subsystem.
     */
    private Drivetrain()
    {
        // initialize talons
        leftTalon = new TalonSRX (CAN_IDs.LEFT_CIM_TALON);
        rightTalon = new TalonSRX (CAN_IDs.RIGHT_CIM_TALON);
        leftVictor = new VictorSPX (CAN_IDs.LEFT_CIM_VICTOR);
        rightVictor = new VictorSPX (CAN_IDs.RIGHT_CIM_VICTOR);
        pigeon = new PigeonIMU(CAN_IDs.PIGEON);
    }
  
    /**
     * Initializes the command using the necessary default command.
     */
    public void initDefaultCommand()
    {
        setDefaultCommand(new DriveWithVelocity(OI.BLACK_XBOX_DRIVE_DEADBAND));
        //setDefaultCommand(new TurnToAngle(90));
    }
    
    /**
     * Runs both motor controllers given a velocity and turn.
     * @param speed the speed at which the controllers should run
     * @param turn the amount by which each speed should be modified to account for turn
     */
    public void arcadeDriveVelocity(double speed, double turn)
    {
        // victor follows talon
        rightTalon.set(ControlMode.Velocity, speed + turn);  
        leftTalon.set(ControlMode.Velocity, speed - turn);  

    }
    
    /**
     * Moves the robot to a given position using PID.
     * @param target the target position 
     */
    public void arcadeDrivePosition (double target)
    {
        rightTalon.set(ControlMode.Position, target);
    }
    
    /**
     * Moves the robot left and right sides to given positions using PID.
     * @param leftTarget the target position for the left side
     * @param rightTarget the target position for the right side
     */
    public void arcadeDrivePosition (double leftTarget, double rightTarget)
    {        
        leftTalon.set(ControlMode.Position, leftTarget);
        rightTalon.set(ControlMode.Position, rightTarget);
    }
    
    /**
     * Performs autonomous-specific commands on the Talon.
     */
    public void talonInitAutonomous()
    {
        talonInit();
    }
    
    /**
     * Performs teleop-specific commands on the Talon.
     */
    public void talonInitTeleop()
    {
        talonInit();
        configureVelocityClosedLoop();
        setRampTime(Drivetrain.MAX_RAMP_TIME);
        
    }
    /**
     * Performs all commands to initialize the talons.
     * 
     * @postcondition the talons are in the correct state to begin robot operation
     */
    public void talonInit()
    {
        zeroAllSensors();
        zeroPigeon();
        setTalonDeadbands();
        initTalonOutput(0);

        clearTrajectoryPoints();

        victorInit();
        
        invertControllers();
        setNeutralMode(NeutralMode.Brake);
        
        ;
        scaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);
        configureNominalPeakOutputs();
        
        ;
        dtsetTalonFramePeriods();
        
        configurePositionClosedLoop();
        configureMotionProfileDriveClosedLoop();

        dtSetCurrentLimit(Drivetrain.PEAK_CURRENT_LIMIT, Drivetrain.PEAK_TIME_MS,
                Drivetrain.CONTINUOUS_CURRENT_LIMIT);

    }
    
    /**
     * Configures the deadbands for the Talons (the output below which the output is essentially zero).
     */
    private void setTalonDeadbands()
    {
        getLeftTalon().configNeutralDeadband(Drivetrain.TALON_DEADBAND, RobotMap.TIMEOUT);
        getRightTalon().configNeutralDeadband(Drivetrain.TALON_DEADBAND, RobotMap.TIMEOUT);
    }

    /**
     * Sets both talons to a given value.
     * @param cm the ControlMode to which both talons will be set (like PercentOutput, Velocity, Position, or Disabled)
     * @param value the value to which both talons will be set
     */
    public void setBoth (ControlMode cm, double value)
    {
        setLeft(cm, value);
        setRight(cm, value);
    }
    
    /**
     * Sets the left talon to a given value.
     * @param cm the ControlMode to which the left talon will be set 
     * (like PercentOutput, Velocity, Position, or Disabled)
     * @param value the value to which the left talon will be set
     */
    public void setLeft (ControlMode cm, double value)
    {
        Robot.dt.getLeftTalon().set(cm, value);
    }
    
    /**
     * Sets the right talon to a given value.
     * @param cm the ControlMode to which the right talon will be set 
     * (like PercentOutput, Velocity, Position, or Disabled)
     * @param value the value to which the right talon will be set
     */
    public void setRight (ControlMode cm, double value)
    {
        Robot.dt.getRightTalon().set(cm, value);
    }
    
    /**
     * Sets both talon sensor positions to a given value in a given PID loop.
     * @param value the value to which both will be set
     * @param pidLoop the loop index (primary/auxiliary) [0,1]
     */
    public void setBothSensorPositions (int value, int pidLoop)
    {
        getLeftTalon().setSelectedSensorPosition(value, pidLoop, RobotMap.TIMEOUT);
        getRightTalon().setSelectedSensorPosition(value, pidLoop, RobotMap.TIMEOUT);
    }
    
    /**
     * Clears trajectory points from both talons.
     */
    public void clearTrajectoryPoints()
    {
        Robot.dt.getLeftTalon().clearMotionProfileTrajectories();
        Robot.dt.getRightTalon().clearMotionProfileTrajectories();
    }
    
    /**
     * Initializes the talon output to a given value.
     * @param output the output to which both talons will be initialized
     */
    public void initTalonOutput(double output)
    {
        setBoth(ControlMode.PercentOutput, output);
    }

    /**
     * Slaves the Victors to directly follow the behavior of their parent talons.
     */
    private void victorInit()
    {
        getLeftVictor().follow(getLeftTalon());
        getRightVictor().follow(getRightTalon());
    }
    
    /**
     * Resets the coefficients in all four PID slots on the Talons.
     */
    public void resetTalonCoefficients()
    {
        // for all PID slots
        for (int slot = 0; slot < 2; slot++)
        {
            resetTalonCoefficients(slot);
        }
    }
    
    /**
     * Resets the coefficients stored in a given PID slot.
     * @param pid_slot the slot where the coefficients will be reset
     */
    public void resetTalonCoefficients (int pid_slot)
    {
        getLeftTalon().configSelectedFeedbackCoefficient(1,
                pid_slot, RobotMap.TIMEOUT); 
        getRightTalon().configSelectedFeedbackCoefficient(1,
                pid_slot, RobotMap.TIMEOUT); 
    }
    
    /**
     * Zeroes all sensors currently configured for the Talons.
     */
    private void zeroAllSensors()
    {
        getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    }
   

    /**
     * Inverts the Talons and Victors to account for wiring inconsistencies (must be
     * tested).
     */
    private void invertControllers()
    {
        getLeftTalon().setInverted(Drivetrain.LEFT_TALON_INVERT);
        getLeftVictor().setInverted(Drivetrain.LEFT_VICTOR_INVERT);
        
        // Invert the following direction (left Talons and Victors were wired
        // oppositely)
    }

    /**
     * Sets the neutral mode to either coast or brake.
     * 
     * @param n
     *            either coast or brake
     */
    private void setNeutralMode(NeutralMode n)
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
    private void setRampTime(double t)
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
    private void scaleVoltage(double nomVoltage)
    {
        getLeftTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        getRightTalon().configVoltageCompSaturation(nomVoltage, RobotMap.TIMEOUT);
        getLeftTalon().enableVoltageCompensation(true);
        getRightTalon().enableVoltageCompensation(true);
    }


    /**
     * Aligns the sensor phase of the encoders to match the motions of the motors.
     * @param leftPhase the left sensor phase 
     * @param rightPhase the right sensor phase
     */
    public void setTalonSensorPhase(boolean leftPhase, boolean rightPhase)
    {
        getLeftTalon().setSensorPhase(leftPhase);
        getRightTalon().setSensorPhase(rightPhase);
    }

    /**
     * Configures the drivetrain velocity closed loop.
     * 
     * @postcondition nominal and peak output in both directions has been set
     */
    private void configureVelocityClosedLoop()
    {
        getLeftTalon().config_kF(Drivetrain.VEL_PID, Drivetrain.VEL_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(Drivetrain.VEL_PID, Drivetrain.VEL_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(Drivetrain.VEL_PID, Drivetrain.VEL_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(Drivetrain.VEL_PID, Drivetrain.VEL_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(Drivetrain.VEL_PID, Drivetrain.VEL_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(Drivetrain.VEL_PID, Drivetrain.VEL_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(Drivetrain.VEL_PID, Drivetrain.VEL_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(Drivetrain.VEL_PID, Drivetrain.VEL_KD_RIGHT, RobotMap.TIMEOUT);
      
    }

    /**
     * Configures the nominal and peak outputs for the two Talons, where nominal output is the minimum percent which can be applied (without breaking static friction)
     * and peak output is the maximum percent which can be applied.
     */
    public void configureNominalPeakOutputs()
    {
        getLeftTalon().configNominalOutputForward(Drivetrain.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configNominalOutputForward(Drivetrain.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().configNominalOutputReverse(-1 * Drivetrain.NOMINAL_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configNominalOutputReverse(-1 * Drivetrain.NOMINAL_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().configPeakOutputForward(Drivetrain.PEAK_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configPeakOutputForward(Drivetrain.PEAK_OUTPUT_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().configPeakOutputReverse(-1 * Drivetrain.PEAK_OUTPUT_LEFT, RobotMap.TIMEOUT);
        getRightTalon().configPeakOutputReverse(-1 * Drivetrain.PEAK_OUTPUT_RIGHT, RobotMap.TIMEOUT);
    }
    /**
     * Configures the drivetrain position closed loop.
     * 
     * @postcondition each talon has configured F, P, I, D and D constants, the
     *                encoders have been zeroed, and the allowable error has been
     *                set
     */
    private void configurePositionClosedLoop()
    {
        getLeftTalon().configAllowableClosedloopError(Drivetrain.POS_PID, Drivetrain.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        getRightTalon().configAllowableClosedloopError(Drivetrain.POS_PID, Drivetrain.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(Drivetrain.POS_PID, Drivetrain.POS_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(Drivetrain.POS_PID, Drivetrain.POS_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(Drivetrain.POS_PID, Drivetrain.POS_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(Drivetrain.POS_PID, Drivetrain.POS_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(Drivetrain.POS_PID, Drivetrain.POS_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(Drivetrain.POS_PID, Drivetrain.POS_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(Drivetrain.POS_PID, Drivetrain.POS_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(Drivetrain.POS_PID, Drivetrain.POS_KD_RIGHT, RobotMap.TIMEOUT);
        
        getLeftTalon().config_IntegralZone(Drivetrain.POS_PID, Drivetrain.POS_IZONE_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_IntegralZone(Drivetrain.POS_PID, Drivetrain.POS_IZONE_RIGHT, RobotMap.TIMEOUT);
    }
    
    /**
     * Selects the profile slots of both talons to their correct places.
     * @param pidSlot the PID slot on the Talon [0, 3]
     * @param pidIndex the PID index (primary/auxiliary or inner/outer) [0, 1]
     */
    public void selectProfileSlots(int pidSlot, int pidIndex)
    {
        getLeftTalon().selectProfileSlot(pidSlot, pidIndex);
        getRightTalon().selectProfileSlot(pidSlot, pidIndex);
    }
    
    /**
     * Configures the angle closed loop for turning in place.
     */
    public void configureAngleClosedLoop()
    {
        getPigeon().setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Drivetrain.Pigeon.PERIOD_MS, RobotMap.TIMEOUT);
        
        getRightTalon().config_kF(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KF, RobotMap.TIMEOUT);
        getRightTalon().config_kP(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KP, RobotMap.TIMEOUT);
        getRightTalon().config_kI(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KI, RobotMap.TIMEOUT);
        getRightTalon().config_kD(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KD, RobotMap.TIMEOUT);
        
        getRightTalon().configMotionCruiseVelocity(Drivetrain.Pigeon.TURN_VEL, RobotMap.TIMEOUT);
        getRightTalon().configMotionAcceleration(Drivetrain.Pigeon.TURN_ACCEL, RobotMap.TIMEOUT);
        
        getRightTalon().config_IntegralZone(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_IZONE, RobotMap.TIMEOUT);
        
        
        getLeftTalon().config_kF(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KF, RobotMap.TIMEOUT);
        getLeftTalon().config_kP(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KP, RobotMap.TIMEOUT);
        getLeftTalon().config_kI(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KI, RobotMap.TIMEOUT);
        getLeftTalon().config_kD(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_KD, RobotMap.TIMEOUT);
        
        getLeftTalon().configMotionCruiseVelocity(Drivetrain.Pigeon.TURN_VEL, RobotMap.TIMEOUT);
        getLeftTalon().configMotionAcceleration(Drivetrain.Pigeon.TURN_ACCEL, RobotMap.TIMEOUT);
        
        getLeftTalon().config_IntegralZone(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.TURN_IZONE, RobotMap.TIMEOUT);
    }
    
    /**
     * Configures the angle closed loop for motion profiling.
     * 
     * @postcondition P, I, and D have been set for both sides; the pigeon zeroed; the frame period set
     */
    public void configureMotionProfileAngleClosedLoop()
    {
        getPigeon().setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Drivetrain.Pigeon.PERIOD_MS, RobotMap.TIMEOUT);
        
        getRightTalon().config_kF(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KF, RobotMap.TIMEOUT);
        getRightTalon().config_kP(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KP, RobotMap.TIMEOUT);
        getRightTalon().config_kI(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KI, RobotMap.TIMEOUT);
        getRightTalon().config_kD(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KD, RobotMap.TIMEOUT);
        
        getLeftTalon().config_kF(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KF, RobotMap.TIMEOUT);
        getLeftTalon().config_kP(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KP, RobotMap.TIMEOUT);
        getLeftTalon().config_kI(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KI, RobotMap.TIMEOUT);
        getLeftTalon().config_kD(Drivetrain.ANGLE_PID, Drivetrain.Pigeon.MOT_PROF_KD, RobotMap.TIMEOUT);

    }

    /**
     * Configures the motion profile closed loop.
     * 
     * @postcondition P, I, and D have been set for both sides; the allowable error configured
     */
    public void configureMotionProfileDriveClosedLoop()
    {
        getLeftTalon().configAllowableClosedloopError(Drivetrain.MOTION_PROFILE_PID, Drivetrain.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        getRightTalon().configAllowableClosedloopError(Drivetrain.MOTION_PROFILE_PID, Drivetrain.POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(Drivetrain.MOTION_PROFILE_PID, Drivetrain.MOTION_PROF_KD_RIGHT, RobotMap.TIMEOUT);
    }
    
    
    /**
     * Resets the Talon frame periods (the frequency at which the Talons will broadcast certain status frames along the CAN bus).
     */
    private void dtsetTalonFramePeriods()
    {
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 3, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, RobotMap.TIMEOUT);
        
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 3, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 20, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, RobotMap.TIME_PER_TRAJECTORY_POINT_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, RobotMap.TIMEOUT);

        
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
    /**
     * Gets the pigeon for this drivetrain.
     * @return the pigeon
     */
    public PigeonIMU getPigeon()
    {
        return pigeon;
    }
    
    /**
     * Gets the current yaw value of the pigeon.
     * @return the yaw
     */
    public double getPigeonYaw()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }
    
    /**
     * Gets the current pitch value of the pigeon.
     * @return the pitch
     */
    public double getPigeonPitch()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[1];
    }
    
    /**
     * Gets the current roll value of the pigeon.
     * @return the roll
     */
    public double getPigeonRoll()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[2];
    }
    
    /**
     * Sets the pigeon yaw to a given value.
     * @param angle the angle value to which the pigeon should be set, in pigeon units 
     * where 1 rotation is 8192 units
     */
    public void setPigeonYaw(double angle)
    {
        getPigeon().setYaw(angle * 64, RobotMap.TIMEOUT);
    }
    
    /**
     * Adds a given value to the pigeon yaw.
     * @param angle the angle value which should be added to the pigeon yaw value, in pigeon units 
     * where 1 rotation is 8192 units
     */
    public void addPigeonYaw (double angle)
    {
        getPigeon().addYaw(angle * 64, RobotMap.TIMEOUT);
    }
 
    /**
     * Zeros the pigeon.
     */
    public void zeroPigeon()
    {
        getPigeon().setYaw(0, RobotMap.TIMEOUT);
        getPigeon().setAccumZAngle(0, RobotMap.TIMEOUT);
    }
    
    /**
     * Configures both Talons to point to a given sensor.
     * @param fd the type of sensor to which the Talon should use
     * @param pidLoop the loop index where this sensor should be placed [0,1]
     */
    public void configBothFeedbackSensors(FeedbackDevice fd, int pidLoop)
    {
        getLeftTalon().configSelectedFeedbackSensor(fd, 
                    pidLoop, RobotMap.TIMEOUT);
        getRightTalon().configSelectedFeedbackSensor(fd, 
                    pidLoop, RobotMap.TIMEOUT);
    }
    
    /**
     * Prints the current output percentage to the motors to SmartDashboard.
     */
    public void printMotorOutputPercentage()
    {
        SmartDashboard.putNumber("Left Talon Output Percentage", Robot.dt.getLeftTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("Right Talon Output Percentage", Robot.dt.getRightTalon().getMotorOutputPercent());
    }
    
    /**
     * Prints the closed loop error of the Talons in a given loop.
     * @param pidLoop the loop index [0,1]
     */
    public void printClosedLoopError (int pidLoop)
    {
        SmartDashboard.putNumber("Left Talon Closed Loop Error " + (pidLoop == 0 ? "Primary" : "Auxiliary"), Robot.dt.getLeftTalon().getClosedLoopError(pidLoop));
        SmartDashboard.putNumber("Right Talon Closed Loop Error " + (pidLoop == 0 ? "Primary" : "Auxiliary"), Robot.dt.getRightTalon().getClosedLoopError(pidLoop));
    }
    
    /**
     * Prints the sensor positions of the Talons in a given loop.
     * @param pidLoop the loop index [0,1]
     */
    public void printSensorPositions (int pidLoop)
    {
        SmartDashboard.putNumber("Left Talon Position " + (pidLoop == 0 ? "Primary" : "Auxiliary"), Robot.dt.getLeftTalon().getSelectedSensorPosition(pidLoop));
        SmartDashboard.putNumber("Right Talon Position" + (pidLoop == 0 ? "Primary" : "Auxiliary"), Robot.dt.getRightTalon().getSelectedSensorPosition(pidLoop));
    }
    
    /**
     * Determines whether the closed loop error for both sides is within a given value.
     * @param loopIndex the loop index, either primary or auxiliary [0,1]
     * @param allowableError the error tolerance to be checked
     * @return true if the absolute value of the error is within the value; false otherwise
     */
    public boolean isClosedLoopErrorWithin (int loopIndex, double allowableError)
    {
        return Math.abs(getLeftTalon().getClosedLoopError(loopIndex)) < allowableError
                && Math.abs(getRightTalon().getClosedLoopError(loopIndex)) < allowableError;
    }
    
    
    /**
     * The velocity amount to which all motion profile trajectories will be added to more easily
     * break static friction at the beginning of the profile.
     */
    public static final double MOT_PROF_ADD_TO_VEL_INIT = 7;
    
    /**
     * The time required to overcome static friction to allow for easier ramping.
     */
    public static final double TIME_TO_OVERCOME_S_FRICTION_MS = 45;
    
    /**
     * The Talon deadband (the output at which the Talon assumes zero output).
     */
    public static final double TALON_DEADBAND = 0.01;
    
    /**
     * Whether the invert the left Talon.
     */
    public static final boolean LEFT_TALON_INVERT = true;
    
    /**
     * Whether to invert the left Victor.
     */
    public static final boolean LEFT_VICTOR_INVERT = true;
    
    /**
     * The PID slot for velocity PID constants.
     */
    public static int VEL_PID = 1;
    
    /**
     * The PID slot for position PID constants.
     */
    public static int POS_PID = 2;
    
    /**
     * The PID slot for angle PID constants.
     */
    public static int ANGLE_PID = 0;
    
    /**
     * The PID slot for motion profile PID constants.
     */
    public static int MOTION_PROFILE_PID = 3;
    
    /**
     * The time for which the drivetrain should ramp up when beginning.
     */
    public static double MAX_RAMP_TIME = 0;
    
    /**
     * The nominal output (or the constant output percemt such that static friction is more easily broken)
     * for the left.
     */
    public static double NOMINAL_OUTPUT_LEFT = 0.1; //0.084;
    
    /**
     * The nominal output (or the constant output percent such that static friction is more easily broken)
     * for the right.
     */
    public static double NOMINAL_OUTPUT_RIGHT = 0.1; //0.084;
    
    /**
     * The peak output (or the maximum possible output percent) for the left.
     */
    public static double PEAK_OUTPUT_LEFT = 1;
    
    /**
     * The peak output (or the maximum possible output percent) for the right.
     */
    public static double PEAK_OUTPUT_RIGHT = 1;
    
    /**
     * The sensor phase for the left Talon when driving straight.
     */
    public static boolean LEFT_TALON_PHASE = false;
    
    /**
     * The sensor phase for the right Talon when driving straight.
     */
    public static boolean RIGHT_TALON_PHASE = false;
    
 // CONCRETE CONSTANTS
    /*public static double VEL_KF_LEFT = .197; //0.178
    // when modifying KP, double until disastrous
    public static double VEL_KP_LEFT = 0; //0.3;
    public static double VEL_KI_LEFT = 0;//0;
    public static double VEL_KD_LEFT = 0; //10;
    
    public static double VEL_KF_RIGHT = .188; //0.18
    public static double VEL_KP_RIGHT = 0;//0.3;
    public static double VEL_KI_RIGHT = 0;//0;
    public static double VEL_KD_RIGHT = 0;//10;*/
    // CARPET CONSTANTS
    
    
   
    /**
     * The practice bot F constant for the velocity closed loop on the left.
     */
    public static double VEL_KF_LEFT_PRACTICE = 0.197;
    
    /**
     * The practice bot P constant for the velocity closed loop on the left.
     */
    public static double VEL_KP_LEFT_PRACTICE = 0.5;
    
    /**
     * The practice bot I constant for the velocity closed loop on the left.
     */
    public static double VEL_KI_LEFT_PRACTICE = 0;
    
    /**
     * The practice bot D constant for the velocity closed loop on the left.
     */
    public static double VEL_KD_LEFT_PRACTICE = 0; 
    
    /**
     * The practice bot F constant for the velocity closed loop on the right.
     */
    public static double VEL_KF_RIGHT_PRACTICE = 0.197; 
    
    /**
     * The practice bot P constant for the velocity closed loop on the right.
     */
    public static double VEL_KP_RIGHT_PRACTICE = 0.5;//0.3;
    
    /**
     * The practice bot I constant for the velocity closed loop on the right.
     */
    public static double VEL_KI_RIGHT_PRACTICE = 0;//0;
    
    /**
     * The practice bot D constant for the velocity closed loop on the right.
     */
    public static double VEL_KD_RIGHT_PRACTICE = 0;//10;
    
    /**
     * The competition bot F constant for the velocity closed loop on the left.
     */
    public static double VEL_KF_LEFT_COMP = 0.197;
    
    /**
     * The competition bot P constant for the velocity closed loop on the left.
     */
    public static double VEL_KP_LEFT_COMP = 0.5;
    
    /**
     * The competition bot I constant for the velocity closed loop on the left.
     */
    public static double VEL_KI_LEFT_COMP = 0;
    
    /**
     * The competition bot D constant for the velocity closed loop on the left.
     */
    public static double VEL_KD_LEFT_COMP = 0; 
    
    /**
     * The competition bot F constant for the velocity closed loop on the right.
     */
    public static double VEL_KF_RIGHT_COMP = 0.197; 
    
    /**
     * The competition bot P constant for the velocity closed loop on the right.
     */
    public static double VEL_KP_RIGHT_COMP = 0.5;//0.3;
    
    /**
     * The competition bot I constant for the velocity closed loop on the right.
     */
    public static double VEL_KI_RIGHT_COMP = 0;//0;
    
    /**
     * The competition bot D constant for the velocity closed loop on the right.
     */
    public static double VEL_KD_RIGHT_COMP = 0;//10;
    
    /**
     * The F constant for the velocity closed loop on the left.
     */
    public static double VEL_KF_LEFT = RobotMap.IS_COMP ? VEL_KF_LEFT_COMP : VEL_KF_LEFT_PRACTICE;
    
    /**
     * The P constant for the velocity closed loop on the left.
     */
    public static double VEL_KP_LEFT = RobotMap.IS_COMP ? VEL_KP_LEFT_COMP : VEL_KP_LEFT_PRACTICE;
    
    /**
     * The I constant for the velocity closed loop on the left.
     */
    public static double VEL_KI_LEFT = RobotMap.IS_COMP ? VEL_KI_LEFT_COMP : VEL_KI_LEFT_PRACTICE;
    
    /**
     * The D constant for the velocity closed loop on the left.
     */
    public static double VEL_KD_LEFT = RobotMap.IS_COMP ? VEL_KD_LEFT_COMP : VEL_KD_LEFT_PRACTICE;
    
    /**
     * The F constant for the velocity closed loop on the right.
     */
    public static double VEL_KF_RIGHT = RobotMap.IS_COMP ? VEL_KF_RIGHT_COMP : VEL_KF_RIGHT_PRACTICE;
    
    /**
     * The P constant for the velocity closed loop on the right.
     */
    public static double VEL_KP_RIGHT = RobotMap.IS_COMP ? VEL_KP_RIGHT_COMP : VEL_KP_RIGHT_PRACTICE;
    
    /**
     * The I constant for the velocity closed loop on the right.
     */
    public static double VEL_KI_RIGHT = RobotMap.IS_COMP ? VEL_KI_RIGHT_COMP : VEL_KI_RIGHT_PRACTICE;
    
    /**
     * The D constant for the velocity closed loop on the right.
     */
    public static double VEL_KD_RIGHT = RobotMap.IS_COMP ? VEL_KD_RIGHT_COMP : VEL_KD_RIGHT_PRACTICE;

    // CONCRETE
//    public static double POS_KF_LEFT = 0;
//    public static double POS_KP_LEFT = 0.1; //0.2 
//    public static double POS_KI_LEFT = 0.000004; //
//    public static double POS_KD_LEFT = 25;
//    
//    public static double POS_KF_RIGHT = 0;
//    public static double POS_KP_RIGHT = 0.1;
//    public static double POS_KI_RIGHT = 0.000004; // 
//    public static double POS_KD_RIGHT = 25;
    // CARPET 
    
    /**
     * The F constant for the position closed loop on the left.
     */
    public static double POS_KF_LEFT = 0;
    
    /**
     * The P constant for the position closed loop on the left.
     */
    public static double POS_KP_LEFT = 0.19;//0.22; //0.2 
    
    /**
     * The I constant for the position closed loop on the left.
     */
    public static double POS_KI_LEFT = 0.0001;//0.00001;//0.001;//.0008;//0.00001;//0.00001; 
    
    /**
     * The D constant for the position closed loop on the left.
     */
    public static double POS_KD_LEFT = 30;//40;//40;
    
    /**
     * The integral zone constant for the position closed loop on the left.
     */
    public static int POS_IZONE_LEFT = 250;
    
    /**
     * The F constant for the position closed loop on the right.
     */
    public static double POS_KF_RIGHT = 0;
    
    /**
     * The P constant for the position closed loop on the right.
     */
    public static double POS_KP_RIGHT = 0.19;
    
    /**
     * The I constant for the position closed loop on the right.
     */
    public static double POS_KI_RIGHT = 0.0001;//0.00001;//0.001;//0.00001;//0.00001;
    
    /**
     * The D constant for the position closed loop on the right.
     */
    public static double POS_KD_RIGHT = 15;//30;//40;
    
    /**
     * The integral zone constant for the position closed loop on the right.
     */
    public static int POS_IZONE_RIGHT = 250;
   
    /**
     * The allowable error for the position closed loop.
     */
    public static int POS_ALLOWABLE_ERROR = 500;

    /**
     * The wheel diameter on this drivetrain.
     */
    public static double WHEELDIAMETER = 4.0;
    
    /**
     * The maximum allowed drive speed for the drivetrain.
     */
    public static double MAX_DRIVE_SPEED_FPS = 14.0;
    
    /**
     * The maximum allowed turn speed for the drivetrain.
     */
    public static double MAX_TURN_SPEED_FPS = 14;
    
    /**
     * The peak current limit for the drivetrain.
     */
    public static int PEAK_CURRENT_LIMIT = 60;
    
    /**
     * The time for which the peak currrent limit is allowed (in ms).
     */
    public static int PEAK_TIME_MS = 200;
    
    /**
     * The continuous current limit for the drivetrain.
     */
    public static int CONTINUOUS_CURRENT_LIMIT = 40;
    
    
    // motion profiling constants
    // CONCRETE
//    public static double DT_MOTION_PROF_KF_LEFT = .182; //0.197
//    public static double DT_MOTION_PROF_KP_LEFT = 0.1; //0.2 
//    public static double DT_MOTION_PROF_KI_LEFT = 0.000004; //
//    public static double DT_MOTION_PROF_KD_LEFT = 25;
//    
//    public static double DT_MOTION_PROF_KF_RIGHT = .178; //0.188
//    public static double DT_MOTION_PROF_KP_RIGHT = 0.1;
//    public static double DT_MOTION_PROF_KI_RIGHT = 0.000004; // 
//    public static double DT_MOTION_PROF_KD_RIGHT = 25;
    // CARPET
    
    /**
     * The F constant for the motion profile closed loop on the left.
     */
    public static double MOTION_PROF_KF_LEFT = 0;
    
    /**
     * The P constant for the motion profile closed loop on the left.
     */
    public static double MOTION_PROF_KP_LEFT = 0;
    
    /**
     * The I constant for the motion profile closed loop on the left.
     */
    public static double MOTION_PROF_KI_LEFT = 0;
    
    /**
     * The D constant for the motion profile closed loop on the left.
     */
    public static double MOTION_PROF_KD_LEFT = 0;
    
    /**
     * The F constant for the motion profile closed loop on the right.
     */
    public static double MOTION_PROF_KF_RIGHT = 0.25;//0.22;//0.235;
    
    /**
     * The P constant for the motion profile closed loop on the right.
     */
    public static double MOTION_PROF_KP_RIGHT = 1.2;//0.8;
    
    /**
     * The I constant for the motion profile closed loop on the right.
     */
    public static double MOTION_PROF_KI_RIGHT = 0;//0.0001;
    
    /**
     * The D constant for the motion profile closed loop on the right.
     */
    public static double MOTION_PROF_KD_RIGHT = 4;

    /**
     * The allowable error for a motion profile closed loop.
     */
    public static final int MOTION_PROFILE_ALLOWABLE_ERROR = 500;
    
    /**
   	 * A wrapper class to house all of the Pigeon-related constants.
   	 * @author Finn Frankis
   	 * @version 7/5/18
   	 */
   	public static class Pigeon
   	{
   	    /**
   	     * The F constant for pigeon motion profiling.
   	     */
   	    public static double MOT_PROF_KF = 0;
   	    
   	    /**
   	     * The P constant for pigeon motion profiling.
   	     */
   	    public static double MOT_PROF_KP = 0.8;//1.2;//2.6;//1.3;//1.1;
   	    
   	    /**
   	     * The I constant for pigeon motion profiling.
   	     */
   	    public static double MOT_PROF_KI = 0;
   	    
   	    /**
   	     * The D constant for pigeon motion profiling.
   	     */
   	    public static double MOT_PROF_KD = 5;//6;//15;
   	    
   	    /**
   	     * The F constant for pigeon turning in place.
   	     */
   	    public static double TURN_KF = 0.5;
   	    
   	    /**
   	     * The P constant for pigeon turning in place.
   	     */
   	    public static double TURN_KP = 3.6; 
   	    
   	    /**
   	     * The I constant for pigeon turning in place.
   	     */
   	    public static double TURN_KI = 0.003;
   	    
   	    /**
   	     * The D constant for pigeon turning in place.
   	     */
   	    public static double TURN_KD = 1; 
   	    
   	    /**
   	     * The integral zone constant for pigeon turning in place.
   	     */
   	    public static int TURN_IZONE = 150; //150;
   	    
   	    /**
   	     * The velocity constant for pigeon turning in place (w/ motion magic).
   	     */
   	    public static int TURN_VEL = 500;
   	    
   	    /**
   	     * The acceleration constant for pigeon turning in place (w/ motion magic).
   	     */
   	    public static int TURN_ACCEL = 600;
   	    
   	    /**
   	     * The frame period (in ms) for pigeon status updates.
   	     */
   	    public static final int PERIOD_MS = 4;
   	
   	    /**
   	     * The Talon slot to configure the Pigeon as a remote sensor.
   	     */
   	    public static final RemoteFeedbackDevice REMOTE_SENSOR_SLOT = RemoteFeedbackDevice.RemoteSensor0;
   	    
   	    /**
   	     * The allowable error for an angle closed loop.
   	     */
   	    public static final int ANGLE_ALLOWABLE_ERROR = 35;
   	    
   	    /**
   	     * The left sensor phase for a turn in place.
   	     */
   	    public static final boolean LEFT_SENSOR_PHASE = false;
   	    
   	    /**
   	     * The right sensor phase for a turn in place.
   	     */
   	    public static final boolean RIGHT_SENSOR_PHASE = false;
   	}

}
