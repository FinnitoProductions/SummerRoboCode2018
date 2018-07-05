package org.usfirst.frc.team1072.robot.subsystems;

import java.util.Iterator;
import java.util.Map;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.DriveWithVelocityCommand;
import org.usfirst.frc.team1072.robot.commands.TurnRobotToAngleCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory.Segment;

/**
 * Represents a drive train with two Talons and two Victors.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class Drivetrain extends PIDSubsystem
{
    private static Drivetrain instance = null;
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    private VictorSPX leftVictor;
    private VictorSPX rightVictor;
    private PigeonIMU pigeon;
    private boolean pidEnabled;
    private double currentPIDOutput;
    private Map<IMotorController, Object[]> controllers; // for PID
    private double startTime;
    
    private Segment[] leftPoints;
    private Segment[] rightPoints;
    
    private Drivetrain()
    {
        // initialize talons
        super("Drivetrain", 
                RobotMap.DT_POS_KP_LEFT / RobotMap.TALON_MOTOR_OUTPUT_UNITS, 
                RobotMap.DT_POS_KI_LEFT / RobotMap.TALON_MOTOR_OUTPUT_UNITS, 
                RobotMap.DT_POS_KD_LEFT /  RobotMap.TALON_MOTOR_OUTPUT_UNITS, 
                RobotMap.DT_POS_KF_LEFT /  RobotMap.TALON_MOTOR_OUTPUT_UNITS, 
                RobotMap.PID_OUTPUT_PERIOD_S);
        leftTalon = new TalonSRX (RobotMap.LEFT_CIM_TALON);
        rightTalon = new TalonSRX (RobotMap.RIGHT_CIM_TALON);
        leftVictor = new VictorSPX (RobotMap.LEFT_CIM_VICTOR);
        rightVictor = new VictorSPX (RobotMap.RIGHT_CIM_VICTOR);
        pigeon = new PigeonIMU(RobotMap.PIGEON_ID);
        pidEnabled = false;

    }
    
    public double getCurrentPIDOutput()
    {
        return currentPIDOutput;
    }
    /**
     * Initializes the command using the four ports for the Talons/Victors.
     */
    public void initDefaultCommand()
    {
        setDefaultCommand(new DriveWithVelocityCommand(OI.BLACK_XBOX_DEADBAND));
        //setDefaultCommand(new DriveToPositionCommand());
        //setDefaultCommand(new TurnRobotToAngleCommand());
    }
    
    /**
     * Runs both motor controllers given a velocity and turn.
     * @param speed the speed at which the controllers should run
     * @param turn the amount by which each speed should be modified to account for turn
     */
    public void arcadeDriveVelocity(double speed, double turn)
    {
        // victor follows talon
        getLeftTalon().selectProfileSlot(RobotMap.DT_VEL_PID, RobotMap.PRIMARY_PID_INDEX);
        getRightTalon().selectProfileSlot(RobotMap.DT_VEL_PID, RobotMap.PRIMARY_PID_INDEX);
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
        getLeftTalon().selectProfileSlot(RobotMap.DT_POS_PID, 0);
        getRightTalon().selectProfileSlot(RobotMap.DT_POS_PID, 0);
        leftTalon.set(ControlMode.Position, target);
        rightTalon.set(ControlMode.Position, target);
    }
    
    public void arcadeDrivePosition (double leftTarget, double rightTarget)
    {
        System.out.println("DRIVING TO POSITION");
        getLeftTalon().selectProfileSlot(RobotMap.DT_POS_PID, 0);
        getRightTalon().selectProfileSlot(RobotMap.DT_POS_PID, 0);
        leftTalon.set(ControlMode.Position, leftTarget);
        rightTalon.set(ControlMode.Position, rightTarget);
    }
    

    /**
     * Performs autonomous-specific commands on the Talon.
     */
    public void talonInitAutonomous()
    {
        talonInit();

        setupMotionProfile(true);
    }
    
    public void talonInitTeleop()
    {
        talonInit();
        disableMotionProfile();
    }
    /**
     * Performs all commands to initialize the talons.
     * 
     * @postcondition the talons are in the correct state to begin robot operation
     */
    public void talonInit()
    {
        victorInit();
        // initTalonOutput(0);

        invertControllers();
        setNeutralMode(NeutralMode.Brake);

        setRampTime(RobotMap.MAX_RAMP_TIME);

        scaleVoltage(RobotMap.NOMINAL_BATTERY_VOLTAGE);

        resetTalonCoefficients();
        setTalonSensorPhase();
        dtsetTalonFramePeriods();
        
        configureVelocityClosedLoop();
        configurePositionClosedLoop();
        configureAngleClosedLoop();
        configureMotionProfileClosedLoop();

        dtSetCurrentLimit(RobotMap.DT_PEAK_CURRENT_LIMIT, RobotMap.DT_PEAK_TIME_MS,
                RobotMap.DT_CONTINUOUS_CURRENT_LIMIT);

    }
    
    /**
     * Slaves the Victors to directly follow the behavior of their parent talons.
     */
    private void victorInit()
    {
        getLeftVictor().follow(getLeftTalon());
        getRightVictor().follow(getRightTalon());
        /*getLeftVictor().configRemoteFeedbackFilter(getLeftTalon().getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_0, RobotMap.TIMEOUT);
        getRightVictor().configRemoteFeedbackFilter(getRightTalon().getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_0, RobotMap.TIMEOUT);
        
        getLeftVictor().configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
        getRightVictor().configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
        
        getLeftVictor().selectProfileSlot(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID);
        getRightVictor().selectProfileSlot(RobotMap.DT_MOTION_PROFILE_PID, RobotMap.PRIMARY_PID);
        
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, RobotMap.QUADRATURE_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, RobotMap.QUADRATURE_PERIOD_MS, RobotMap.TIMEOUT);*/
    }
    
    public void victorTeleopInit()
    {
        /*getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, RobotMap.TIMEOUT);*/
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
    
    private void resetTalonCoefficients()
    {
        // for all PID slots
        for (int slot = 0; slot < RobotMap.NUM_PID_SLOTS; slot++)
        {
            getLeftTalon().configSelectedFeedbackCoefficient(1,
                    slot, RobotMap.TIMEOUT); 
            getRightTalon().configSelectedFeedbackCoefficient(1,
                    slot, RobotMap.TIMEOUT); 
        }
    }
   

    /**
     * Inverts the Talons and Victors to account for wiring inconsistencies (must be
     * tested).
     */
    private void invertControllers()
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
     * Configures the encoders either to use pulse width or quadrature velocity.
     * 
     * @param f
     *            the feedback device to use to configure (either quadrature or
     *            sensor velocity)
     */
    public void velocityConfigureSensors(FeedbackDevice f)
    {
        //getLeftTalon().configSelectedFeedbackSensor(f, RobotMap.DT_VEL_PID, RobotMap.TIMEOUT);
        //getRightTalon().configSelectedFeedbackSensor(f, RobotMap.DT_VEL_PID, RobotMap.TIMEOUT);
    }

    /**
     * Aligns the sensor phase of the encoders to match the motions of the motors.
     */
    private void setTalonSensorPhase()
    {
        getLeftTalon().setSensorPhase(RobotMap.DT_LEFT_TALON_PHASE);
        getRightTalon().setSensorPhase(RobotMap.DT_RIGHT_TALON_PHASE);
    }

    /**
     * Configures the drivetrain velocity closed loop.
     * 
     * @postcondition nominal and peak output in both directions has been set
     */
    private void configureVelocityClosedLoop()
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

        getLeftTalon().config_kF(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.DT_VEL_PID, RobotMap.DT_VEL_KD_RIGHT, RobotMap.TIMEOUT);
      
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
        positionConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        getLeftTalon().setSelectedSensorPosition(0, RobotMap.DT_POS_PID, RobotMap.TIMEOUT);
        getRightTalon().setSelectedSensorPosition(0, RobotMap.DT_POS_PID, RobotMap.TIMEOUT);

        getLeftTalon().configAllowableClosedloopError(RobotMap.DT_POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);
        getRightTalon().configAllowableClosedloopError(RobotMap.DT_POS_PID, RobotMap.DT_POS_ALLOWABLE_ERROR, RobotMap.TIMEOUT);

        getLeftTalon().config_kF(RobotMap.DT_POS_PID, RobotMap.DT_POS_KF_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kF(RobotMap.DT_POS_PID, RobotMap.DT_POS_KF_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kP(RobotMap.DT_POS_PID, RobotMap.DT_POS_KP_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kP(RobotMap.DT_POS_PID, RobotMap.DT_POS_KP_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kI(RobotMap.DT_POS_PID, RobotMap.DT_POS_KI_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.DT_POS_PID, RobotMap.DT_POS_KI_RIGHT, RobotMap.TIMEOUT);

        getLeftTalon().config_kD(RobotMap.DT_POS_PID, RobotMap.DT_POS_KD_LEFT, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.DT_POS_PID, RobotMap.DT_POS_KD_RIGHT, RobotMap.TIMEOUT);
    }
    
    private void positionConfigureSensors(FeedbackDevice f)
    {
        //getLeftTalon().configSelectedFeedbackSensor(f, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        //getRightTalon().configSelectedFeedbackSensor(f, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
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
    public void configureAngleClosedLoop()
    {
        pigeonInit();
        getLeftTalon().configRemoteFeedbackFilter(getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_0, 
                RobotMap.TIMEOUT);
        getRightTalon().configRemoteFeedbackFilter(getPigeon().getDeviceID(), 
                RemoteSensorSource.Pigeon_Yaw, 
                RobotMap.REMOTE_0, 
                RobotMap.TIMEOUT);
        
        getPigeon().setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, RobotMap.PIGEON_PERIOD_MS, RobotMap.TIMEOUT);
        
        getRightTalon().config_kP(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KP, RobotMap.TIMEOUT);
        getRightTalon().config_kI(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KI, RobotMap.TIMEOUT);
        getRightTalon().config_kD(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KD, RobotMap.TIMEOUT);
        
        getLeftTalon().config_kP(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KP, RobotMap.TIMEOUT);
        getLeftTalon().config_kI(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KI, RobotMap.TIMEOUT);
        getLeftTalon().config_kD(RobotMap.DT_ANGLE_PID, RobotMap.PIGEON_ANGLE_KD, RobotMap.TIMEOUT);

    }
    
    private void pigeonInit()
    {
        getPigeon().setYaw(0, RobotMap.TIMEOUT);
        
    }

    private void configureMotionProfileClosedLoop()
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
     * Resets the talon frame periods.
     */
    private void dtsetTalonFramePeriods()
    {
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 3, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 30, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getLeftTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 30, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        //getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, RobotMap.MAX_TALON_FRAME_PERIOD_MS, RobotMap.TIMEOUT);
        //getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, RobotMap.TIMEOUT);
        //getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TIMEOUT);
        //getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.TIMEOUT);
        getRightTalon().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, RobotMap.TIMEOUT);
        
    }
   
    public void setupMotionProfile(boolean isArc)
    {
        if (isArc)
        {            
            //getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
           // getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
            
            
            //getLeftTalon().configAuxPIDPolarity(arg0, RobotMap.TIMEOUT);

        }
        else
        {
            //getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
            //getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT);
        }
    }
    
    public void disableMotionProfile()
    {
        
    }
    
    @Deprecated
    public void configureEncoderAverage()
    {
        getLeftTalon().configRemoteFeedbackFilter(getRightTalon().getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_1, RobotMap.TIMEOUT);
        getRightTalon().configRemoteFeedbackFilter(getLeftTalon().getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_1, RobotMap.TIMEOUT);
        
        getLeftTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
        getRightTalon().configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT);
        getLeftTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
        getRightTalon().configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT);
        
        getLeftTalon().configSelectedFeedbackCoefficient(0.5,
                RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT); //using native sensor units, take average of sum
        getRightTalon().configSelectedFeedbackCoefficient(0.5,
                RobotMap.DT_MOTION_PROFILE_PID, RobotMap.TIMEOUT); //using native sensor units
    }
    
    public void clearFeedbackCoefficient(int pidSlot)
    {
        getLeftTalon().configSelectedFeedbackCoefficient(1,
                pidSlot, RobotMap.TIMEOUT); //using native sensor units, take average of sum
        getRightTalon().configSelectedFeedbackCoefficient(1,
                pidSlot, RobotMap.TIMEOUT); //using native sensor units
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
    @Override
    protected double returnPIDInput()
    {
       double currentTime =  Timer.getFPGATimestamp() * RobotMap.MS_PER_SEC;
       double timeElapsed = currentTime - startTime;
       int startPoint = (int) (timeElapsed/RobotMap.TIME_PER_TRAJECTORY_POINT_MS);
       int endPoint = startPoint + 1;
       if (endPoint >= leftPoints.length)
       {
           setSetpoint((leftPoints[leftPoints.length-1].position + rightPoints[leftPoints.length-1].position)/2);
       }
       else
       {
           // interpolate to find point for current time
           double interpStartTime = startPoint * RobotMap.TIME_PER_TRAJECTORY_POINT_MS;
           double interpEndTime = endPoint * RobotMap.TIME_PER_TRAJECTORY_POINT_MS;
           double deltaT = interpEndTime - interpStartTime; 
           
           double startPosLeft = leftPoints[startPoint].position;
           double endPosLeft = leftPoints[endPoint].position;
           double slopeLeft = (endPosLeft - startPosLeft) / (interpEndTime - interpStartTime);

           
           double startPosRight = rightPoints[startPoint].position;
           
           double endPosRight = rightPoints[endPoint].position;
           double slopeRight = (endPosRight - startPosRight) / (interpEndTime - interpStartTime);
           
           double targetPosLeft = slopeLeft * (timeElapsed - interpStartTime) + startPosLeft;
           double targetPosRight = slopeRight * (timeElapsed - interpStartTime) + startPosRight;
           System.out.println(" " + targetPosLeft + " " + targetPosRight); 
           SmartDashboard.putNumber("TARGET POS ", (targetPosLeft + targetPosRight)/2);
           SmartDashboard.putNumber("ERROR", (targetPosLeft+targetPosRight) / 2 
                   - (getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) +getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX))/2 );
           setSetpoint((targetPosLeft + targetPosRight)/2);
       }

        return (leftTalon.getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) + 
                rightTalon.getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX))/2;
    }
    @Override
    protected void usePIDOutput(double output)
    {
        if (pidEnabled)
        {
            currentPIDOutput = output;
            //System.out.println("PID OUTPUT : " + output + " TIME SINCE START: " + (Timer.getFPGATimestamp() * 1000 - startTime));
            SmartDashboard.putNumber("TALON OUTPUT", output + Math.random() * 0.0000000000001);
            SmartDashboard.putNumber("POSITION ERROR", this.getPIDController().getError());
            SmartDashboard.putNumber("ENCODER POSITION", getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX));
            //SmartDashboard.putNumber("RIGHT POSITION ERROR", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID));
            for (IMotorController imc : controllers.keySet())
            {
                imc.set(ControlMode.PercentOutput, output);
                
                //this.getPIDController().setOutputRange(0, 1);
            }
            
        }
        else
            currentPIDOutput = -1;
        
    }
    
    public void enablePID()
    {
        leftTalon.selectProfileSlot(RobotMap.DT_POS_PID, RobotMap.PRIMARY_PID_INDEX);
        rightTalon.selectProfileSlot(RobotMap.DT_POS_PID, RobotMap.PRIMARY_PID_INDEX);
        pidEnabled = true;
        
        controllers = Robot.m_autonomousCommand.getCurrentPath().getControllers();
        startTime = Robot.m_autonomousCommand.getCurrentPath().getStartTime();
        
        Iterator<IMotorController> it = controllers.keySet().iterator();
        leftPoints = Robot.m_autonomousCommand.getCurrentPath().getControllerTrajectory(it.next()).segments;
        rightPoints = Robot.m_autonomousCommand.getCurrentPath().getControllerTrajectory(it.next()).segments;

        this.enable();
    }
    
    public void disablePID()
    {
        pidEnabled = false;
        this.disable();
    }

    public void zeroPigeon()
    {
        getPigeon().setYaw(0, RobotMap.TIMEOUT);
        getPigeon().setAccumZAngle(0, RobotMap.TIMEOUT);
    }

}
