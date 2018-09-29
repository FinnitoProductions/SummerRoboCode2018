package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.harkerrobolib.wrappers.DPadButtonWrapper;
import org.usfirst.frc.team1072.harkerrobolib.wrappers.GamepadWrapper;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;

import edu.wpi.first.wpilibj.command.CommandGroup;



/**
 * Keeps track of the IO of the robot, such as button mappings.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class OI
{
    /**
     * The port for the driver controller.
     */
    public static final int DRIVER_PORT = 0;

    /**
     * The port for the operator controller.
     */
    public static final int OPERATOR_PORT = 1;
    
    /**
     * The deadband (range for which no input is received) of the black XBOX controller.
     */
    public static final double BLACK_XBOX_DRIVE_DEADBAND = 0.09;

    public static final double BLACK_XBOX_ELEVATOR_DEADBAND = 0.15;
    
    /**
     * The deadband (range for which no input is received) of the Logitech controller.
     */
    public static final double LOGITECH_DEADBAND = 0.15;
    
    /**
     * The current instance of the singleton OI.
     */
    private static OI oi = null;
  
    /**
     * The gamepad for use by the driver.
     */
    private GamepadWrapper driverGamepad = new GamepadWrapper(DRIVER_PORT);
    
    /**
     * The gamepad for use by the operator.
     */
    private GamepadWrapper operatorGamepad = new GamepadWrapper(OPERATOR_PORT, GamepadWrapper.SETTING_LOGITECH);
    
    /**
     * Constructs a new OI.
     */
    public OI ()
    {
        DPadButtonWrapper upDPadDriver = new DPadButtonWrapper(driverGamepad, 0);
        DPadButtonWrapper leftDPadDriver = new DPadButtonWrapper (driverGamepad, 270);
        DPadButtonWrapper downDPadDriver= new DPadButtonWrapper (driverGamepad, 180);
        DPadButtonWrapper rightDPadDriver = new DPadButtonWrapper (driverGamepad, 90);
        
        upDPadDriver.whenPressed(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
        downDPadDriver.whenPressed( new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
        leftDPadDriver.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
        rightDPadDriver.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        
        if (RobotMap.TWO_CONTROLLERS)
        {
            DPadButtonWrapper upDPadOperator = new DPadButtonWrapper(operatorGamepad, 0);
            DPadButtonWrapper leftDPadOperator = new DPadButtonWrapper (operatorGamepad, 270);
            DPadButtonWrapper downDPadOperator = new DPadButtonWrapper (operatorGamepad, 180);
            DPadButtonWrapper rightDPadOperator = new DPadButtonWrapper (operatorGamepad, 90);

            upDPadOperator.whenPressed(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
            downDPadOperator.whenPressed( new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
            leftDPadOperator.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
            rightDPadOperator.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        }
        
        CommandGroup lowerAndOpen = new CommandGroup();
            lowerAndOpen.addSequential(new MoveElevatorMotionMagic(ElevatorConstants.INTAKE_HEIGHT));
            lowerAndOpen.addSequential(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
            lowerAndOpen.addParallel(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
        driverGamepad.getButtonA().whenPressed(lowerAndOpen);
        
        driverGamepad.getButtonX().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SWITCH_HEIGHT));
        driverGamepad.getButtonB().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SCALE_LOW_HEIGHT));
        driverGamepad.getButtonY().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SCALE_HIGH_HEIGHT));     
    }

    /**
     * Gets the current instance of the driver gamepad.
     * @return the driver gamepad instance
     */
    public GamepadWrapper getDriverGamepad() 
    {
        return driverGamepad;
    }
    
    /**
     * Gets the current instance of the operator gamepad.
     * @return the operator gamepad instance
     */
    public GamepadWrapper getOperatorGamepad() 
    {
        return operatorGamepad;
    }
    
    /**
     * Gets the current instance of the singleton OI.
     * @return the current instance
     */
    public static OI getInstance()
    {
        if (oi == null)
            oi = new OI();
        return oi;
    }
}
