package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.harkerrobolib.wrappers.DPadButtonWrapper;
import org.usfirst.frc.team1072.harkerrobolib.wrappers.GamepadWrapper;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;



/**
 * Keeps track of the IO of the robot, such as button mappings.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class OI
{
    /**
     * The port for the black XBOX controller.
     */
    public static final int BLACK_XBOX_PORT = 0;

    /**
     * The deadband (range for which no input is received) of the black XBOX controller.
     */
    public static final double BLACK_XBOX_DEADBAND = 0.15;
    
    /**
     * The current instance of the singleton OI.
     */
    private static OI oi = null;
  
    /**
     * The gamepad currently in use.
     */
    private GamepadWrapper gw = new GamepadWrapper(BLACK_XBOX_PORT);
    
    /**
     * Constructs a new OI.
     */
    public OI ()
    {
        DPadButtonWrapper upDPad = new DPadButtonWrapper(gw, 0);
        DPadButtonWrapper leftDPad = new DPadButtonWrapper (gw, 270);
        DPadButtonWrapper downDPad = new DPadButtonWrapper (gw, 180);
        DPadButtonWrapper rightDPad = new DPadButtonWrapper (gw, 90);
        
        upDPad.whenPressed(new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
        downDPad.whenPressed( new SetSolenoid(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
        leftDPad.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
        rightDPad.whenPressed(new SetSolenoid(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        
        gw.getButtonA().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.INTAKE_HEIGHT));
        gw.getButtonX().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SWITCH_HEIGHT));
        gw.getButtonB().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SCALE_LOW_HEIGHT));
        gw.getButtonY().whenPressed(new MoveElevatorMotionMagic(ElevatorConstants.SCALE_HIGH_HEIGHT));     
    }

    /**
     * Gets the current instance of the gamepad.
     * @return the gamepad instance
     */
    public GamepadWrapper getGamepad() 
    {
        return gw;
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
