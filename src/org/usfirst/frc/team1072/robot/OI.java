package org.usfirst.frc.team1072.robot;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1072.harkerrobolib.wrappers.DPadButtonWrapper;
import org.usfirst.frc.team1072.harkerrobolib.wrappers.GamepadWrapper;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorMotionMagicCommand;
import org.usfirst.frc.team1072.robot.commands.SetSolenoidCommand;
import org.usfirst.frc.team1072.robot.commands.TurnRobotToAngleCommand;
import org.usfirst.frc.team1072.robot.subsystems.Gamepad;
import org.usfirst.frc.team1072.util.Position;
import org.usfirst.frc.team1072.util.Position.PositionUnit;

import edu.wpi.first.wpilibj.command.InstantCommand;



/**
 * Keeps track of the IO of the robot, such as button mappings.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class OI
{
    private static OI oi = null;
    public static int AXIS_MULTIPLIER = -1;
    public static int XBOX_360_PORT = 0;
    public static int AXES_MULTIPLIER = -1;
    
    /*
     * XBox Controller
     * Axis 0 - Left X
     * Axis 1 - Left Y
     * Axis 2 - Trigger (LT)
     * Axis 3 - Trigger (RT)
     * Axis 4 - Right X
     * Axis 5 - Right Y
     */
    
    public static int BLACK_XBOX_LEFT_X = 0;
    public static int BLACK_XBOX_LEFT_Y = 1;
    public static int BLACK_XBOX_RIGHT_X = 4;
    public static int BLACK_XBOX_RIGHT_Y = 5;
    public static int BLACK_XBOX_LEFT_TRIGGER = 2;
    public static int BLACK_XBOX_RIGHT_TRIGGER = 3;
    public static int BLACK_XBOX_LEFT_BUMPER = 5;
    public static int BLACK_XBOX_RIGHT_BUMPER = 6;
    public static double BLACK_XBOX_DEADBAND = 0.15;
    public static boolean BLACK_XBOX_INVERT_Y = true;
    public static int BLACK_XBOX_PORT = 0;
    public static boolean BLACK_XBOX_IS_ANALOG_TRIGGER = true;
    /*
     * White USB Controller
     * Axis 0 - Left X
     * Axis 1 - Left Y 
     * Axis 2/3 - Right X
     * Axis 4 - Right Y
     * 
     * Button 5 - Left Bumper
     * Button 6 - Right Bumper
     */
    
    public static int CHEAP_WHITE_LEFTX = 0;
    public static int CHEAP_WHITE_LEFTY = 1;
    public static int CHEAP_WHITE_RIGHTX = 2;
    public static int CHEAP_WHITE_RIGHTY = 4;
  

    
    public static String LEFT_X = "leftX";
    public static String LEFT_Y = "leftY";
    public static String RIGHT_X = "rightX";
    public static String RIGHT_Y = "rightY";
    public static String LEFT_TRIGGER = "leftTrigger";
    public static String RIGHT_TRIGGER = "rightTrigger";
    public static String LEFT_BUMPER = "leftBumper";
    public static String RIGHT_BUMPER = "rightBumper";
    
    //private Gamepad gw;
    private GamepadWrapper gw = new GamepadWrapper(BLACK_XBOX_PORT);
    public OI ()
    {
        /*Map<String, Integer> ports = new HashMap<String, Integer>();
        ports.put(LEFT_X, BLACK_XBOX_LEFT_X);
        ports.put(LEFT_Y, BLACK_XBOX_LEFT_Y);
        ports.put(RIGHT_X, BLACK_XBOX_RIGHT_X);
        ports.put(RIGHT_Y, BLACK_XBOX_RIGHT_Y);
        ports.put(LEFT_BUMPER, BLACK_XBOX_LEFT_BUMPER);
        ports.put(RIGHT_BUMPER, BLACK_XBOX_RIGHT_BUMPER);
        ports.put(LEFT_TRIGGER, BLACK_XBOX_LEFT_TRIGGER);
        ports.put(RIGHT_TRIGGER, BLACK_XBOX_RIGHT_TRIGGER);
        
        gw = new Gamepad(BLACK_XBOX_DEADBAND, BLACK_XBOX_INVERT_Y, ports, BLACK_XBOX_PORT, BLACK_XBOX_IS_ANALOG_TRIGGER);*/
        
        DPadButtonWrapper upDPad = new DPadButtonWrapper(gw, 0);
        DPadButtonWrapper leftDPad = new DPadButtonWrapper (gw, 270);
        DPadButtonWrapper downDPad = new DPadButtonWrapper (gw, 180);
        DPadButtonWrapper rightDPad = new DPadButtonWrapper (gw, 90);
        
        upDPad.whenPressed(new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY, IntakeConstants.UP));
        downDPad.whenPressed( new SetSolenoidCommand(IntakeConstants.UPDOWN_KEY, IntakeConstants.DOWN));
        leftDPad.whenPressed(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.DECOMPRESS));
        rightDPad.whenPressed(new SetSolenoidCommand(IntakeConstants.COMPRESSDECOMPRESS_KEY, IntakeConstants.COMPRESS));
        
        gw.getButtonA().whenPressed(
                new DriveToPositionCommand
                (new Position(PositionUnit.FEET, 3.85, DrivetrainConstants.WHEELDIAMETER).getEncoderUnits()));//new MoveElevatorMotionMagicCommand(ElevatorConstants.INTAKE_HEIGHT));
        gw.getButtonX().whenPressed(new MoveElevatorMotionMagicCommand(ElevatorConstants.SWITCH_HEIGHT));
        gw.getButtonB().whenPressed(new MoveElevatorMotionMagicCommand(ElevatorConstants.SCALE_LOW_HEIGHT));
        gw.getButtonY().whenPressed(new MoveElevatorMotionMagicCommand(ElevatorConstants.SCALE_HIGH_HEIGHT));
        
        /*gw.getButtonA().whenPressed(new TurnRobotToAngleCommand(90));
        gw.getButtonX().whenPressed(new TurnRobotToAngleCommand(0));
        gw.getButtonB().whenPressed(new TurnRobotToAngleCommand(-90));
        gw.getButtonY().whenPressed(new TurnRobotToAngleCommand(-180));*/
        
    }

    
    public GamepadWrapper getGamepad() 
    {
        return gw;
    }
    
    public static OI getInstance()
    {
        if (oi == null)
            oi = new OI();
        return oi;
    }
    
    
    
    
    
    /*GamepadWrapper gw = new GamepadWrapper (XBOX_360_PORT);
    
    /**
     * Gets the gamepad.
     * @return the gamepad
     */
   /* public GamepadWrapper getGamePad()
    {
        return gw;
    }*/
}
