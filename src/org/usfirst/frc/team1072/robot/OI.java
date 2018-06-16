package org.usfirst.frc.team1072.robot;

//import org.harker.robotics.harkerrobolib.*;
//import org.harker.robotics.harkerrobolib.wrappers.GamepadWrapper;

/**
 * Keeps track of the IO of the robot, such as button mappings.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class OI
{
    
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
    
    public static int BLACK_XBOX_LEFTX = 0;
    public static int BLACK_XBOX_LEFTY = 1;
    public static int BLACK_XBOX_RIGHTX = 4;
    public static int BLACK_XBOX_RIGHTY = 5;
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
    public static int LEFT_BUMPER = 5;
    public static int RIGHT_BUMPER = 6;
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
