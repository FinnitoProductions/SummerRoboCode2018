package org.usfirst.frc.team1072.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1072.robot.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents a wrapper class for gamepads.
 * @author Finn Frankis
 * @version 6/16/18
 */
public class Gamepad extends Subsystem
{
    private double deadband;
    private int invertY;
    // see OI.java for necessary keys
    private Map<String, Integer> ports; 
    private int port;
    private Joystick jt;
    private double triggerDeadband;
    private boolean isAnalogTrigger;

    /**
     * Creates a new gamepad.
     * @param deadband the range for which the gamepad range will be considered zero
     * @param invertY whether or not to invert the Y stick
     * @param ports a map containing the ports of buttons and axes
     * @param port the joystick port 
     * @param isAnalogTrigger whether or not the trigger is an analog axis or simply a button
     */
    public Gamepad (double deadband, boolean invertY, Map<String, Integer> ports, int port, boolean isAnalogTrigger)
    {
        this.deadband = deadband;
        if (invertY)
            this.invertY = -1;
        else
            this.invertY = 1;
        this.port = port;
        this.ports = ports;
        jt = new Joystick(port);     
        triggerDeadband = 0;
        this.isAnalogTrigger = isAnalogTrigger;
    }
    
    public double getLeftX() throws IOException
    {
        if (!ports.containsKey(OI.LEFT_X))
            throw new IOException("Left X nonexistent");
        double x = jt.getRawAxis(ports.get(OI.LEFT_X));
        if (Math.abs(x) <= deadband)
            return 0;
        return x;
    }
    
    public double getLeftY() throws IOException
    {
        if (!ports.containsKey(OI.LEFT_Y))
            throw new IOException("Left Y nonexistent");
        double y = invertY * jt.getRawAxis(ports.get(OI.LEFT_Y));
        if (Math.abs(y) <= deadband)
            return 0;
        return y;
    }

    
    public double getRightX() throws IOException
    {
        if (!ports.containsKey(OI.RIGHT_X))
            throw new IOException("Right X nonexistent");
        double x = jt.getRawAxis(ports.get(OI.RIGHT_X));
        if (Math.abs(x) <= deadband)
            return 0;
        return x;
    }
    
    public double getRightY() throws IOException
    {
        if (!ports.containsKey(OI.RIGHT_Y))
            throw new IOException("Right Y nonexistent");
        double y = invertY * jt.getRawAxis(ports.get(OI.RIGHT_Y));
        if (Math.abs(y) <= deadband)
            return 0;
        return y;
    }
    
    /**
     * Returns the left trigger value, only if the trigger is analog.
     * @return if analog, the variable axis value where 0 is untouched and 1 is all the way; if not analog, 0 is not pressed and 1 is pressed
     * @throws IOException if the port was not provided
     */
    public double getLeftTrigger() throws IOException
    {
        if (!ports.containsKey(OI.LEFT_TRIGGER))
            throw new IOException("Left Trigger Nonexistent");
        if (!isAnalogTrigger)
            throw new IOException("Trigger not analog. Call getLeftTriggerButton().");

        double y = jt.getRawAxis(ports.get(OI.LEFT_TRIGGER));
        if (Math.abs(y) <= deadband)
            return 0;
        return y;
    }
    
    /**
     * Returns the left trigger output in a true-false button style (works for analog and button type triggers)
     * @return true if the trigger button is pushed down; false otherwise
     * @throws IOException if the port was not provided
     */
    public boolean getLeftTriggerButton() throws IOException
    {
        if (!ports.containsKey(OI.LEFT_TRIGGER))
            throw new IOException("Left Trigger Nonexistent");
        if (isAnalogTrigger)
        {
            if (jt.getRawAxis(ports.get(OI.LEFT_TRIGGER)) > triggerDeadband)
                return true;
            return false;
        }
        else
        {
            return jt.getRawButton(ports.get(OI.LEFT_TRIGGER));
        }
    }
    
    /**
     * Returns the right trigger value.
     * @return if analog, the variable axis value where 0 is untouched and 1 is all the way; if not analog, 0 is not pressed and 1 is pressed
     * @throws IOException if the port was not provided
     */
    public double getRightTrigger() throws IOException
    {
        if (!ports.containsKey(OI.RIGHT_TRIGGER))
            throw new IOException("Right Trigger Nonexistent");
        if (!isAnalogTrigger)
            throw new IOException("Trigger not analog. Call getRightTriggerButton().");
        double y = jt.getRawAxis(ports.get(OI.RIGHT_TRIGGER));
        if (Math.abs(y) <= triggerDeadband)
            return 0;
        return y;
    }
    
    /**
     * Returns the right trigger value in a true-false button style (works for analog and button type triggers)
     * @return true if the trigger button is pushed down; false otherwise
     * @throws IOException if the port was not provided
     */
    public boolean getRightTriggerButton() throws IOException
    {
        if (!ports.containsKey(OI.RIGHT_TRIGGER))
            throw new IOException("Right Trigger Nonexistent");
        if (isAnalogTrigger)
        {
            if (jt.getRawAxis(ports.get(OI.RIGHT_TRIGGER)) > triggerDeadband)
                return true;
            return false;
        }
        else
        {
            return jt.getRawButton(ports.get(OI.RIGHT_TRIGGER));
        }
    }
    
    /**
     * Returns the left bumper input.
     * @return true if the bumper is pressed; false otherwise
     * @throws IOException
     */
    public boolean getLeftBumper() throws IOException
    {
        if (!ports.containsKey(OI.LEFT_BUMPER))
            throw new IOException("Left Bumper Nonexistent");
        return jt.getRawButton(ports.get(OI.LEFT_BUMPER));
    }
    
    /**
     * Returns the right bumper input.
     * @return true if the bumper is pressed; false otherwise
     * @throws IOException
     */
    public boolean getRightBumper() throws IOException
    {
        if (!ports.containsKey(OI.RIGHT_BUMPER))
            throw new IOException("Right Bumper Nonexistent");
        return jt.getRawButton(ports.get(OI.RIGHT_BUMPER));
    }
    
    /**
     * Gets the D-Pad value on the controller.
     * @return the D-pad value as an integer in degrees, where the right button corresponds to 90 degrees and the top to 0 degrees.
     */
    public int getDPad()
    {
        return Math.round(jt.getPOV());
    }
   
    
    
    @Override
    protected void initDefaultCommand()
    {
        // TODO Auto-generated method stub
        
    }
}
