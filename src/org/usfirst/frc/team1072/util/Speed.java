package org.usfirst.frc.team1072.util;

/**
 * Represents a speed value with easy conversion to other values.
 * @author Finn Frankis
 * @version 7/5/18
 */
public class Speed
{
    private SpeedUnit unit;
    private double value;
    private double wheelDiameter;

    /**
     * Represents the various possible units for a speed.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public enum SpeedUnit
    {
        FEET_PER_SECOND, ENCODER_UNITS;
    }
    
    /**
     * Constructs a new speed.
     * @param unit the unit of measure, where encoder units is in ticks per 100 ms
     * @param value the value of the speed
     * @param wheelDiameter the diameter of the wheels used on the robot
     */
    public Speed(SpeedUnit unit, double value, double wheelDiameter)
    {
        this.unit = unit;
        this.value = value;
        this.wheelDiameter = wheelDiameter;
    }
    
    /**
     * Converts this speed to feet per second.
     * @return this speed, converted into feet per second
     */
    public double getFeetPerSecond()
    {
        if (unit == SpeedUnit.FEET_PER_SECOND)
            return value;
        else
            return value * 10.0 // convert to ticks per second
                    / ConversionFactors.TICKS_PER_REV // convert to revolutions per second
                    * (wheelDiameter * Math.PI) // convert to inches per second
                    / ConversionFactors.INCHES_PER_FOOT; // convert to feet per second
    }
    
    /**
     * Converts this speed to encoder units (ticks per 100 ms).
     * @return this speed, converted into ticks per 100 ms.
     */
    public double getEncoderUnits()
    {
        if (unit == SpeedUnit.ENCODER_UNITS)
            return value;
        else
            return value / 10.0 // convert to feet per 100 ms
                    * ConversionFactors.INCHES_PER_FOOT // convert to inches per 100 ms
                    / (wheelDiameter* Math.PI) // convert to revolutions per 100ms
                    * ConversionFactors.TICKS_PER_REV // convert to ticks per 100ms
                    
                    ; 
    }
}
