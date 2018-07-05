package org.usfirst.frc.team1072.util;

/**
 * Represents a position with easy conversion to other units.
 * @author Finn Frankis
 * @version 7/5/18
 */
public class Position
{
    private PositionUnit unit;
    private double value;
    private double wheelDiameter;

    /**
     * Represents the various possible units for a position.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public enum PositionUnit
    {
        FEET, ENCODER_UNITS;
    }
    
    /**
     * Constructs a new position.
     * @param unit the unit of measure, where encoder units is in ticks
     * @param value the value of the position
     * @param wheelDiameter the diameter of the wheels used on the robot
     */
    public Position(PositionUnit unit, double value, double wheelDiameter)
    {
        this.unit = unit;
        this.value = value;
        this.wheelDiameter = wheelDiameter;
    }
    
    /**
     * Converts this position to feet.
     * @return this position, converted into feet
     */
    public double getFeet()
    {
        if (unit == PositionUnit.FEET)
            return value;
        else
            return value / ConversionFactors.TICKS_PER_REV // convert to revolutions
                    * (wheelDiameter * Math.PI) // convert to inches
                    / ConversionFactors.INCHES_PER_FOOT; // convert to fee
    }
    
    /**
     * Converts this speed to encoder units (ticks).
     * @return this speed, converted into ticks.
     */
    public double getEncoderUnits()
    {
        if (unit == PositionUnit.ENCODER_UNITS)
            return value;
        else
            return value * ConversionFactors.INCHES_PER_FOOT // convert to inches
                    / (wheelDiameter * Math.PI) // convert to revolutions
                    * ConversionFactors.TICKS_PER_REV;// convert to ticks 
    }
}
