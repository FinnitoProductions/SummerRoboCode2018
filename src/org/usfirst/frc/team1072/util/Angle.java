package org.usfirst.frc.team1072.util;

/**
 * Represents an angle with easy conversion.
 * @author Finn Frankis
 * @version 7/5/18
 */
public class Angle
{
    AngleUnit unit;
    double value;

    /**
     * Represents the various possible units for an angle.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public enum AngleUnit
    {
        RADIANS, DEGREES, PIGEON_UNITS;
    }
    
    /**
     * Constructs a new Angle object.
     * @param unit the unit of the angle (radians, degrees, pigeon units)
     * @param value the value for the angle
     */
    public Angle(AngleUnit unit, double value)
    {
        this.unit = unit;
        this.value = value;
    }
    
    /**
     * Gets the value of this angle in degrees.
     * @return the angle value in degrees
     */
    public double getDegrees()
    {
        if (unit == AngleUnit.DEGREES)
            return value;
        else if (unit == AngleUnit.RADIANS)
            return value * ConversionFactors.DEGREES_PER_ROTATION/ConversionFactors.RADIANS_PER_ROTATION;
        else 
            return value * ConversionFactors.DEGREES_PER_ROTATION/ConversionFactors.PIGEON_UNITS_PER_ROTATION;
    }
    
    /**
     * Gets the value of this angle in radians.
     * @return the angle value in radians
     */
    public double getRadians()
    {
        if (unit == AngleUnit.RADIANS)
            return value;
        else if (unit == AngleUnit.DEGREES)
            return value * ConversionFactors.RADIANS_PER_ROTATION/ConversionFactors.DEGREES_PER_ROTATION;
        else 
            return value * ConversionFactors.RADIANS_PER_ROTATION/ConversionFactors.PIGEON_UNITS_PER_ROTATION;
    }
    
    /**
     * Gets the value of this angle in pigeon units.
     * @return the angle value in pigeon units
     */
    public double getPigeonUnits()
    {
        if (unit == AngleUnit.PIGEON_UNITS)
            return value;
        else if (unit == AngleUnit.DEGREES)
            return value * ConversionFactors.PIGEON_UNITS_PER_ROTATION/ConversionFactors.DEGREES_PER_ROTATION;
        else 
            return value * ConversionFactors.PIGEON_UNITS_PER_ROTATION/ConversionFactors.RADIANS_PER_ROTATION;
    }
}

