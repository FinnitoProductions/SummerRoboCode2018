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
    
    public Angle(AngleUnit unit, double value)
    {
        this.unit = unit;
        this.value = value;
    }
    
    public double getDegrees()
    {
        if (unit == AngleUnit.DEGREES)
            return value;
        else if (unit == AngleUnit.RADIANS)
            return value * ConversionFactors.DEGREES_PER_ROTATION/ConversionFactors.RADIANS_PER_ROTATION;
        else 
            return value * ConversionFactors.DEGREES_PER_ROTATION/ConversionFactors.PIGEON_UNITS_PER_ROTATION;
    }
    
    public double getRadians()
    {
        if (unit == AngleUnit.RADIANS)
            return value;
        else if (unit == AngleUnit.DEGREES)
            return value * ConversionFactors.RADIANS_PER_ROTATION/ConversionFactors.DEGREES_PER_ROTATION;
        else 
            return value * ConversionFactors.RADIANS_PER_ROTATION/ConversionFactors.PIGEON_UNITS_PER_ROTATION;
    }
    
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

