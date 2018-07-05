/**
 * 
 */
package org.usfirst.frc.team1072.util;

import org.usfirst.frc.team1072.util.Position.PositionUnit;

/**
 * 
 * @author Finn Frankis
 * @version 7/5/18
 */
public class Time
{
    private TimeUnit unit;
    private double value;
    private double wheelDiameter;

    /**
     * Represents the various possible units for a time.
     * @author Finn Frankis
     * @version 7/5/18
     */
    public enum TimeUnit
    {
        SECONDS, MILLISECONDS, MICROSECONDS, NANOSECONDS;
    }
    
    /**
     * Constructs a new time.
     * @param unit the unit of measure
     * @param value the value of the time
     */
    public Time(TimeUnit unit, double value)
    {
        this.unit = unit;
        this.value = value;
    }
    
    /**
     * Converts this time to seconds.
     * @return this time, converted into seconds
     */
    public double getSeconds()
    {
        if (unit == TimeUnit.SECONDS)
            return value;
        else if (unit == TimeUnit.MILLISECONDS)
            return value / ConversionFactors.MS_PER_SEC;
        else if (unit == TimeUnit.MICROSECONDS)
            return value / ConversionFactors.MICROSECS_PER_SEC;
        else
            return value / ConversionFactors.NANOSECS_PER_SEC;
    }
    
    /**
     * Converts this time to milliseconds.
     * @return this time, converted into milliseconds.
     */
    public double getMilliseconds()
    {
        if (unit == TimeUnit.MILLISECONDS)
            return value;
        else if (unit == TimeUnit.SECONDS)
            return value * ConversionFactors.MS_PER_SEC;
        else if (unit == TimeUnit.MICROSECONDS)
            return value / ConversionFactors.MICROSECS_PER_SEC * ConversionFactors.MS_PER_SEC;
        else
            return value / ConversionFactors.NANOSECS_PER_SEC * ConversionFactors.MS_PER_SEC;
    }
    
    /**
     * Converts this time to microseconds.
     * @return this time, converted into microseconds.
     */
    public double getMicroseconds()
    {
        if (unit == TimeUnit.MICROSECONDS)
            return value;
        else if (unit == TimeUnit.SECONDS)
            return value * ConversionFactors.MICROSECS_PER_SEC;
        else  if (unit == TimeUnit.MILLISECONDS)
            return value / ConversionFactors.MS_PER_SEC * ConversionFactors.MICROSECS_PER_SEC;
        else
            return value / ConversionFactors.NANOSECS_PER_SEC * ConversionFactors.MICROSECS_PER_SEC;
    }
    
    /**
     * Converts this time to nanoseconds.
     * @return this time, converted into nanoseconds.
     */
    public double getNanoseconds()
    {
        if (unit == TimeUnit.NANOSECONDS)
            return value;
        else if (unit == TimeUnit.SECONDS)
            return value * ConversionFactors.NANOSECS_PER_SEC;
        else  if (unit == TimeUnit.MILLISECONDS)
            return value / ConversionFactors.MS_PER_SEC * ConversionFactors.NANOSECS_PER_SEC;
        else
            return value / ConversionFactors.MICROSECS_PER_SEC * ConversionFactors.NANOSECS_PER_SEC;
    }
}
