package org.usfirst.frc.team1072.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1072.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Pneumatics extends Subsystem
{
    private Compressor c = new Compressor(RobotMap.COMPRESSOR_PORT);
    private static Pneumatics pn = new Pneumatics();
    private Map<String, DoubleSolenoid> solMap = new HashMap<String, DoubleSolenoid>();
    private DoubleSolenoid intake_updown = new DoubleSolenoid(/*RobotMap.FIRST_PCM_ID, */
            RobotMap.INTAKE_UP_SOL, RobotMap.INTAKE_DOWN_SOL);
    private DoubleSolenoid intake_compressdecompress = new DoubleSolenoid (/*RobotMap.FIRST_PCM_ID,*/
            RobotMap.INTAKE_COMPRESS_SOL, RobotMap.INTAKE_DECOMPRESS_SOL);
    
    @Override
    protected void initDefaultCommand()
    {
        solMap.put(RobotMap.INTAKE_UPDOWN_KEY, intake_updown);
        solMap.put(RobotMap.INTAKE_COMPRESSDECOMPRESS_KEY, intake_compressdecompress);
    }
    
    public DoubleSolenoid getSolenoid(String key)
    {
        return solMap.get(key);
    }
    
    public void setCompressor(boolean state)
    {
        c.setClosedLoopControl(state);
    }
    
    public Compressor getCompressor()
    {
        return c;
    }

    public static Pneumatics getInstance()
    {
        if (pn == null)
            pn = new Pneumatics();
        return pn;
    }
}
