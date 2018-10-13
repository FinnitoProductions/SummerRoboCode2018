package org.usfirst.frc.team1072.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1072.robot.commands.intake.SetCompressor;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents a pneumatics subsystem with full control over the pneumatic systems.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Pneumatics extends Subsystem
{
    /**
     * The compressor for use in the pneumatic subsystem.
     */
    private Compressor c;
    
    /**
     * The current instance of this singleton subsystem.
     */
    private static Pneumatics pn;
    
    /**
     * The solenoid to raise and lower the intake.
     */
    private static DoubleSolenoid intake_updown;
    
    /**
     * The solenoid to compress and decompress the intake.
     */
    private static DoubleSolenoid intake_compressdecompress;

	/**
	 * The CAN ID of the compressor.
	 */
	public static final int COMPRESSOR_PORT = 0;
    
    
    public enum SolenoidDirection {
    	UP(intake_updown, Intake.UP), 
    	DOWN(intake_updown, Intake.DOWN), 
    	COMPRESS(intake_compressdecompress, Intake.COMPRESS), 
    	DECOMPRESS(intake_compressdecompress, Intake.DECOMPRESS);
    	
    	
    	private final DoubleSolenoid solenoid;
    	private final DoubleSolenoid.Value state;
    	
    	SolenoidDirection(DoubleSolenoid solenoid, DoubleSolenoid.Value state) {
    		this.solenoid = solenoid;
    		this.state = state;
    	}
    	
    	public DoubleSolenoid getSolenoid() {return solenoid;}
    	public DoubleSolenoid.Value getState() {return state;}
    }
    
    public enum SolenoidType {
    	UPDOWN, COMPRESSDECOMPRESS;
    }
    /**
     * Constructs a new Pneumatics.
     */
    public Pneumatics ()
    {
        c = new Compressor(Pneumatics.COMPRESSOR_PORT);
        intake_updown = new DoubleSolenoid(/*RobotMap.FIRST_PCM_ID, */
                Intake.INTAKE_UP_SOL, Intake.INTAKE_DOWN_SOL);
        intake_compressdecompress = new DoubleSolenoid (/*RobotMap.FIRST_PCM_ID,*/
                Intake.INTAKE_COMPRESS_SOL, Intake.INTAKE_DECOMPRESS_SOL); 
    }
    /**
     * Initializes the command, setting up all the objects and the map of solenoids.
     */
    protected void initDefaultCommand()
    {
        setDefaultCommand(new SetCompressor(true));
    }
    
    /**
     * Gets a solenoid given the key in the map.
     * @param key the key in the map
     * @return the solenoid stored by the given key
     */
    public DoubleSolenoid getSolenoid(SolenoidType type)
    {
    	return (type == SolenoidType.UPDOWN) ? intake_updown : intake_compressdecompress; 
    }

    public void setSolenoid(SolenoidDirection direction)
    {
    	direction.getSolenoid().set(direction.getState());
    }
    
    public void toggleSolenoid (SolenoidType type) {
    	DoubleSolenoid solenoid = getSolenoid(type);
    	solenoid.set(solenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }
    
    /**
     * Sets the compressor given a state.
     * @param state the state to which the compressor will be set
     */
    public void setCompressor(boolean state)
    {
        c.setClosedLoopControl(state);
    }
    
    /**
     * Gets the compressor.
     * @return the compressor
     */
    public Compressor getCompressor()
    {
        return c;
    }

    /**
     * Gets the instance of this Pneumatics, creating a new one if necessary,
     * @return the instance of Pneumatics
     */
    public static Pneumatics getInstance()
    {
        if (pn == null)
            pn = new Pneumatics();
        return pn;
    }
}
