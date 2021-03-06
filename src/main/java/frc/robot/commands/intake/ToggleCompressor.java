package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a command to toggle the compressor.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class ToggleCompressor extends Command
{
    /**
     * The desired state for the compressor.
     */
    private boolean state;
    /**
     * Sets up the command, requiring the pneumatics subsystem.
     */
    public ToggleCompressor() 
    { 
        requires(Intake.pn); 
    }

    /**
     * Initializes the command.
     */
    public void initialize()
    {
        state = !Intake.pn.getCompressor().getClosedLoopControl();
        Intake.pn.setCompressor(state); 
    }
    /**
     * Determines whether the commmand has finished.
     */
    protected boolean isFinished() { return Intake.pn.getCompressor().getClosedLoopControl() == state; }
}
