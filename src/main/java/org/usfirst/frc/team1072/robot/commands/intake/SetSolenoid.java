package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team1072.robot.RobotMap;

/**
 * Represents a command to set the solenoid to a given state.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class SetSolenoid extends InstantCommand
{
    /**
     * The key for the solenoid to be actuated in the solenoid map.
     */
    private String solenoidKey;
    
    /**
     * The intended state for the solenoid.
     */
    private DoubleSolenoid.Value solenoidState;
    
    /**
     * Sets up the solenoid command, requiring the intake.
     * @param key the key in the solenoid map containing the solenoid to be modified
     * @param state the state of the solenoid (forward, off, or reverse)
     */
    public SetSolenoid(String key, DoubleSolenoid.Value state) 
    { 
        requires(Intake.pn); 
        solenoidKey = key;
        solenoidState = state;
    }
    
    /**
     * Sets a given solenoid to a given state.
     */
    public void initialize()
    {
        if (solenoidKey.equals(IntakeConstants.UPDOWN_KEY) && solenoidState.equals(IntakeConstants.DOWN) && Robot.el.getBottomRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX))
        Intake.pn.getSolenoid(solenoidKey).set(solenoidState); 
    }
}
