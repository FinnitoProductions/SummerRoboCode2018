package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Intakes continuously until the cube has been successfully intaken.
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class IntakeUntilSuccessful extends Command
{
    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        Robot.intake.intakeOuttakeCube(-1);
    }
    /**
    * Determines whether the command has finished.
    * @return true if the current limit has exceeded the specified value; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        return Robot.intake.getLeftTalon().getOutputCurrent() > IntakeConstants.INTAKE_CURRENT_SPIKE;
    }
    
    protected void end ()
    {
        Robot.intake.intakeOuttakeCube(0);
    }
    
}
