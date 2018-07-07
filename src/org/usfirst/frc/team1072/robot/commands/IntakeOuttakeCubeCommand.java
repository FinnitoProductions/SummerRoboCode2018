package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents the command to control the process of intaking and outtaking cubes.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class IntakeOuttakeCubeCommand extends Command
{
    /**
     * Constructs a new command requiring the intake.
     */
    public IntakeOuttakeCubeCommand() 
    { 
        requires(Robot.intake); 
    }

    /**
     * Executes the intake/outtake command given a left and right speed, using the trigger.
     */
    public void execute() 
    {
        OI oi = OI.getInstance();
        if (oi.getGamepad().getLeftTriggerPressed())
            Robot.intake.intakeOuttakeCube(oi.getGamepad().getLeftTrigger());
        else if (oi.getGamepad().getRightTriggerPressed())
            Robot.intake.intakeOuttakeCube(-1 * oi.getGamepad().getRightTrigger());
        else
            Robot.intake.intakeOuttakeCube(0);
    }
    
    /**
     * Calls when the command is interrupted or cancelled.
     */
    @Override
    public void end()
    {
        Robot.intake.intakeOuttakeCube(0);
    }
    
    /**
     * Determines whether this command has finished.
     */
    protected boolean isFinished() { return false; }

}
