package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Intakes and outtakes cubes in a timed manner (intended for autonomous).
 * @author Finn Frankis
 * @version 6/21/18
 */
public class IntakeOuttakeTimedCommand extends TimedCommand
{
    private boolean intake;
    
    public IntakeOuttakeTimedCommand(double timeout, boolean intake)
    {
        super(timeout);
        requires(Robot.intake);
        this.intake = intake;
    }
    
    public void execute()
    {
        if (intake)
        {
            Robot.intake.intakeOuttakeCube(1);
        }
        else
            Robot.intake.intakeOuttakeCube(-1);
    }
}
