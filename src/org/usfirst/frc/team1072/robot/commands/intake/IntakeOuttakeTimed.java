package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Intakes and outtakes cubes in a timed manner (intended for autonomous).
 * @author Finn Frankis
 * @version 6/21/18
 */
public class IntakeOuttakeTimed extends TimedCommand
{
    private IntakeType intake;
    
    /**
     * Represents the various possible functions performable by the intake.
     * @author Finn Frankis
     * @version Jul 24, 2018
     */
    public enum IntakeType
    {
        INTAKE, OUTTAKE, NONE
    }
    /**
     * Constructs a new IntakeOuttakeTimed.
     * @param timeout the time for which the intake should occur in seconds
     * @param intake whether to intake forward or backward
     */
    public IntakeOuttakeTimed(double timeout, IntakeType intake)
    {
        super(timeout);
        requires(Robot.intake);
        this.intake = intake;
    }
    
    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        if (intake.equals(IntakeType.INTAKE))
        {
            Robot.intake.intakeOuttakeCube(1);
        }
        else if (intake.equals(IntakeType.OUTTAKE))
            Robot.intake.intakeOuttakeCube(-1);
        else
            Robot.intake.intakeOuttakeCube(0);
    }
}
