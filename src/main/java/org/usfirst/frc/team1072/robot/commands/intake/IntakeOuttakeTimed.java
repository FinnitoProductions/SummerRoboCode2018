package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;
import org.usfirst.frc.team1072.util.Conversions;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Intakes and outtakes cubes in a timed manner (intended for autonomous).
 * @author Finn Frankis
 * @version 6/21/18
 */
public class IntakeOuttakeTimed extends Command
{
    /**
     * Whether to intake or outtake (true for intake, false for outtake).
     */
    private IntakeType intake;
    
    private double startTime;

    private double timeout;

    /**
     * Constructs a new IntakeOuttakeTimed.
     * @param timeout the time for which the intake should occur in seconds
     * @param intake the direction of intaking
     */
    public IntakeOuttakeTimed(double timeout, IntakeType intake)
    {
        startTime = Robot.getCurrentTimeMs();
        requires(Robot.intake);
        this.intake = intake;
        this.timeout = timeout;
    }
    
    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        if (intake == IntakeType.OUTTAKE)
        {
            Robot.intake.intakeOuttakeCube(Intake.INTAKE_DIR);
        }
        else if (intake == IntakeType.INTAKE)
            Robot.intake.intakeOuttakeCube(-Intake.INTAKE_DIR);
        else
            Robot.intake.intakeOuttakeCube(0);
    }

    protected boolean isFinished () {
        return Math.abs(Robot.getCurrentTimeMs() - startTime) >= timeout * Conversions.MS_PER_SEC;
    }
}
