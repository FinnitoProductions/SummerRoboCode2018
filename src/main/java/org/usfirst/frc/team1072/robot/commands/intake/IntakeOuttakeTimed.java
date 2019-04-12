package org.usfirst.frc.team1072.robot.commands.intake;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Intakes and outtakes cubes in a timed manner (intended for autonomous).
 * @author Finn Frankis
 * @version 6/21/18
 */
public class IntakeOuttakeTimed extends TimedCommand
{
    /**
     * Whether to intake or outtake (true for intake, false for outtake).
     */
    private IntakeType intake;
    
    private double startTime;

    private double timeout;

    private double speed;

    /**
     * Constructs a new IntakeOuttakeTimed.
     * @param timeout the time for which the intake should occur in seconds
     * @param intake the direction of intaking
     */
    public IntakeOuttakeTimed(double timeout, IntakeType intake)
    {
        super(timeout);
        requires(Robot.intake);
        this.intake = intake;
        this.timeout = timeout;
    }

    public IntakeOuttakeTimed(double timeout, IntakeType intake, double speed)
    {
        this(timeout, intake);
        this.speed = speed;
    }
    
    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        if (intake == IntakeType.OUTTAKE)
        {
            Robot.intake.intakeOuttakeCube(Intake.INTAKE_DIR * speed);
        }
        else if (intake == IntakeType.INTAKE)
            Robot.intake.intakeOuttakeCube(-Intake.INTAKE_DIR * speed);
        else
            Robot.intake.intakeOuttakeCube(0);
    }
}
