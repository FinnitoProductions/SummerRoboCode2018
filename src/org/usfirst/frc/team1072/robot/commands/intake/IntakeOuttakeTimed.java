package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;
import org.usfirst.frc.team1072.robot.subsystems.Intake.IntakeType;

import edu.wpi.first.wpilibj.command.TimedCommand;

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
    }
    
    /**
     * Executes the command periodically.
     */
    public void execute()
    {
        if (intake == IntakeType.INTAKE)
        {
            Robot.intake.intakeOuttakeCube(IntakeConstants.INTAKE_DIR);
        }
        else if (intake == IntakeType.OUTTAKE)
            Robot.intake.intakeOuttakeCube(-IntakeConstants.INTAKE_DIR);
        else
            Robot.intake.intakeOuttakeCube(0);
    }
}
