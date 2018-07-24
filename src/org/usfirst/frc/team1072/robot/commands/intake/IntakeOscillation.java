package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap.IntakeConstants;

/**
 * Intakes a cube with oscillations, to better dislodge it from a diagonal position.
 * @author Finn Frankis
 * @version Jul 24, 2018
 */
public class IntakeOscillation extends IntakeOuttakeTimed
{
    private double startTime;
    private boolean isIntaking;
    private double intakePeriod;
    private double stopPeriod;
    
    /**
     * Constructs a new IntakeOscillation.
     * @param intakePeriod the time (in ms) which should be spent intaking for every given oscillation
     * @param stopPeriod the time (in ms) at which the intake should stop for every given oscillation
     * @param timeout the total time (in seconds) for which the command should run
     */
    public IntakeOscillation (double intakePeriod, double stopPeriod, double timeout)
    {
        super (timeout, IntakeType.INTAKE);
        this.intakePeriod = intakePeriod;
        this.stopPeriod = stopPeriod;
    }
    
    public void initialize()
    {
        startTime = Robot.getCurrentTimeMs();
        Robot.intake.intakeOuttakeCube(IntakeConstants.INTAKE_SIGN);
        isIntaking = true;
    }
    
    public void execute()
    {
        if (isIntaking && timeElapsedSinceChange() > intakePeriod)
        {
            startTime = Robot.getCurrentTimeMs();
            Robot.intake.intakeOuttakeCube(0);
            isIntaking = false;
        }
        else if (!isIntaking && timeElapsedSinceChange() > stopPeriod)
        {
            startTime = Robot.getCurrentTimeMs();
            Robot.intake.intakeOuttakeCube(IntakeConstants.INTAKE_SIGN);
            isIntaking = true;
        }
    }
    
    /**
     * Gets the total time (in ms) elapsed since the most recent change has occurred.
     * @return the total time elapsed since the last change
     */
    private double timeElapsedSinceChange()
    {
        return Robot.getCurrentTimeMs() - startTime;
    }
}
