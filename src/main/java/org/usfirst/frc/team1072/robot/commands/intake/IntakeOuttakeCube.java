package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents the command to control the process of intaking and outtaking cubes.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class IntakeOuttakeCube extends Command
{
    /**
     * Constructs a new command requiring the intake.
     */
    public IntakeOuttakeCube() 
    { 
        requires(Robot.intake); 
    }

    /**
     * Executes the intake/outtake command given a left and right speed, using the trigger.
     */
    public void execute() 
    {
        OI oi = OI.getInstance();
        boolean intakeEnabled = false;
        if (RobotMap.TWO_CONTROLLERS)
        {
            double leftInput = oi.getOperatorGamepad().getLeftY();
            double rightInput = oi.getOperatorGamepad().getRightY();
            if (Math.abs(leftInput) > OI.LOGITECH_DEADBAND || Math.abs(rightInput) > OI.LOGITECH_DEADBAND)
            {
                Robot.intake.setLeft(Math.abs(leftInput) > OI.LOGITECH_DEADBAND ? -leftInput * Math.abs(leftInput) * 0.8 : 0);
                Robot.intake.setRight(Math.abs(rightInput) > OI.LOGITECH_DEADBAND ? -rightInput * Math.abs(rightInput) * 0.8 : 0);
                intakeEnabled = true;
            }
        }
        if (!intakeEnabled)
        {
            if (oi.getDriverGamepad().getRightTriggerPressed())
                Robot.intake.intakeOuttakeCube(Intake.INTAKE_DIR * oi.getDriverGamepad().getRightTrigger());
            else if (oi.getDriverGamepad().getLeftTriggerPressed())
                Robot.intake.intakeOuttakeCube(-Intake.INTAKE_DIR * oi.getDriverGamepad().getLeftTrigger());
            else
                Robot.intake.intakeOuttakeCube(0);
        }
        
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
