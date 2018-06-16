package org.usfirst.frc.team1072.robot.commands;

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
    public IntakeOuttakeCubeCommand() { requires(Robot.intake); }
    
    /**
     * Executes the intake/outtake command given a left and right speed.
     * @param leftSpeed the speed of the left intake
     * @param rightSpeed the speed of the right intake
     */
    public void execute(double leftSpeed, double rightSpeed) {
        Robot.intake.intakeOuttakeCube(leftSpeed, rightSpeed);
    }
    
    /**
     * Executes the intake/outtake command given a general speed (to be used if only 
     * one joystick is used for control).
     * @param speed the speed for both sides
     */
    public void execute (double speed) { Robot.intake.intakeOuttakeCube(speed); }

    /**
     * Determines whether this command has finished.
     */
    protected boolean isFinished() { return true; }

}
