package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeOuttakeCubeCommand extends Command
{

    public IntakeOuttakeCubeCommand()
    {
        requires(Robot.intake);
    }
    public void execute(double leftSpeed, double rightSpeed)
    {
        Robot.intake.intakeOuttakeCube(leftSpeed, rightSpeed);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }

}
