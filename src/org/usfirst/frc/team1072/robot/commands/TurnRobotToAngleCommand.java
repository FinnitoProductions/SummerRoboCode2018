package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnRobotToAngleCommand extends Command
{
    private double angle;
    public TurnRobotToAngleCommand(double angle)
    {
        requires (Robot.dt);
        this.angle = angle;
    }
    
    @Override
    public void execute()
    {
        OI oi = new OI();
        /*double joystickLeft =  oi.getGamepad().getLeftY();
        double joystickRight = oi.getGamepad().getRightY() * 180.0;
        Robot.dt.getRightTalon().set(ControlMode.PercentOutput, joystickLeft, DemandType.AuxPID, joystickRight);
        Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, joystickLeft, DemandType.AuxPID, joystickRight);*/
        
        
        double[] ypr = new double[3];
        Robot.dt.getPigeon().getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Pigeon Yaw (from COMMAND)", ypr[0]);
        if (Math.abs(ypr[0] - angle) > .7)
        {
            double turnSpeed = .3;//Robot.speedToEncoderUnits(-0.1 * oi.getGamepad().getLeftX() * RobotMap.MAX_TURN_SPEED);
            System.out.println("not there yet");
            if (angle < ypr[0])
            {
                Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, turnSpeed);
                Robot.dt.getRightTalon().set(ControlMode.PercentOutput, -turnSpeed);
            }
            else if (angle > ypr[0])
            {
                Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, -turnSpeed);
                Robot.dt.getRightTalon().set(ControlMode.PercentOutput, turnSpeed);
            }
        }
        else
            Robot.dt.arcadeDriveVelocity(0, 0);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }

}
