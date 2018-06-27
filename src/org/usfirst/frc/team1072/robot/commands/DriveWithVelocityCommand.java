package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Commands the drivetrain to drive with a given velocity. Vel PID
 * 
 * @author Finn Frankis
 * @version 6/11/18
 */
public class DriveWithVelocityCommand extends Command
{
    private double driveSpeed;
    private double turnSpeed;
    /**
     * Creates a new DriveWithVelocityCommand object requiring the Drivetrain.
     */
    public DriveWithVelocityCommand() 
    { 
        requires(Robot.dt); 
    }
    
    /**
     * Executes the command to drive with a given velocity.
     * @param speed the speed at which the robot will drive
     * @param turn the amount by which the robot should turn
     */
    @Override
    public void execute() 
    { 
        OI oi = OI.getInstance();
        driveSpeed = Robot.speedToEncoderUnits(oi.getGamepad().getLeftY() * RobotMap.MAX_DRIVE_SPEED); 
        turnSpeed = Robot.speedToEncoderUnits(-1 * oi.getGamepad().getLeftX() * RobotMap.MAX_TURN_SPEED);
        if (Math.abs(oi.getGamepad().getLeftX()) < 0.1)
            turnSpeed = 0;
        Robot.dt.arcadeDriveVelocity(
                driveSpeed, 
                turnSpeed); 
        
        SmartDashboard.putNumber("LEFT Motor Voltage", driveSpeed - Robot.dt.getLeftTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("RIGHT Motor Voltage", driveSpeed - Robot.dt.getRightTalon().getMotorOutputVoltage());
        SmartDashboard.putNumber("LEFT TALON ERROR", driveSpeed - Robot.dt.getLeftTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
        SmartDashboard.putNumber("RIGHT TALON ERROR", driveSpeed - Robot.dt.getRightTalon().getSelectedSensorVelocity(RobotMap.DT_VEL_PID));
    }
    
    public double getDriveSpeed()
    {
        return driveSpeed;
    }
    
    public double getTurnSpeed()
    {
        return turnSpeed;
    }
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return true; }

}
