package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents an intake subsystem with pneumatic and motor control.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Intake extends Subsystem
{
    public static Intake intake = null;
    public static Pneumatics pn = Pneumatics.getInstance();
    
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    /**
     * Initializes the command using the ports for the left and right Talons.
     */
    protected void initDefaultCommand()
    {
        // TODO Auto-generated method stub
        leftTalon = new TalonSRX(RobotMap.INTAKE_TALON_LEFT);
        rightTalon = new TalonSRX(RobotMap.INTAKE_TALON_RIGHT);
    }

    /**
     * Intakes/outakes a cube given a left and right speed (if two joysticks)
     * @param speedLeft the percent output for the left intake
     * @param speedRight the percent output for the right intake
     */
    public void intakeOuttakeCube(double speedLeft, double speedRight)
    {
        leftTalon.set(ControlMode.PercentOutput, speedLeft);
        rightTalon.set(ControlMode.PercentOutput, speedRight);
    }
    
    /**
     * Intakes/outakes a cube given a single speed for both halves.
     * @param speed the speed at which both halves should be run
     */
    public void intakeOuttakeCube (double speed)
    {
        leftTalon.set(ControlMode.PercentOutput, speed);
        rightTalon.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Gets the left Talon on the intake.
     * @return the left Talon
     */
    public TalonSRX getLeftTalon() { return leftTalon; }

    /**
     * Gets the right Talon on the intake.
     * @return the right Talon
     */
    public TalonSRX getRightTalon() { return rightTalon; }

    /**
     * Gets the instance of this singleton Intake, returning a new one if one has not yet been created.
     * @return this Intake instance
     */
    public static Intake getInstance()
    {
        if (intake == null) intake = new Intake();
        return intake;
    }


}
