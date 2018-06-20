package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.DriveWithVelocityCommand;
import org.usfirst.frc.team1072.robot.commands.TurnRobotToAngleCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents a drive train with two Talons and two Victors.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class Drivetrain extends Subsystem
{
    private static Drivetrain instance = null;
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    private VictorSPX leftVictor;
    private VictorSPX rightVictor;
    private PigeonIMU pigeon;
    
    public Drivetrain()
    {
        // initialize talons
        leftTalon = new TalonSRX (RobotMap.LEFT_CIM_TALON);
        rightTalon = new TalonSRX (RobotMap.RIGHT_CIM_TALON);
        leftVictor = new VictorSPX (RobotMap.LEFT_CIM_VICTOR);
        rightVictor = new VictorSPX (RobotMap.RIGHT_CIM_VICTOR);
        pigeon = new PigeonIMU(Robot.intake.getRightTalon());
        

    }
    /**
     * Initializes the command using the four ports for the Talons/Victors.
     */
    public void initDefaultCommand()
    {
        setDefaultCommand(new DriveWithVelocityCommand());
        //setDefaultCommand(new TurnRobotToAngleCommand());
    }
    
    /**
     * Runs both motor controllers given a velocity and turn.
     * @param speed the speed at which the controllers should run
     * @param turn the amount by which each speed should be modified to account for turn
     */
    public void arcadeDriveVelocity(double speed, double turn)
    {
        // victor follows talon
        //leftTalon.set(ControlMode.Velocity, speed - turn);
        rightTalon.set(ControlMode.Velocity, speed + turn);  
        leftTalon.set(ControlMode.Velocity, speed - turn);   
    }
    
    /**
     * Moves the robot to a given position using PID.
     * @param target the target position 
     */
    public void arcadeDrivePosition (double target)
    {
        leftTalon.set(ControlMode.Position, target);
        rightTalon.set(ControlMode.Position, target);
    }
    
    /**
     * Gets the left Talon on the drivetrain.
     * @return the left Talon
     */
    public TalonSRX getLeftTalon() { return leftTalon; }

    /**
     * Gets the right Talon on the drivetrain.
     * @return the right Talon
     */
    public TalonSRX getRightTalon() { return rightTalon; }

    /**
     * Gets the left Victor on the drivetrain.
     * @return the left Victor
     */
    public VictorSPX getLeftVictor() { return leftVictor; }
    
    /**
     * Gets the right Victor on the drivetrain.
     * @return the right Victor.
     */
    public VictorSPX getRightVictor() { return rightVictor; }

    /**
     * Gets the instance of this Drivetrain, creating a new one if necessary.
     * @return the instance of this singleton class
     */
    public static Drivetrain getInstance()
    {
        if (instance == null) instance = new Drivetrain();
        return instance;
    }
    
    /**
     * Gets the pigeon for this drivetrain.
     * @return the pigeon
     */
    public PigeonIMU getPigeon()
    {
        return pigeon;
    }

}
