package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.commands.DriveToPositionCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorMotionMagicCommand;
import org.usfirst.frc.team1072.robot.commands.MoveElevatorVelocityCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Represents an Elevator subsystem with control over the four motors.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class Elevator extends Subsystem
{
    private static Elevator el;
    
    private VictorSPX topRightVictor;
    private VictorSPX bottomLeftVictor;
    private VictorSPX topLeftVictor;
    private TalonSRX bottomRightTalon; 
    
    /**
     * Initializes the command using the four ports in RobotMap.
     */
    public Elevator()
    {
        topRightVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_TOPRIGHT);
        topLeftVictor = new VictorSPX (RobotMap.ELEVATOR_VICTOR_TOPLEFT);
        bottomLeftVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_BOTTOMLEFT);
        bottomRightTalon = new TalonSRX(RobotMap.ELEVATOR_TALON);
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(new MoveElevatorVelocityCommand());
    }
    
    /**
     * Moves the elevator given a velocity.
     * @param speed the speed with which the elevator will be moved
     */
    public void moveElevatorVelocity(double speed)
    {
        // feed forward counterracts gravity
        bottomRightTalon.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Moves the elevator given a position setpoint.
     * @param position the position to which the elevator will be moved
     */
    public void moveElevatorPosition(double position)
    {
        bottomRightTalon.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Moves the elevator using motion magic. 
     * @param targetPos the position to which the elevatr will be moved
     */
    public void moveElevatorMotionMagic(double targetPos)
    {
        bottomRightTalon.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    /**
     * Gets the top right Victor on the elevator.
     * @return the top right Victor
     */
    public VictorSPX getTopRightVictor() { return topRightVictor; }

    /**
     * Gets the bottom left Victor on the elevator.
     * @return the bottom left Victor
     */
    public VictorSPX getBottomLeftVictor() { return bottomLeftVictor; }

    /**
     * Gets the top left Victor on the elevator.
     * @return the top left Victor
     */
    public VictorSPX getTopLeftVictor() { return topLeftVictor; }

    /**
     * Gets the bottom right Talon on the elevator.
     * @return the bottom right Talon
     */
    public TalonSRX getBottomRightTalon() { return bottomRightTalon; }

    /**
     * Gets the instance of the singleton Elevator, creating a new one if necessary.
     * @return the instance of Elevator
     */
    public static Elevator getInstance()
    {
        if (el == null) el = new Elevator();
        return el;
    }
    
}
