package org.usfirst.frc.team1072.robot;

import org.usfirst.frc.team1072.robot.commands.elevator.MoveElevatorMotionMagic;
import org.usfirst.frc.team1072.robot.commands.intake.IntakeOuttakeIndefinite;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoid;
import org.usfirst.frc.team1072.robot.commands.intake.SetSolenoidStealth;
import org.usfirst.frc.team1072.robot.commands.intake.ToggleSolenoid;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;
import org.usfirst.frc.team1072.robot.subsystems.Intake;
import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidDirection;
import org.usfirst.frc.team1072.robot.subsystems.Pneumatics.SolenoidType;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.DPadButtonWrapper;
import harkerrobolib.wrappers.GamepadWrapper;



/**
 * Keeps track of the IO of the robot, such as button mappings.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class OI
{
    /**
     * The port for the driver controller.
     */
    public static final int DRIVER_PORT = 0;

    /**
     * The port for the operator controller.
     */
    public static final int OPERATOR_PORT = 1;
    
    /**
     * The deadband (range for which no input is received) of the black XBOX controller.
     */
    public static final double BLACK_XBOX_DRIVE_DEADBAND = 0.09;

    public static final double BLACK_XBOX_ELEVATOR_DEADBAND = 0.15;

    /**
     * The deadband (range for which no input is received) of the Logitech controller.
     */
    public static final double LOGITECH_DEADBAND = 0.15;
    
    /**
     * The current instance of the singleton OI.
     */
    private static OI oi = null;
  
    /**
     * The gamepad for use by the driver.
     */
    private GamepadWrapper driverGamepad = new GamepadWrapper(DRIVER_PORT);
    
    /**
     * The gamepad for use by the operator.
     */
    private GamepadWrapper operatorGamepad = new GamepadWrapper(OPERATOR_PORT, GamepadWrapper.SETTING_LOGITECH);
    
    /**
     * Constructs a new OI.
     */
    public OI ()
    {
        DPadButtonWrapper upDPadDriver = new DPadButtonWrapper(driverGamepad, 0);
        DPadButtonWrapper leftDPadDriver = new DPadButtonWrapper (driverGamepad, 270);
        DPadButtonWrapper downDPadDriver= new DPadButtonWrapper (driverGamepad, 180);
        DPadButtonWrapper rightDPadDriver = new DPadButtonWrapper (driverGamepad, 90);
        
        upDPadDriver.whenPressed(new SetSolenoid(SolenoidDirection.UP));
        downDPadDriver.whenPressed( new SetSolenoid(SolenoidDirection.DOWN));
        leftDPadDriver.whenPressed(new SetSolenoid(SolenoidDirection.DECOMPRESS));
        rightDPadDriver.whenPressed(new SetSolenoid(SolenoidDirection.COMPRESS));
        
        if (RobotMap.TWO_CONTROLLERS)
        {
            DPadButtonWrapper upDPadOperator = new DPadButtonWrapper(operatorGamepad, 0);
            DPadButtonWrapper leftDPadOperator = new DPadButtonWrapper (operatorGamepad, 270);
            DPadButtonWrapper downDPadOperator = new DPadButtonWrapper (operatorGamepad, 180);
            DPadButtonWrapper rightDPadOperator = new DPadButtonWrapper (operatorGamepad, 90);

            operatorGamepad.getButtonBumperLeft().whenPressed(new ToggleSolenoid(SolenoidType.COMPRESSDECOMPRESS));
            operatorGamepad.getButtonBumperRight().whenPressed(new ToggleSolenoid(SolenoidType.COMPRESSDECOMPRESS));
            
            CommandGroup compressRaise = new CommandGroup();
            	compressRaise.addSequential(new ConditionalCommand(new MoveElevatorMotionMagic(Elevator.RAISE_HEIGHT)) {

					@Override
					protected boolean condition() {
                        Elevator.getInstance().getBottomRightTalon().configSelectedFeedbackSensor
                        (FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
                        System.out.println(Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition(0));
                        
                        return Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition(0) < 
								Elevator.RAISE_HEIGHT;
					}
            		
            	});
            	compressRaise.addSequential(new SetSolenoidStealth(SolenoidDirection.COMPRESS));
            upDPadOperator.whenPressed(new SetSolenoid(SolenoidDirection.UP));
            downDPadOperator.whenPressed( new SetSolenoid(SolenoidDirection.DOWN));
            leftDPadOperator.whenPressed(new SetSolenoid(SolenoidDirection.DECOMPRESS));
            rightDPadOperator.whenPressed(compressRaise);

            Command halfSpeed = new IntakeOuttakeIndefinite (0.4);
            operatorGamepad.getButtonA().whenPressed(halfSpeed);
            operatorGamepad.getButtonA().cancelWhenReleased(halfSpeed);

            Command threeQuarterSpeed = new IntakeOuttakeIndefinite(0.75);
            operatorGamepad.getButtonY().whenPressed(threeQuarterSpeed);
            operatorGamepad.getButtonY().cancelWhenReleased(threeQuarterSpeed);
        }
        
        CommandGroup lowerAndOpen = new CommandGroup();
            lowerAndOpen.addSequential(new SetSolenoidStealth(SolenoidDirection.DECOMPRESS));
            lowerAndOpen.addSequential(new MoveElevatorMotionMagic(Elevator.INTAKE_HEIGHT));
            lowerAndOpen.addSequential(new SetSolenoidStealth(SolenoidDirection.DOWN));
        driverGamepad.getButtonA().whenPressed(lowerAndOpen);
        
        CommandGroup raiseElevatorIntake = new CommandGroup();
        	raiseElevatorIntake.addParallel(new SetSolenoidStealth (SolenoidDirection.UP));
            raiseElevatorIntake.addParallel(new MoveElevatorMotionMagic(Elevator.SCALE_HIGH_HEIGHT));
            raiseElevatorIntake.addParallel(new InstantCommand() {
                public void initialize() {
                    System.out.println("Y PRESSED");
                }
            });
        driverGamepad.getButtonX().whenPressed(new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT));
        driverGamepad.getButtonB().whenPressed(new MoveElevatorMotionMagic(Elevator.SCALE_LOW_HEIGHT));
        driverGamepad.getButtonY().whenPressed(raiseElevatorIntake);     
    }

    /**
     * Gets the current instance of the driver gamepad.
     * @return the driver gamepad instance
     */
    public GamepadWrapper getDriverGamepad() 
    {
        return driverGamepad;
    }
    
    /**
     * Gets the current instance of the operator gamepad.
     * @return the operator gamepad instance
     */
    public GamepadWrapper getOperatorGamepad() 
    {
        return operatorGamepad;
    }
    
    /**
     * Gets the current instance of the singleton OI.
     * @return the current instance
     */
    public static OI getInstance()
    {
        if (oi == null)
            oi = new OI();
        return oi;
    }
}
