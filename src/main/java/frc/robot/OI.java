package frc.robot;

import harkerrobolib.auto.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.intake.IntakeOuttakeIndefinite;
import frc.robot.commands.intake.SetSolenoid;
import frc.robot.commands.intake.SetSolenoidStealth;
import frc.robot.commands.intake.ToggleSolenoid;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics.SolenoidDirection;
import frc.robot.subsystems.Pneumatics.SolenoidType;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import harkerrobolib.wrappers.HSDPadButton;
import harkerrobolib.wrappers.HSGamepad;
import harkerrobolib.wrappers.LogitechGamepad;
import harkerrobolib.wrappers.XboxGamepad;



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
    public static final double BLACK_XBOX_DRIVE_DEADBAND = 0.04;

    public static final double BLACK_XBOX_ELEVATOR_DEADBAND = 0.08;

    public static final double DRIVER_TRIGGER_DEADBAND = 0.15;
    /**
     * The deadband (range for which no input is received) of the Logitech controller.
     */
    public static final double LOGITECH_TRIGGER_DEADBAND = 0.15;
    
    /**
     * The current instance of the singleton OI.
     */
    private static OI oi = null;
  
    /**
     * The gamepad for use by the driver.
     */
    private HSGamepad driverGamepad = new XboxGamepad(DRIVER_PORT);
    
    /**
     * The gamepad for use by the operator.
     */
    private HSGamepad operatorGamepad = new LogitechGamepad(OPERATOR_PORT);
    
    /**
     * Constructs a new OI.
     */
    public OI ()
    {
        HSDPadButton upDPadDriver = new HSDPadButton(driverGamepad, 0);
        HSDPadButton leftDPadDriver = new HSDPadButton (driverGamepad, 270);
        HSDPadButton downDPadDriver= new HSDPadButton (driverGamepad, 180);
        HSDPadButton rightDPadDriver = new HSDPadButton (driverGamepad, 90);
        
        HSDPadButton upDPadOperator = new HSDPadButton(operatorGamepad, 0);
        HSDPadButton leftDPadOperator = new HSDPadButton (operatorGamepad, 270);
        HSDPadButton downDPadOperator = new HSDPadButton (operatorGamepad, 180);
        HSDPadButton rightDPadOperator = new HSDPadButton (operatorGamepad, 90);
        
        CommandGroup compressRaise = new CommandGroup();
    	compressRaise.addSequential(new ConditionalCommand(new MoveElevatorMotionMagic(Elevator.RAISE_HEIGHT)) {

			@Override
			protected boolean condition() {
                Elevator.getInstance().getBottomRightTalon().configSelectedFeedbackSensor
                (FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
                System.out.println(Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition(0));
                
                return Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition(0) < 
						Elevator.RAISE_HEIGHT;
			}
    		
    	});
    	compressRaise.addSequential(new SetSolenoidStealth(SolenoidDirection.COMPRESS));
        
        upDPadDriver.whenPressed(new SetSolenoid(SolenoidDirection.UP));
        downDPadDriver.whenPressed( new SetSolenoid(SolenoidDirection.DOWN));
        leftDPadDriver.whenPressed(new SetSolenoid(SolenoidDirection.DECOMPRESS));
        rightDPadDriver.whenPressed(compressRaise);
        
        if (RobotMap.TWO_CONTROLLERS)
        {
            operatorGamepad.getButtonBumperLeft().whenPressed(new ToggleSolenoid(SolenoidType.COMPRESSDECOMPRESS));
            operatorGamepad.getButtonBumperRight().whenPressed(new ToggleSolenoid(SolenoidType.COMPRESSDECOMPRESS));
            
            
            upDPadOperator.whenPressed(new SetSolenoid(SolenoidDirection.UP));
            downDPadOperator.whenPressed( new SetSolenoid(SolenoidDirection.DOWN));
            leftDPadOperator.whenPressed(new SetSolenoid(SolenoidDirection.DECOMPRESS));
            rightDPadOperator.whenPressed(compressRaise);

            operatorGamepad.getButtonA().whilePressed(new SequentialCommandGroup(new SetSolenoid(SolenoidDirection.DECOMPRESS),
                                                        new IntakeOuttakeIndefinite (0.4)));
            operatorGamepad.getButtonY().whilePressed(new SequentialCommandGroup(new SetSolenoid(SolenoidDirection.DECOMPRESS),
                                                        new IntakeOuttakeIndefinite(0.75)));
        }
        
        CommandGroup lowerAndOpen = new CommandGroup();
            lowerAndOpen.addSequential(new SetSolenoidStealth(SolenoidDirection.DECOMPRESS));
            lowerAndOpen.addSequential(new MoveElevatorMotionMagic(Elevator.INTAKE_HEIGHT));
            lowerAndOpen.addSequential(new SetSolenoidStealth(SolenoidDirection.DOWN));
        // driverGamepad.getButtonA().whenPressed(lowerAndOpen);
        
        CommandGroup raiseElevatorIntake = new CommandGroup();
        	raiseElevatorIntake.addParallel(new SetSolenoidStealth (SolenoidDirection.UP));
            raiseElevatorIntake.addParallel(new MoveElevatorMotionMagic(Elevator.SCALE_HIGH_HEIGHT));
            raiseElevatorIntake.addParallel(new InstantCommand() {
                public void initialize() {
                    System.out.println("Y PRESSED");
                }
            });

        driverGamepad.getButtonStart().whenPressed(new InstantCommand() {
            public void initialize() {
                RobotMap.SAFETY_MODE = (RobotMap.SAFETY_MODE == RobotMap.SafetyMode.SAFE ? RobotMap.SafetyMode.NOT_SAFE : RobotMap.SafetyMode.SAFE);
            }
        });
        // driverGamepad.getButtonX().whenPressed(new MoveElevatorMotionMagic(Elevator.SWITCH_HEIGHT));
        // driverGamepad.getButtonB().whenPressed(new MoveElevatorMotionMagic(Elevator.SCALE_LOW_HEIGHT));
        // //driverGamepad.getButtonY().whenPressed(new ZeroElevator());
        // driverGamepad.getButtonStickRight().whenPressed(new ZeroElevator());
        // driverGamepad.getButtonY().whenPressed(raiseElevatorIntake);
    }

    /**
     * Gets the current instance of the driver gamepad.
     * @return the driver gamepad instance
     */
    public HSGamepad getDriverGamepad() 
    {
        return driverGamepad;
    }
    
    /**
     * Gets the current instance of the operator gamepad.
     * @return the operator gamepad instance
     */
    public HSGamepad getOperatorGamepad() 
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
