package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driverController = new XboxController(0);
   // private final XboxController operatorController = new XboxController(1);
    public ShootAngle m_Shootangle = new ShootAngle(); 
    double angle = 4;


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
     private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kStart.value);

    /* Subsystems */
    
    private final Swerve s_Swerve = new Swerve();

    private final TalonFX intake = new TalonFX(17);
    
    private void intakeOnCommand() {
        intake.set(0.95);
    }
     private void intakeOffCommand() {
        intake.set(0);
    }
    private void outtakeOnCommand() {
        intake.set(-0.75);
    }
     private void outtakeOffCommand() {
        intake.set(0);
    }

    
    private final TalonFX shooterFeed = new TalonFX(60);
    private final CANSparkFlex shooterLeft = new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex shooterRight = new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
    
    private void shooterOnCommand() {
        shooterFeed.set(-1);
        shooterLeft.set(-1);
        shooterRight.set(-.95);
    }
     private void shooterOffCommand() {
        shooterFeed.set(0);
        shooterLeft.set(0);
        shooterRight.set(0);
    }
  

    private final TalonFX wrist = new TalonFX(14);

    private void wristUpOnCommand() {
        wrist.set(0.2);
    }
     private void wristUpOffCommand() {
        wrist.set(0);
    }
    private void wristDownOnCommand() {
        wrist.set(-0.2);
    }
     private void wristDownOffCommand() {
        wrist.set(0);
    }

    private void shooterUpCommand() {
        m_Shootangle.ShootAngleChange(0.1);
    }

    private void shooterDownCommand() {
        m_Shootangle.ShootAngleChange(-0.1);
    }


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverController.getRawAxis(translationAxis), 
                () -> -driverController.getRawAxis(strafeAxis), 
                () -> -driverController.getRawAxis(rotationAxis) * -1,
                () -> robotCentric.getAsBoolean()
            )
        );
     //shooterFeed.setInverted(true);
       
   
        // Configure the button bindings
        configureButtonBindings();
    }
   
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        new Trigger(driverController::getBackButton)
                .onTrue(new InstantCommand(
                        () -> s_Swerve.zeroHeading()
                ));

        new Trigger(driverController::getAButton)
                .whileTrue(new InstantCommand(
                        () -> intakeOnCommand()
                ));

        new Trigger(driverController::getAButtonReleased)
                .onTrue(new InstantCommand(
                        () -> intakeOffCommand()
                ));

        new Trigger(driverController::getYButton)
                .whileTrue(new InstantCommand(
                        () -> outtakeOnCommand()
                ));

        new Trigger(driverController::getYButtonReleased)
                .onTrue(new InstantCommand(
                        () -> outtakeOffCommand()
                ));

        new Trigger(driverController::getXButton)
                .whileTrue(new InstantCommand(
                        () -> shooterOnCommand()
                ));

        new Trigger(driverController::getXButtonReleased)
                .onTrue(new InstantCommand(
                        () -> shooterOffCommand()
                ));

        new Trigger(driverController::getLeftBumper)
                .whileTrue(new InstantCommand(
                        () -> wristDownOnCommand()
                ));

        new Trigger(driverController::getLeftBumperReleased)
                .onTrue(new InstantCommand(
                        () -> wristDownOffCommand()
                ));

        new Trigger(driverController::getRightBumper)
                .whileTrue(new InstantCommand(
                        () -> wristUpOnCommand()
                ));

        new Trigger(driverController::getRightBumperReleased)
                .whileTrue(new InstantCommand(
                        () -> wristUpOffCommand()
                ));
       
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.2)
                .onTrue(new InstantCommand(
                        () -> shooterUpCommand()
                ));

        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2)
                .onTrue(new InstantCommand(
                        () -> shooterDownCommand()
                ));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new LeaveZone(s_Swerve);
    }
}
