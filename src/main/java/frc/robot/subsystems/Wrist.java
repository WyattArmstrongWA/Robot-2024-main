package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;

public class Wrist extends SubsystemBase {

    public static final double MAX_ANGLE = 4.2;

    private final TalonFX wristMotor = new TalonFX(14);
    private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, false);

  
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  /* Start at position 0, no feed forward, use slot 1 */
  /* Keep a brake request so we can disable the motor */

  private final Mechanisms m_mechanism = new Mechanisms();

  double desiredRotations;


public void ShootAngleinit() {
     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
     TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = .5; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 4;
    configs.Voltage.PeakReverseVoltage = -4;
    
    configs.Slot1.kP = 3; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 30;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 30;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = wristMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    wristMotor.setPosition(0);
}

public void ShootAnglePeriodic() {
    m_mechanism.update(wristMotor.getPosition());
}

public void ShootAngleSet(double angle) {
    if (angle > MAX_ANGLE){
        desiredRotations = MAX_ANGLE;
    }else if (angle < 0) {
        desiredRotations = 0;
    }else {
    desiredRotations = angle;
    }
    wristMotor.setControl(m_voltagePosition.withPosition(desiredRotations));
}

}