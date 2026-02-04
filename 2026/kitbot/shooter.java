package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
private final CANBus canivore = new CANBus("canivore");
private final TalonFX shootMotor = new TalonFX(10, canivore);  // Initializes the shootMotor
private final StatusSignal<AngularVelocity> shooterVelocity;
private final VelocityVoltage shooterMotorVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
boolean isSpinUp = false;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configMotor(shootMotor, false, 200.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    shooterVelocity = shootMotor.getVelocity();
  }

  //Turns on motor
    public void spinUp() {
     spinUpAtRPM(4800);//need to be tested or else switch back to 👇
     //shootMotor.setControl(shooterMotorVelocityRequest.withVelocity(80).withEnableFOC(true));
    }


  // Turn off motor
  public void spinDown() {
    shootMotor.setControl(shooterMotorVelocityRequest.withVelocity(0.0).withEnableFOC(true));
    isSpinUp = false;
  }

  public void spinUpAtRPM(double rpm) {
    double rps = rpm / 60.0;
    shootMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
    isSpinUp = true;
  }

  // Publish Shooter information (Motor state, Velocity) to SmartDashboard.
  public void updateDash() {
    SmartDashboard.putBoolean("Shoot motor is on", isSpinUp);
    //SmartDashboard.putNumber("Shooter Velocity (RPS)", getVelocity());
   // SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocityRPM());
  }
  // Returns the motor velocity as a double in RPS (Rotations Per Second)
  public double getVelocity() {
    return shooterVelocity.refresh().getValueAsDouble();
  }
  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getVelocityRPM() {
    return getVelocity() * 60.0;
  }

  // Configs the motor settings and PID
  private void configMotor(TalonFX motor, boolean invert, double currentLimit) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Current limit configuration.
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}
