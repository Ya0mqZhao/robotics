package frc.robot;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
  public enum Mode {FORWARD, JAM, IDLE}
  private final TalonFX IndexMotor = new TalonFX(0, "canivore");
  private final CANrange indexSensor = new CANrange(0);
  private final Timer IndexTimer = new Timer();
  private final StatusSignal<Boolean> fuelDetected;
  private Mode currState = Mode.IDLE;
  private boolean isShooting = false;
  
  public Indexer() {
    configMotor();
    IndexTimer.start();
    configCANrange(indexSensor);
    fuelDetected = indexSensor.getIsDetected();
  }

  public void init() {
    currState = Mode.IDLE;
    isShooting = false;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
    IndexTimer.reset();
  }

  public void periodic() {
    fuelDetected.refresh();
    
    if (isShooting && currState == Mode.FORWARD) {
      if (!getSensor() && IndexTimer.get() > 5.0) {
        currState = Mode.JAM;
        IndexTimer.reset();
      } else if (getSensor()) {
        IndexTimer.reset();
      }
    }
    
    if (currState == Mode.JAM && IndexTimer.get() > 5.0) {
      currState = Mode.IDLE;
      isShooting = false;
    }

    switch(currState) {
      case FORWARD:
        IndexMotor.setControl(new VelocityVoltage(-10.0).withEnableFOC(true));
        break;
      case JAM:
        IndexMotor.setControl(new VelocityVoltage(20.0).withEnableFOC(true));
        break;
      case IDLE:
        IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
        break;
    }
  }

  public void start() {
    currState = Mode.FORWARD;
    isShooting = false;
    IndexTimer.reset();
  }

  public void stop() {
    currState = Mode.IDLE;
    isShooting = false;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
  }

  public void startShooting() {
    currState = Mode.FORWARD;
    isShooting = true;
    IndexTimer.reset();
  }
  
  public void jammed(){
    currState = Mode.JAM;
    isShooting = false;
    IndexTimer.reset();
  }

  public double getSensorTimer() {
    return IndexTimer.get();
  }

  public boolean getSensor() {
    return fuelDetected.getValue();
  }

  public void updateDash() {
    SmartDashboard.putString("Indexer State", currState.toString());
    SmartDashboard.putBoolean("Indexer Sensor", getSensor());
    SmartDashboard.putBoolean("Fuel Detected", getSensor());
    SmartDashboard.putBoolean("Is Shooting", isShooting);
    SmartDashboard.putNumber("Indexer Sensor Timer", getSensorTimer());
  }
    
  private void configMotor() {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0;
    motorConfigs.Slot0.kP = 0.2;
    motorConfigs.Slot0.kI = 0.1;
    motorConfigs.Slot0.kD = 0.0;
    motorConfigs.Slot0.kV = 0.12;
    motorConfigs.Slot0.kS = 0.1;
    IndexMotor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  private void configCANrange(CANrange sensor) {
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();
    sensorConfigs.ProximityParams.ProximityThreshold = 0.4;
    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
}
