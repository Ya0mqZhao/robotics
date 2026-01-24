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
  private final AnalogPotentiometer sensor = new AnalogPotentiometer(0, 5.0);
  
  public Indexer() {
    configMotor();
    IndexTimer.start();
    configCANrange(indexSensor);
    fuelDetected = indexSensor.getIsDetected();
  }

  public void init() {
    currState = Mode.IDLE;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
    IndexTimer.reset();
  }

  public void periodic() {
    if (getSensor() && IndexTimer.get() > 2) {//if sensor detects something and constantly detected for over 2 second execute mode.jam
      currState = Mode.JAM;
    }
      else{
        IndexTimer.reset();
      }
    
    if (currState == Mode.JAM && !getSensor()) {//codition to get out of mode.jam 
      currState = Mode.IDLE;
      
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
    IndexTimer.reset();
  }

  public void stop() {
    currState = Mode.IDLE;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
  }
  public void jammed(){
    currState = Mode.JAM;
    IndexTimer.reset();
  }
  public double getSensorTimer() {
    return 0.0;
  }

  public boolean getSensor() {
    return fuelDetected.refresh().getValue();
  }

  public void updateDash() {
    SmartDashboard.putString("Indexer State", currState.toString());
    SmartDashboard.putBoolean("Indexer Sensor", getSensor());
    SmartDashboard.putBoolean("Fuel Detected", getSensor());
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

    // Sets the proximity for how far away the fuel can be to get detected.
    sensorConfigs.ProximityParams.ProximityThreshold = 0.4;

    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
}
