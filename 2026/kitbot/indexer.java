
package frc.robot;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
  public enum Mode {FORWARD, JAM, IDLE}
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX IndexMotor = new TalonFX(0, canivore);
  private final TalonFX shootMotor = new TalonFX(10, canivore);  // Initializes the shootMotor
  private final CANrange ShooterSensor = new CANrange(11, canivore);
  private final CANrange HopperSensor = new CANrange(12, canivore);
  private final Timer ShooterTimer = new Timer();
  private final Timer HopperTimer = new Timer();
  private final Timer JamTimer = new Timer();
  private final StatusSignal<Boolean> shooterFuelDetected;
  private final StatusSignal<Boolean> hopperFuelDetected;
  private Mode currState = Mode.IDLE;
  private boolean isShooting = false;
  
  // Initialize indexer: configure motor, start timer, configure sensor, and obtain fuelDetected data
  public Indexer() {
    configMotor();
    ShooterTimer.start();
    configCANrange(ShooterSensor);
    configCANrange(HopperSensor);
    shooterFuelDetected = ShooterSensor.getIsDetected();
    hopperFuelDetected = HopperSensor.getIsDetected();
  }

  // Runs code once at start: set current state to IDLE, stop shooting, stops motor, and reset the index timer
  public void init() {
    currState = Mode.IDLE;
    isShooting = false;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
    ShooterTimer.reset();
  }

  // Runs code periodically: refresh the fuel sensor, run the shooting/jam state machine, and set motor outputs for each state
  public void periodic() {
    shooterFuelDetected.refresh();
    hopperFuelDetected.refresh();
    if (getShooterSensor()){
     ShooterTimer.restart();
    }
    if (getHopperSensor()){
      HopperTimer.restart();
    }
    switch(currState) {
      case FORWARD://just going forward
        IndexMotor.setControl(new VelocityVoltage(10.0).withEnableFOC(true));
        if(HopperTimer.get() > 3.0 ){
          currState = Mode.IDLE;
        }
        else if(ShooterTimer.get() > 10.0 && HopperTimer.get() < 2.0 ){
          currState = Mode.JAM;
          JamTimer.restart();
        }
        break;
      case JAM://execute when sensor is on false for 5 second
        IndexMotor.setControl(new VelocityVoltage(-10.0).withEnableFOC(true));
        if (HopperTimer.get() >10.0 ) {
          currState = Mode.IDLE;
         }
         else if (ShooterTimer.get() <1.0 && JamTimer.get() >10.0){
          currState = Mode.FORWARD;
         }
        break;
      case IDLE://just stops
        IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
        break;
    }
  }

  // Marks the Indexer as running forward (not shooting) and resets the jam timer.
  public void start() {
    currState = Mode.FORWARD;
    isShooting = true;
    ShooterTimer.reset();
  }

  //Marks the indexer as idle, stops shooting, and reset the jam timer.
  public void stop() {
    currState = Mode.IDLE;
    isShooting = false;
    IndexMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
  }

  // Marks the Indexer as running forward, enable shooting mode, and reset the jam timer.
  public void startShooting() {
    currState = Mode.FORWARD;
    isShooting = true;
    ShooterTimer.reset();
  }
  
  // Mark the indexer as jammed, stop shooting, and reset the timer used for jam handling.
  public void jammed(){
    currState = Mode.JAM;
    isShooting = false;
    ShooterTimer.reset();
  }

  // Return elapsed time (seconds) from IndexTimer since last reset
  public double getSensorTimer() {
    return ShooterTimer.get();
  }

  // return a boolean if the sensor sense a fuel.
  public boolean getShooterSensor() {
    return shooterFuelDetected.getValue();
  }
  public boolean getHopperSensor() {
    return hopperFuelDetected.getValue();
  }

  // Publish indexer information (state, sensor, shooting flag, and timer) to SmartDashboard
  public void updateDash() {
    SmartDashboard.putString("Indexer State", currState.toString());
    SmartDashboard.putBoolean("Fuel Detected", getShooterSensor());
    SmartDashboard.putBoolean("Is Shooting", isShooting);
    SmartDashboard.putNumber("Indexer Sensor Timer", getSensorTimer());
  }
  
  // Configs the motor settings and PID
  private void configMotor() {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0;
    motorConfigs.Slot0.kP = 0.2;
    motorConfigs.Slot0.kI = 0.1;
    motorConfigs.Slot0.kD = 0.0;
    motorConfigs.Slot0.kV = 0.12;
    motorConfigs.Slot0.kS = 0.1;
    IndexMotor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  // Configs the sensor settings 
  private void configCANrange(CANrange sensor) {
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();
    sensorConfigs.ProximityParams.ProximityThreshold = 0.45;
    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
  
}
