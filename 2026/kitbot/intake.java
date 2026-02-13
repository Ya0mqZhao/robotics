package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
  public enum Mode {HOME, LEFT, RIGHT, STOW}
  public Mode currMode = Mode.HOME;
  public double desiredLeftArmPosition = 0.0;
  public double desiredLeftRollerVelocity = 0.0;
  public double desiredRightArmPosition = 0.0;
  public double desiredRightRollerVelocity = 0.0;
  
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX rightIntakeDeploy = new TalonFX(14, canivore);
  private final TalonFX rightIntake = new TalonFX(15, canivore);
  private final TalonFX leftIntakeDeploy = new TalonFX(16, canivore);
  private final TalonFX leftIntake = new TalonFX(17, canivore);
  // These are MaxSplineThroughbore Encoders, not CANrange
  private final CANSparkMax leftArmSensor = new CANSparkMax(29, MotorType.kBrushless);
  private final CANSparkMax rightArmSensor = new CANSparkMax(30, MotorType.kBrushless);
  private final Timer intakeTimer = new Timer();
  private boolean isHomed = false;
  
  // Control requests - separate requests per motor
  private final PositionVoltage leftArmPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage rightArmPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage leftRollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage rightRollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut leftArmVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightArmVoltageRequest = new VoltageOut(0.0);
  
  // Status signals to avoid garbage collection
  private final StatusSignal<AngularVelocity> leftArmVelocity;
  private final StatusSignal<AngularVelocity> rightArmVelocity;
  private final StatusSignal<Angle> leftArmEncoderPosition;
  private final StatusSignal<Angle> rightArmEncoderPosition;

  public Intake() {
    configMotor(rightIntakeDeploy, true, 40.0, true);
    configMotor(rightIntake, false, 60.0, false);
    configMotor(leftIntakeDeploy, false, 40.0, true);
    configMotor(leftIntake, true, 60.0, false);
    configCANrange(leftArmSensor);
    configCANrange(rightArmSensor);
    
    // Initialize status signals
    leftArmVelocity = leftIntakeDeploy.getVelocity();
    rightArmVelocity = rightIntakeDeploy.getVelocity();
    leftArmEncoderPosition = leftIntakeDeploy.getPosition();
    rightArmEncoderPosition = rightIntakeDeploy.getPosition();
    
    intakeTimer.start();
  }

  public void init() {
    // Only set initial state once
    if (currMode != Mode.HOME) {
      currMode = Mode.HOME;
      desiredLeftArmPosition = 0.0;
      desiredLeftRollerVelocity = 0.0;
      desiredRightArmPosition = 0.0;
      desiredRightRollerVelocity = 0.0;
      isHomed = false;
    }
    intakeTimer.restart(); // restart, not reset
  }

  public void periodic() {
    // Refresh status signals once per cycle
    leftArmVelocity.refresh();
    rightArmVelocity.refresh();
    leftArmEncoderPosition.refresh();
    rightArmEncoderPosition.refresh();
    
    switch (currMode) {
      case HOME:
        leftIntakeDeploy.setControl(leftArmVoltageRequest.withOutput(-2.0));
        rightIntakeDeploy.setControl(rightArmVoltageRequest.withOutput(-2.0));
        leftIntake.setControl(leftRollerVelocityRequest.withVelocity(0.0));
        rightIntake.setControl(rightRollerVelocityRequest.withVelocity(0.0));
        if (Math.abs(leftArmVelocity.getValueAsDouble()) > 0.05) {
          intakeTimer.restart();
        }
        if (Math.abs(rightArmVelocity.getValueAsDouble()) > 0.05) {
          intakeTimer.restart();
        }

        if (intakeTimer.hasElapsed(1.0)) {
          leftIntakeDeploy.setPosition(0.0, 0.03);
          rightIntakeDeploy.setPosition(0.0, 0.03);
          isHomed = true;
          currMode = Mode.STOW;
          intakeTimer.restart();
        }
        break;

      case LEFT:
        if (rightArmSensor.getDistance().getValueAsDouble() < 0.67) { 
          // Right arm is up - deploy left arm
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = 2.0;
          desiredLeftRollerVelocity = 10.0;
          desiredRightRollerVelocity = 0.0;
        } else {
          // Right arm not up yet - bring it up, hold left position
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = leftArmEncoderPosition.getValueAsDouble();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0;
        }
        break;

      case RIGHT:
        // Sensor < 0.67 means arm is up (detected)
        if (leftArmSensor.getDistance().getValueAsDouble() < 0.67) {
          // Left arm is up - deploy right arm
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = 2.0;
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 10.0;
        } else {
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = rightArmEncoderPosition.getValueAsDouble();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0; 
        }
        break;
        
      case STOW:
        // Use motor position, not sensor
        desiredLeftArmPosition = 0.0;
        desiredRightArmPosition = 0.0;
        desiredLeftRollerVelocity = 0.0;
        desiredRightRollerVelocity = 0.0;
        break;
    }
    
    // Apply desired positions/velocities (except in HOME mode which uses voltage)
    if (currMode != Mode.HOME) {
      leftIntakeDeploy.setControl(leftArmPositionRequest.withPosition(desiredLeftArmPosition));
      leftIntake.setControl(leftRollerVelocityRequest.withVelocity(desiredLeftRollerVelocity));
      rightIntakeDeploy.setControl(rightArmPositionRequest.withPosition(desiredRightArmPosition));
      rightIntake.setControl(rightRollerVelocityRequest.withVelocity(desiredRightRollerVelocity));
    }
  }

  public void leftIntake() {
    if (isHomed) {
      currMode = Mode.LEFT;
      intakeTimer.restart();
    }
  }

  public void rightIntake() {
    if (isHomed) {
      currMode = Mode.RIGHT;
      intakeTimer.restart();
    }
  }

  public void stowIntake() {
    if (isHomed) {
      currMode = Mode.STOW;
      intakeTimer.restart();
    }
  }

  public Mode getMode() {
    return currMode;
  }

  public double getLeftArmSensorPosition() {
    return leftArmSensor.getDistance().getValueAsDouble();
  }

  public double getLeftArmEncoderPosition() {
    return leftArmEncoderPosition.getValueAsDouble();
  }

  public double getLeftArmDesiredPosition() {
    return desiredLeftArmPosition;
  }

  public double getLeftRollerVelocity() {
    return leftIntake.getVelocity().getValueAsDouble();
  }

  public double getLeftRollerDesiredVelocity() {
    return desiredLeftRollerVelocity;
  }

  public boolean leftArmInPosition() {
    return Math.abs(getLeftArmEncoderPosition() - desiredLeftArmPosition) < 0.1;
  }

  public boolean leftRollerAtSpeed() {
    return Math.abs(getLeftRollerVelocity() - desiredLeftRollerVelocity) < 1.0;
  }

  public double getRightArmEncoderPosition() {
    return rightArmEncoderPosition.getValueAsDouble();
  }

  public double getRightArmDesiredPosition() {
    return desiredRightArmPosition;
  }

  public double getRightRollerVelocity() {
    return rightIntake.getVelocity().getValueAsDouble();
  }

  public double getRightRollerDesiredVelocity() {
    return desiredRightRollerVelocity;
  }
  
  public boolean rightArmInPosition() {
    return Math.abs(getRightArmEncoderPosition() - desiredRightArmPosition) < 0.1;
  }

  public boolean rightRollerAtSpeed() {
    return Math.abs(getRightRollerVelocity() - desiredRightRollerVelocity) < 1.0;
  }

  public boolean isReady() {
    return leftArmInPosition() && leftRollerAtSpeed() && 
           rightArmInPosition() && rightRollerAtSpeed();
  }

  public boolean getIsHomed() {
    return isHomed;
  }

  public void updateDash() {
    SmartDashboard.putString("Intake Mode", currMode.toString());
    SmartDashboard.putBoolean("Intake Ready", isReady());
    SmartDashboard.putBoolean("Intake Homed", isHomed);
    SmartDashboard.putNumber("Intake Timer", intakeTimer.get());
    SmartDashboard.putNumber("Left Arm Encoder", getLeftArmEncoderPosition());
    SmartDashboard.putNumber("Left Arm Desired", getLeftArmDesiredPosition());
    SmartDashboard.putNumber("Left Roller Vel", getLeftRollerVelocity());
    SmartDashboard.putNumber("Left Sensor", getLeftArmSensorPosition());
    SmartDashboard.putNumber("Right Arm Encoder", getRightArmEncoderPosition());
    SmartDashboard.putNumber("Right Arm Desired", getRightArmDesiredPosition());
    SmartDashboard.putNumber("Right Roller Vel", getRightRollerVelocity());
    SmartDashboard.putNumber("Right Sensor", rightArmSensor.getDistance().getValueAsDouble());
  }
  
  private void configMotor(TalonFX motor, boolean invert, double currentLimit, boolean isArmMotor) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
    
    if (isArmMotor) {
      motorConfigs.Slot0.kP = 2.0;
      motorConfigs.Slot0.kI = 0.5;
      motorConfigs.Slot0.kD = 0.0;
    } else {
      motorConfigs.Slot0.kP = 0.2;
      motorConfigs.Slot0.kI = 0.1;
      motorConfigs.Slot0.kD = 0.0;
      motorConfigs.Slot0.kV = 0.12;
      motorConfigs.Slot0.kS = 0.1;
    }
    
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  private void configCANrange(CANrange sensor) {
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();
    sensorConfigs.ProximityParams.ProximityThreshold = 0.4;
    sensor.getConfigurator().apply(sensorConfigs, 0.03);
  }
}
