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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final CANrange leftArmSensor = new CANrange(29, canivore); // Code Review: Not CANrange, MaxSplineThroughbore Encoder
  private final CANrange rightArmSensor = new CANrange(30, canivore); // Code Review: Not CANrange, MaxSplineThroughbore Encoder 
  private final Timer intakeTimer = new Timer();
  private boolean isHomed = false;
  // Control requests
  private final PositionVoltage armPositionRequest = new PositionVoltage(0.0).withEnableFOC(true); // Code Review: Put more requests per motor stated 
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true); // Code Review: Put more requests per motor stated 
  private final VoltageOut voltageOutRequest = new VoltageOut(0.0); // Code Review: Put more requests per motor stated 

  public Intake() {
    configMotor(rightIntakeDeploy, true, 40.0, true);
    configMotor(rightIntake, false, 60.0, false);
    configMotor(leftIntakeDeploy, false, 40.0, true);
    configMotor(leftIntake, true, 60.0, false);
    configCANrange(leftArmSensor);
    configCANrange(rightArmSensor);
    intakeTimer.start();
  }

  public void init() {
    currMode = Mode.HOME; // Code Review: there are two moments of this starting in home state?
    desiredLeftArmPosition = 0.0; // Code Review: there are two moments of this
    desiredLeftRollerVelocity = 0.0; // Code Review: there are two moments of this
    desiredRightArmPosition = 0.0; // Code Review: there are two moments of this
    desiredRightRollerVelocity = 0.0; // Code Review: there are two moments of this
    isHomed = false; // Code Review: there are two moments of this
    intakeTimer.reset(); // Code Review: not reset but restart, since we want to start counting time from the moment we enter the homing state
  }

  public void periodic() {
    switch (currMode) {
      case HOME:
        leftIntakeDeploy.setControl(voltageOutRequest.withOutput(-2.0));
        rightIntakeDeploy.setControl(voltageOutRequest.withOutput(-2.0));
        leftIntake.setControl(rollerVelocityRequest.withVelocity(0.0));
        rightIntake.setControl(rollerVelocityRequest.withVelocity(0.0));
        
        double leftVel = Math.abs(leftIntakeDeploy.getVelocity().getValueAsDouble());
        double rightVel = Math.abs(rightIntakeDeploy.getVelocity().getValueAsDouble()); // Code Review: too meny object created, forces garbige collection, cause robot to lag. suggestion: Make a (satis) signal in the start.
        
        if (leftVel > 0.05 || rightVel > 0.05) { // Code Review: Puting left and right together just put separately
          intakeTimer.restart();
        }

        if (intakeTimer.hasElapsed(1.0)) {
          leftIntakeDeploy.setPosition(0.0, 0.03);
          rightIntakeDeploy.setPosition(0.0, 0.03);
          isHomed = true;
          currMode = Mode.STOW;
        }
        break;

      case LEFT:
        if (rightArmSensor.getDistance().getValueAsDouble() < 0.67) { 
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = 2.0;
          desiredLeftRollerVelocity = 10.0;
          desiredRightRollerVelocity = 0.0;
        } else {
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = getLeftArmPosition();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0;
          intakeTimer.restart(); // Code review: WHAT POINT
        }
        break;

      case RIGHT:
        if (leftArmSensor.getDistance().getValueAsDouble() < 0.67) {//check real value
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = 2.0;
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 10.0;
        } else {
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = getRightArmPosition();
          desiredLeftRollerVelocity = 0.0;
          desiredRightRollerVelocity = 0.0; 
          intakeTimer.restart(); // Code review: WHAT POINT
        }
        break;
        
      case STOW:
        double currentLeftSensor = leftArmSensor.getDistance().getValueAsDouble();
        double currentRightSensor = rightArmSensor.getDistance().getValueAsDouble(); // Code Review: Dont use sensor, use moter postion.

        if (currentLeftSensor > 0.5) { // Code Review: Dont use this, go to zero position directly. From line 115-135
          desiredLeftArmPosition = 0.0;
        } else {
          desiredLeftArmPosition = 0.0;
        }
        if (currentRightSensor > 0.5) {
          desiredRightArmPosition = 0.0;
        } else {
          desiredRightArmPosition = 0.0;
        }
        desiredLeftRollerVelocity = 0.0;
        desiredRightRollerVelocity = 0.0;
        break;
    }
    
    if (currMode != Mode.HOME) {
      leftIntakeDeploy.setControl(armPositionRequest.withPosition(desiredLeftArmPosition));
      leftIntake.setControl(rollerVelocityRequest.withVelocity(desiredLeftRollerVelocity));
      rightIntakeDeploy.setControl(armPositionRequest.withPosition(desiredRightArmPosition));
      rightIntake.setControl(rollerVelocityRequest.withVelocity(desiredRightRollerVelocity));
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

  public double getLeftArmPosition() {
    return leftArmSensor.getDistance().getValueAsDouble();
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
    return Math.abs(getLeftArmPosition() - desiredLeftArmPosition) < 0.1;
  }

  public boolean leftRollerAtSpeed() {
    return Math.abs(getLeftRollerVelocity() - desiredLeftRollerVelocity) < 1.0;
  }

  public double getRightArmPosition() {
    return rightIntakeDeploy.getPosition().getValueAsDouble();
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
    return Math.abs(getRightArmPosition() - desiredRightArmPosition) < 0.1;
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
    SmartDashboard.putNumber("Left Arm Position", getLeftArmPosition());
    SmartDashboard.putNumber("Left Arm Desired", getLeftArmDesiredPosition());
    SmartDashboard.putNumber("Left Roller Vel", getLeftRollerVelocity());
    SmartDashboard.putNumber("Left Sensor", leftArmSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Right Arm Position", getRightArmPosition());
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

//add another system where if sensor reads range too far and arm intake both distance too far, try to set arm back to position 0 and stop everything.
