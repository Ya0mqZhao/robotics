package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private final Timer intakeTimer = new Timer();
  private boolean isHomed = false;
  private boolean rightArmConfirmedUp = false;
  private boolean leftArmConfirmedUp = false; 
  
  // Control requests
  private final PositionVoltage armPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut voltageOutRequest = new VoltageOut(0.0);

  public Intake() {
    configMotor(rightIntakeDeploy, true, 40.0, true);
    configMotor(rightIntake, false, 60.0, false);
    configMotor(leftIntakeDeploy, false, 40.0, true);
    configMotor(leftIntake, true, 60.0, false);
    intakeTimer.start();
  }

  public void init() {
    currMode = Mode.HOME;
    desiredLeftArmPosition = 0.0;
    desiredLeftRollerVelocity = 0.0;
    desiredRightArmPosition = 0.0;
    desiredRightRollerVelocity = 0.0;
    isHomed = false;
    rightArmConfirmedUp = false;
    leftArmConfirmedUp = false;
    intakeTimer.reset();
  }

  public void periodic() {
    switch (currMode) {
      case HOME:
        if (!isHomed) {
          leftIntakeDeploy.setControl(voltageOutRequest.withOutput(-2.0));
          rightIntakeDeploy.setControl(voltageOutRequest.withOutput(-2.0));
          leftIntake.setControl(rollerVelocityRequest.withVelocity(0.0));
          rightIntake.setControl(rollerVelocityRequest.withVelocity(0.0));
          double leftVel = Math.abs(leftIntakeDeploy.getVelocity().getValueAsDouble());
          double rightVel = Math.abs(rightIntakeDeploy.getVelocity().getValueAsDouble());
          boolean leftStopped = leftVel < 0.05;
          boolean rightStopped = rightVel < 0.05;
          
          if (leftStopped && rightStopped) {
            if (intakeTimer.hasElapsed(1.0)) {
              leftIntakeDeploy.setPosition(0.0);
              rightIntakeDeploy.setPosition(0.0);
              isHomed = true;
              currMode = Mode.STOW;
              rightArmConfirmedUp = false;
              leftArmConfirmedUp = false;
            }
          } else {
            intakeTimer.reset();
          }
        }
        break;
      case LEFT:
        if (!rightArmConfirmedUp) {
          desiredRightArmPosition = 0.0;
          desiredLeftArmPosition = getLeftArmPosition();
          if (Math.abs(getRightArmPosition() - 0.0) < 0.1) {
            rightArmConfirmedUp = true;
            intakeTimer.reset();
          }
        } else {
          if (intakeTimer.hasElapsed(0.1)) {
            desiredRightArmPosition = 0.0;
            desiredLeftArmPosition = 2.0;
          }
        }
        desiredLeftRollerVelocity = 10.0;
        desiredRightRollerVelocity = 0.0; 
        break;
        
      case RIGHT:
        if (!leftArmConfirmedUp) {
          desiredLeftArmPosition = 0.0;
          desiredRightArmPosition = getRightArmPosition(); 
          if (Math.abs(getLeftArmPosition() - 0.0) < 0.1) {
            leftArmConfirmedUp = true;
            intakeTimer.reset(); 
          }
        } else {
          if (intakeTimer.hasElapsed(0.1)) {
            desiredLeftArmPosition = 0.0;
            desiredRightArmPosition = 2.0;
          }
        }
        desiredLeftRollerVelocity = 0.0;
        desiredRightRollerVelocity = 10.0;
        break;
        
      case STOW:
        desiredLeftArmPosition = 0.0;
        desiredRightArmPosition = 0.0;
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
      rightArmConfirmedUp = false;
      leftArmConfirmedUp = false;
      intakeTimer.reset();
    }
  }

  public void rightIntake() {
    if (isHomed) {
      currMode = Mode.RIGHT;
      rightArmConfirmedUp = false;
      leftArmConfirmedUp = false;
      intakeTimer.reset();
    }
  }

  public void stowIntake() {
    if (isHomed) {
      currMode = Mode.STOW;
      rightArmConfirmedUp = false;
      leftArmConfirmedUp = false;
      intakeTimer.reset();
    }
  }

  public Mode getMode() {
    return currMode;
  }

  public double getLeftArmPosition() {
    return leftIntakeDeploy.getPosition().getValueAsDouble();
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
    SmartDashboard.putBoolean("Right Arm Confirmed Up", rightArmConfirmedUp);
    SmartDashboard.putBoolean("Left Arm Confirmed Up", leftArmConfirmedUp);
    SmartDashboard.putNumber("Intake Timer", intakeTimer.get());
    SmartDashboard.putNumber("Left Arm Position", getLeftArmPosition());
    SmartDashboard.putNumber("Left Arm Desired", getLeftArmDesiredPosition());
    SmartDashboard.putNumber("Left Roller Vel", getLeftRollerVelocity());
    SmartDashboard.putNumber("Right Arm Position", getRightArmPosition());
    SmartDashboard.putNumber("Right Arm Desired", getRightArmDesiredPosition());
    SmartDashboard.putNumber("Right Roller Vel", getRightRollerVelocity());
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
}
