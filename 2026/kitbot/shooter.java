package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private final TalonFX shootMotor = new TalonFX(10, "canivore");
    private boolean isSpinUp = false;
    
    public Shooter() {
        configMotor(shootMotor, false, 80.0);
    }

    public void spinUp() {
        shootMotor.setControl(new VelocityVoltage(80.0).withEnableFOC(true));
        isSpinUp = true;
    }
    public void spinDown() {
        shootMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
        isSpinUp = false;
    }
    public void updateDash() {
        SmartDashboard.putBoolean("Shoot motor is on", isSpinUp);
    }
    
    private void configMotor(TalonFX motor, boolean invert, double currentLimit) {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
        
        motorConfigs.Slot0.kP = 0.25;
        motorConfigs.Slot0.kI = 0.5;
        motorConfigs.Slot0.kD = 0.0;
        motorConfigs.Slot0.kV = 0.12;
        motorConfigs.Slot0.kS = 0.16;
        
        motor.getConfigurator().apply(motorConfigs, 0.03);
    }
}
