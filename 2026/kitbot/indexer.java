package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
    public enum Mode {FORWARD, JAM, IDLE}
    private final TalonFX intakeMotor = new TalonFX(0); // Activates TalonFX on CAN ID 0
    private final Timer indexTimer = new Timer();
    private Mode currState = Mode.IDLE; // Stores the current mode of the launcher.
    
    // Configuration constants - adjust these for your robot
    private static final double FORWARD_SPEED = 25.0; // Rotations per second
    private static final double JAM_SPEED = -15.0; // Reverse speed for clearing jams
    private static final double SENSOR_THRESHOLD = 2.5; // Voltage threshold for sensor
    private final AnalogPotentiometer sensor = new AnalogPotentiometer(0, 5.0); // Analog port 0, 0-5V range
    
    public Indexer() {
        configMotor();
        indexTimer.start();
    }

    // Initialize indexer
    public void init() {
        currState = Mode.IDLE;
        intakeMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
        indexTimer.reset();
    }

    // Periodic update - call every robot cycle
    public void periodic() {
        // Check for jam when running forward (sensor triggered for too long)
        if (currState == Mode.FORWARD && getSensor() && indexTimer.hasElapsed(2.0)) {
            currState = Mode.JAM;
            indexTimer.reset();
        }
        
        // Handle jam recovery (run reverse for 0.5 seconds)
        if (currState == Mode.JAM && indexTimer.hasElapsed(0.5)) {
            currState = Mode.IDLE;
        }
        
        // Apply motor control based on current state
        switch(currState) {
            case FORWARD:
                intakeMotor.setControl(new VelocityVoltage(FORWARD_SPEED).withEnableFOC(true));
                break;
            case JAM:
                intakeMotor.setControl(new VelocityVoltage(JAM_SPEED).withEnableFOC(true));
                break;
            case IDLE:
            default:
                intakeMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
                break;
        }
    }

    // Starts the index motor
    public void start() {
        currState = Mode.FORWARD;
        indexTimer.reset();
    }

    // Stops the index motor
    public void stop() {
        currState = Mode.IDLE;
        intakeMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
    }

    // Return the timer for the sensor
    public double getSensorTimer() {
        return indexTimer.get();
    }

    // Return the sensor state (true/false)
    public boolean getSensor() {
        return sensor.get() > SENSOR_THRESHOLD;
    }

    // Updates/uploads the index info on dashboard
    public void updateDash() {
        SmartDashboard.putString("Indexer State", currState.toString());
        SmartDashboard.putBoolean("Indexer Sensor", getSensor());
        SmartDashboard.putNumber("Indexer Sensor Value", sensor.get());
        SmartDashboard.putNumber("Indexer Sensor Timer", getSensorTimer());
        SmartDashboard.putNumber("Indexer Velocity", getVelocity());
    }
    
    // Returns the velocity of the Index motor spinning
    private double getVelocity() {
        return intakeMotor.getVelocity().getValue();
    }

    // Configure the motor index PID and etc...
    private void configMotor() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limit configuration
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0;
        
        // VelocityVoltage closed-loop control configuration
        motorConfigs.Slot0.kP = 0.2;
        motorConfigs.Slot0.kI = 0.1;
        motorConfigs.Slot0.kD = 0.0;
        motorConfigs.Slot0.kV = 0.12;
        motorConfigs.Slot0.kS = 0.1;
        
        intakeMotor.getConfigurator().apply(motorConfigs, 0.03);
    }
}
