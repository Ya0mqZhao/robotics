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
    
    private final TalonFX motor1 = new TalonFX(0); // Activates TalonFX on CAN ID 0
    private final Timer indexTimer = new Timer();
    private Mode currState = Mode.IDLE; // Stores the current mode of the indexer
    
    // Configuration constants (adjust these based on your robot)
    private static final double FORWARD_SPEED = 20.0; // Rotations per second
    private static final double JAM_SPEED = -10.0; // Reverse speed for clearing jams
    private static final double SENSOR_TIMEOUT = 2.0; // Seconds before considering jammed
    
    // If you have a sensor for detecting notes (e.g., beam break, IR sensor)
    private boolean noteSensorTriggered = false; // Simulated sensor state
    private double lastSensorActivationTime = 0.0;
    
    public Indexer() {
        configMotor();
        indexTimer.start();
    }

    // Initialize indexer
    public void init() {
        currState = Mode.IDLE;
        stop();
        indexTimer.reset();
        lastSensorActivationTime = Timer.getFPGATimestamp();
    }

    // Periodic update - should be called every robot loop
    public void periodic() {
        // Update sensor state (you'll need to implement actual sensor reading)
        // For now, this is simulated
        updateSensorState();
        
        // State machine logic
        switch (currState) {
            case FORWARD:
                // Check for jam (sensor not triggered within timeout)
                if (Timer.getFPGATimestamp() - lastSensorActivationTime > SENSOR_TIMEOUT) {
                    currState = Mode.JAM;
                    System.out.println("Indexer: Jam detected, switching to JAM mode");
                }
                break;
                
            case JAM:
                // Run reverse for a short time, then return to IDLE
                if (indexTimer.hasElapsed(0.5)) { // Clear jam for 0.5 seconds
                    currState = Mode.IDLE;
                    stop();
                    indexTimer.reset();
                    System.out.println("Indexer: Jam cleared, returning to IDLE");
                }
                break;
                
            case IDLE:
            default:
                // Do nothing in idle state
                break;
        }
        
        // Apply motor control based on current state
        applyMotorControl();
    }

    // Starts the index motor in forward direction
    public void start() {
        if (currState != Mode.FORWARD) {
            currState = Mode.FORWARD;
            indexTimer.reset();
            lastSensorActivationTime = Timer.getFPGATimestamp();
            System.out.println("Indexer: Starting forward");
        }
    }

    // Stops the index motor
    public void stop() {
        currState = Mode.IDLE;
        motor1.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
        System.out.println("Indexer: Stopped");
    }

    // Manually trigger jam recovery
    public void clearJam() {
        currState = Mode.JAM;
        indexTimer.reset();
        System.out.println("Indexer: Manual jam clear initiated");
    }

    // Return the timer for the sensor (time since last sensor activation)
    public double getSensorTimer() {
        return Timer.getFPGATimestamp() - lastSensorActivationTime;
    }

    // Return the sensor state (true = note detected)
    public boolean getSensor() {
        return noteSensorTriggered;
    }

    // Simulated sensor update - replace with actual sensor code
    private void updateSensorState() {
        // TODO: Replace with actual sensor reading
        // For example, if using an analog sensor:
        // noteSensorTriggered = (sensor.get() > 0.5);
        
        // For now, simulate random sensor triggers for testing
        if (currState == Mode.FORWARD && Math.random() < 0.05) {
            noteSensorTriggered = true;
            lastSensorActivationTime = Timer.getFPGATimestamp();
        } else {
            noteSensorTriggered = false;
        }
    }

    // Updates/uploads the index info on dashboard
    public void updateDash() {
        SmartDashboard.putString("Indexer State", currState.toString());
        SmartDashboard.putBoolean("Indexer Sensor", getSensor());
        SmartDashboard.putNumber("Indexer Sensor Timer", getSensorTimer());
        SmartDashboard.putNumber("Indexer Velocity", getVelocity());
        SmartDashboard.putNumber("Indexer Current", motor1.getStatorCurrent().getValue());
    }

    // Returns the velocity of the Index motor spinning
    private double getVelocity() {
        return motor1.getVelocity().getValue();
    }

    // Apply motor control based on current state
    private void applyMotorControl() {
        switch (currState) {
            case FORWARD:
                motor1.setControl(new VelocityVoltage(FORWARD_SPEED).withEnableFOC(true));
                break;
            case JAM:
                motor1.setControl(new VelocityVoltage(JAM_SPEED).withEnableFOC(true));
                break;
            case IDLE:
            default:
                motor1.setControl(new VelocityVoltage(0.0).withEnableFOC(true));
                break;
        }
    }

    // Configure the motor index PID and etc...
    private void configMotor() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        
        // Motor output configuration
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast is often better for indexing
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Adjust as needed
        
        // Current limit configuration
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 40.0; // Lower current limit for indexing
        
        // Velocity closed-loop control configuration
        motorConfigs.Slot0.kP = 0.15; // Lower P gain for smoother indexing
        motorConfigs.Slot0.kI = 0.1;
        motorConfigs.Slot0.kD = 0.0;
        motorConfigs.Slot0.kV = 0.12;
        motorConfigs.Slot0.kS = 0.1; // Lower static friction compensation
        
        // Apply configuration
        motor1.getConfigurator().apply(motorConfigs, 0.03);
        
        System.out.println("Indexer motor configured");
    }
    
    // Getter for current state
    public Mode getCurrentState() {
        return currState;
    }
    
    // Check if indexer is running
    public boolean isRunning() {
        return currState == Mode.FORWARD;
    }
    
    // Check if indexer is jammed
    public boolean isJammed() {
        return currState == Mode.JAM;
    }
}
