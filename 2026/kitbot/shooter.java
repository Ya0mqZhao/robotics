package frc.robot;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Shooter {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hoodMotor = new TalonFX(9, canivore);  
  private final TalonFX shootMotorRight = new TalonFX(12, canivore);  
  private final TalonFX shootMotorLeft = new TalonFX(11, canivore); 
  private final CANcoder hoodEncoder = new CANcoder(28, canivore); 
  private final StatusSignal<AngularVelocity> shooterVelocityRight;
  private final StatusSignal<AngularVelocity> shooterVelocityLeft;
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Voltage> shooterVoltageRight;
  private final VelocityVoltage shooterMotorRightVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final Follower shooterMotorLeftFollowerRequest = new Follower(12, MotorAlignmentValue.Opposed);
  private final MotionMagicTorqueCurrentFOC hoodMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); 
  private final double rpmTol = 200.0; // Can adjust
  private final double hoodTol = 0.005; // Can adjust
  public final double hoodMinPosition = 0.020; // Can adjust
  public final double hoodMaxPosition = 0.115; // Can adjust
  private double shootingRPM = 3000.0; // Can adjust
  private double desiredHoodPosition = hoodMinPosition;

  // Simulation
  private final TalonFXSimState hoodMotorSim = hoodMotor.getSimState();
  private final TalonFXSimState shootMotorRightSim = shootMotorRight.getSimState();
  private final TalonFXSimState shootMotorLeftSim = shootMotorLeft.getSimState();
  private final CANcoderSimState hoodEncoderSim = hoodEncoder.getSimState();
  private double desiredRPMSim = 0;
  public static final double hoodGearRatio = 1;

  // Shoot-on-the-fly data structures that will reference Robot's arrays
  private double[] distanceRPMArray = {2.0, 6.0}; // Default values, will be updated from Robot
  private double[] RPMArray = {2500.0, 3800.0}; // Default values, will be updated from Robot
  private double[] distanceArray = {2.0, 3.0, 4.0, 5.0, 6.0}; // Default values, will be updated from Robot
  private double[] hoodArray = {0.05, 0.0625, 0.07, 0.065, 0.06}; // Default values, will be updated from Robot
  
  // Precomputed LUTs for shoot-on-the-fly
  private final NavigableMap<Double, Double> RPM_VELOCITY_TO_DISTANCE = new TreeMap<>();
  private final NavigableMap<Double, Double> HOOD_VELOCITY_TO_DISTANCE = new TreeMap<>();
  private boolean lutInitialized = false;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configHoodEncoder(hoodEncoder);
    configShootMotor(shootMotorRight, true);
    configShootMotor(shootMotorLeft, false); // Configures the motor with counterclockwise rotation positive.
    configHoodMotor(hoodMotor, false);
    shooterVelocityRight = shootMotorRight.getVelocity();
    shooterVelocityLeft = shootMotorLeft.getVelocity();
    hoodPosition = hoodEncoder.getAbsolutePosition();
    shooterVoltageRight = shootMotorRight.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocityRight, shooterVelocityLeft, hoodPosition, shooterVoltageRight);
	  ParentDevice.optimizeBusUtilizationForAll(shootMotorLeft, hoodMotor, hoodEncoder);
  }
  
  // Method to update tuning data from Robot class
  public void updateTuningData(double[] distanceRPM, double[] rpm, double[] distance, double[] hood) {
    this.distanceRPMArray = distanceRPM;
    this.RPMArray = rpm;
    this.distanceArray = distance;
    this.hoodArray = hood;
    
    // Rebuild velocity LUTs with new data
    buildVelocityLUTs();
  }

  // Build velocity-to-effective-distance LUTs for shoot-on-the-fly
  private void buildVelocityLUTs() {
    RPM_VELOCITY_TO_DISTANCE.clear();
    HOOD_VELOCITY_TO_DISTANCE.clear();
    
    // Build RPM LUT: velocity = distance / timeOfFlight, but since we don't have timeOfFlight in arrays,
    // we'll use a simplified approach: velocity is proportional to distance * RPM factor
    // This is an approximation - you may need to adjust based on your actual shooter characteristics
    for (int i = 0; i < distanceRPMArray.length; i++) {
      double dist = distanceRPMArray[i];
      double rpm = RPMArray[i];
      // Approximate velocity as proportional to RPM (higher RPM = higher muzzle velocity)
      // You may need to calibrate this factor (0.1) based on real data
      double velocity = rpm * 0.1; 
      RPM_VELOCITY_TO_DISTANCE.put(velocity, dist);
    }
    
    // Build Hood LUT
    for (int i = 0; i < distanceArray.length; i++) {
      double dist = distanceArray[i];
      double hoodPos = hoodArray[i];
      // Approximate velocity based on hood position (different mapping than RPM)
      double velocity = hoodPos * 100.0; // Scale factor - calibrate this!
      HOOD_VELOCITY_TO_DISTANCE.put(velocity, dist);
    }
    
    lutInitialized = true;
  }

  public ShooterResult calculateShootOnTheFly(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation,
      boolean useRPM) {

    if (!lutInitialized) {
      buildVelocityLUTs();
    }

    // 1) Future position (optional)
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

    // 2) Vector to goal in FIELD frame
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();

    // Guard
    if (distance < 1e-6) {
      return new ShooterResult(new Rotation2d(), 0.0, 0.0);
    }

    // 3) Get baseline params from appropriate arrays
    double baselineValue;
    double minDistance, maxDistance;
    double[] distArray, valueArray;
    
    if (useRPM) {
      distArray = distanceRPMArray;
      valueArray = RPMArray;
    } else {
      distArray = distanceArray;
      valueArray = hoodArray;
    }
    
    minDistance = distArray[0];
    maxDistance = distArray[distArray.length - 1];
    
    // Clamp distance and interpolate baseline value
    double clampedDist = MathUtil.clamp(distance, minDistance, maxDistance);
    baselineValue = interpolateValue(clampedDist, distArray, valueArray);

    // 4) Direction to goal
    Translation2d targetDirection = toGoal.div(distance);

    // 5) Calculate radial velocity component (toward/away from target)
    double vRadial = robotVelocity.getX() * targetDirection.getX()
                   + robotVelocity.getY() * targetDirection.getY();

    // 6) Required velocity along line-to-goal (baseline - radial component)
    double requiredVelocity1D = baselineValue - vRadial;

    // 7) Limit how much the velocity can change to avoid spikes
    double maxExtraMps = 3.5; 
    requiredVelocity1D = MathUtil.clamp(
        requiredVelocity1D,
        baselineValue - maxExtraMps,
        baselineValue + maxExtraMps
    );

    // 8) Convert required velocity to effective distance, then to required value
    double effectiveDistance;
    if (useRPM) {
      effectiveDistance = velocityToEffectiveDistance(requiredVelocity1D, RPM_VELOCITY_TO_DISTANCE);
    } else {
      effectiveDistance = velocityToEffectiveDistance(requiredVelocity1D, HOOD_VELOCITY_TO_DISTANCE);
    }
    
    double clampedEff = MathUtil.clamp(effectiveDistance, minDistance, maxDistance);
    double requiredValue = interpolateValue(clampedEff, distArray, valueArray);

    // 9) Calculate turret angle (same for both cases)
    // Build desired target velocity vector toward goal
    Translation2d targetVelocity = targetDirection.times(baselineValue);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    Rotation2d turretFieldAngle = shotVelocity.getAngle();

    return new ShooterResult(turretFieldAngle, requiredValue, effectiveDistance);
  }

  /**
   * Linear interpolation helper
   */
  private double interpolateValue(double distance, double[] distArray, double[] valueArray) {
    if (distance <= distArray[0]) return valueArray[0];
    if (distance >= distArray[distArray.length - 1]) return valueArray[valueArray.length - 1];
    
    for (int i = 0; i < distArray.length - 1; i++) {
      if (distance >= distArray[i] && distance <= distArray[i + 1]) {
        double t = (distance - distArray[i]) / (distArray[i + 1] - distArray[i]);
        return valueArray[i] + t * (valueArray[i + 1] - valueArray[i]);
      }
    }
    return valueArray[0];
  }

  /**
   * Converts required velocity to effective distance using velocity LUT
   */
  private double velocityToEffectiveDistance(double velocity, NavigableMap<Double, Double> velocityLUT) {
    Map.Entry<Double, Double> floor = velocityLUT.floorEntry(velocity);
    Map.Entry<Double, Double> ceil = velocityLUT.ceilingEntry(velocity);

    if (floor == null) {
      return velocityLUT.firstEntry().getValue();
    }
    if (ceil == null) {
      return velocityLUT.lastEntry().getValue();
    }
    if (ceil.getKey().equals(floor.getKey())) {
      return ceil.getValue();
    }

    // Interpolate distance between the two surrounding velocity keys
    double v0 = floor.getKey();
    double d0 = floor.getValue();
    double v1 = ceil.getKey();
    double d1 = ceil.getValue();

    double t = (velocity - v0) / (v1 - v0);
    return MathUtil.interpolate(d0, d1, t);
  }

  // Convenience methods for RPM-based compensation
  public ShooterResult calculateShootOnTheFlyRPM(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {
    return calculateShootOnTheFly(robotPosition, robotVelocity, goalPosition, latencyCompensation, true);
  }

  // Convenience methods for hood-based compensation
  public ShooterResult calculateShootOnTheFlyHood(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {
    return calculateShootOnTheFly(robotPosition, robotVelocity, goalPosition, latencyCompensation, false);
  }

  // Turns on motor. Sets the speed of the motor in rotations per minute.
  public void spinUp() {
    desiredRPMSim = shootingRPM;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(shootingRPM/60.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftFollowerRequest);
  }

  // Turn off motor.
  public void spinDown() {
    desiredRPMSim = 0;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(0.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftFollowerRequest);
  }

  public void setShootingRPM(double rpm) {
    if (rpm > 5800.0) {
      shootingRPM = 5800.0;
    } else if (rpm < 600.0) {
      shootingRPM = 600.0;
    } else {
      shootingRPM = rpm;
    }
  }

  public void setHoodPosition(double position) {
    if (position < hoodMinPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
      desiredHoodPosition = hoodMinPosition;
    } else if (position > hoodMaxPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMaxPosition));
      desiredHoodPosition = hoodMaxPosition;
    } else {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(position));
      desiredHoodPosition = position;
    }
  }

  public void lowerHood() {
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
    desiredHoodPosition = hoodMinPosition;
  }

  public boolean hoodIsInPosition() {
    // TODO: Fix simulationPeriodic
    if (Robot.isSimulation()) return true;

    return Math.abs(desiredHoodPosition - getHoodPosition()) < hoodTol;
  }

  public double getHoodPosition() {
    return hoodPosition.refresh().getValueAsDouble();
  }

  // Returns true or false based on whether the shooter motor is near the desired RPM.
  public boolean shooterIsAtSpeed() {
    // use abs() on the left/right shooter to compare magnitudes 
    // otherwise you may have (2800 - -2800) = 5600 rpm which shows as not ready
    return Math.abs(shootingRPM - Math.abs(getLeftShooterRPM())) < rpmTol && Math.abs(shootingRPM - Math.abs(getRightShooterRPM())) < rpmTol;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getLeftShooterRPM() {
    return shooterVelocityLeft.refresh().getValueAsDouble()*60.0;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getRightShooterRPM() {
    return shooterVelocityRight.refresh().getValueAsDouble()*60.0;
  }

  public boolean isReady() {
    return shooterIsAtSpeed() && hoodIsInPosition();
  }

  // Publish Shooter information (Motor state, Velocity) to SmartDashboard.
  public void updateDash() {
    // SmartDashboard.putNumber("Shooter getRightShooterRPM", getRightShooterRPM());
    // SmartDashboard.putNumber("Shooter getLeftShooterRPM", getLeftShooterRPM());
    // SmartDashboard.putBoolean("Shooter shooterIsAtSpeed", shooterIsAtSpeed());
    // SmartDashboard.putNumber("Shooter shootingRPM", shootingRPM);
    // SmartDashboard.putNumber("Shooter getHoodPosition", getHoodPosition());
    // SmartDashboard.putBoolean("Shooter hoodIsInPosition", hoodIsInPosition());
    // SmartDashboard.putNumber("Shooter desiredHoodPosition", desiredHoodPosition);
    // SmartDashboard.putBoolean("Shooter isReady", isReady());
  }

  public void simulationPeriodic() {
    // Very basic - just jump to the desired values
    // Update the motors
    shootMotorRightSim.setRotorVelocity(desiredRPMSim / 60.0);
    shootMotorLeftSim.setRotorVelocity(desiredRPMSim / 60.0);

    // Update the hood position.
    // TODO:
    hoodMotorSim.setRawRotorPosition(desiredHoodPosition/hoodGearRatio);
    hoodEncoderSim.setRawPosition(desiredHoodPosition/hoodGearRatio);
  }

  private void configHoodEncoder(CANcoder CANsensor) {
    CANcoderConfiguration sensorConfigs = new CANcoderConfiguration();

    sensorConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; 
    sensorConfigs.MagnetSensor.MagnetOffset = 0.378662109375;
    sensorConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    CANsensor.getConfigurator().apply(sensorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configShootMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


    // P = 0.25 , I = 0.5, D = 0.0, V = 0.12, S = 0.16


    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.02; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configHoodMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorConfigs.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = 211.68;

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*211.68); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*211.68); // Units: roations per second.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Record class for shooter results
  public static record ShooterResult(Rotation2d turretFieldAngle, double requiredValue, double effectiveDistance) {}
}
