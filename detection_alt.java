// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();
  XRPGyro gyro = new XRPGyro();
  AnalogInput sensor = new AnalogInput(2);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
@Override
public void autonomousInit() {
    m_drivetrain.resetEncoders();
    gyro.reset();
    sensor.resetAccumulator();
    step = 0;
    obstacleStartTime = 0;
}

@Override
public void autonomousPeriodic() {
    double sensorValue = sensor.getAverageVoltage();
    sensor.getAverageValue(); 
    double Y = m_drivetrain.getLeftDistanceInch(); 
    double X = m_drivetrain.getRightDistanceInch(); 
    double gyroValue = gyro.getAngle(); 
    double average = (Y+X)/2; 
  
    System.out.println(gyroValue); 
    if (sensorValue > MAX_SENSOR) {
        m_drivetrain.arcadeDrive(0, 0);
        return;
    }
    if (step == 0) {
        if (sensorValue < OBSTACLE_THRESHOLD) {
            step = 1;
            obstacleStartTime = System.currentTimeMillis();
        } else {
            m_drivetrain.arcadeDrive(DRIVE_POWER, 0);
        }
    } else if (step == 1) { 
        if (sensorValue > CLEAR_THRESHOLD ||
            System.currentTimeMillis() - obstacleStartTime > MAX_OBSTACLE_TIME) {
            step = 0;
        } else {
            m_drivetrain.arcadeDrive(0, TURN_POWER);
        }
    }
}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
