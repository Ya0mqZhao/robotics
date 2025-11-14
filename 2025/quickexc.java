package frc.robot;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AlgaeYeeter.ArmPosition;
import frc.robot.Elevator.Level;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  private double speedScaleFactor = 0.65; // Scales the translational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private double rotationScaleFactor = 0.3; // Scales the rotational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Elevator elevator = new Elevator(); // Contains the elevator motor and limit switches.
  private final CoralSpitter coralSpitter = new CoralSpitter(); // Contains the coral ejector motor and coral sensor. 
  private final Climber climber = new Climber(); // Contains the climber motor.
  private final AlgaeYeeter algaeYeeter = new AlgaeYeeter(); // Contains the algae sensor, algae yeeter arm motor, and algae yeeter intake motors.
  private final CANdle frontCandle = new CANdle(0, "canivore"); // Initializes the lights on the front of the robot.
  private final CANdle backCandle = new CANdle(1, "canivore"); // Initializes the lights on the back of the robot.
  private final RGBWColor offColor = new RGBWColor(0, 0, 0); // Represents LEDs that are off.
  private final RGBWColor redColor = new RGBWColor(255, 0, 0); // Represents red LEDs.
  private final RGBWColor greenColor = new RGBWColor(0, 255, 0); // Represents green LEDs.
  private final RGBWColor blueColor = new RGBWColor(0, 0, 255); // Represents blue LEDs.
  private RGBWColor currentColor = offColor; // Stores the current color of the CANdle.
  private SolidColor SolidColorRequest = new SolidColor(0, 39); // Used to control the CANdle lights.
  private boolean candleStrobeState = true; // Stores whether the candles are on or off. Used to produce a stobe effect.
  private int period = 0; // Keeps track of how many periods have elapsed since the begining of the program.
  
  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "WIP1"; 
  private static final String auto2 = "1-Piece Coral L4 (Index 3) Center (90 Start Heading)"; 
  private static final String auto3 = "3-Piece Coral L2/L4/L4 (Index 4, 5, 11) Left Side (180 Start Heading)";
  private static final String auto4 = "3-Piece Coral L2/L4/L4 (Index 2, 1, 7) Right Side (90 Start Heading)";
  private static final String auto5 = "3-Piece Coral L4/L4/L4 (Index 4, 5, 11) Left Side (180 Start Heading)";
  private static final String auto6 = "3-Piece Coral L4/L4/L4 (Index 2, 1, 7) Right Side (90 Start Heading)"; 
  private static final String auto7 = "algae1";
  private static final String auto8 = "WIP2"; 
  private static final String auto9 = "WIP3"; 
  private static final String auto10 = "WIP4";
  private String autoSelected;
  private int autoStage = 1;
  private boolean autoCompleted = false;
  private Timer coralTimer = new Timer();

  // Auto Aim Variables
  private final double reefX = 176.75*0.0254; // The x-coordinate of the center of the reef in meters.
  private final double reefY = Drivetrain.fieldWidth/2.0; // The y-coordinate of the center of the reef in meters.
  private double[] scoringPositionsX = new double[30]; // Contains all scoring positions of the robot in the x-direction.
  private double[] scoringPositionsY = new double[30]; // Contains all scoring positions of the robot in the y-direction.
  private double[] scoringHeadings = new double[30]; // Contains all scoring headings of the robot.
  private double[] scoreDistances = new double[30]; // Stores the distance to each scoring location from the currenly position of the robot. Updated when scoreCalc() is called.
  private int nearestScoreIndex = 0; // Array index corresponding to the closest scoring location to the current position of the robot. Updated when scoreCalc() is called.
  private enum scoreMode {Branch, L1, Algae};
  private scoreMode currScoreMode = scoreMode.Branch;

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    autoChooser.addOption(auto5, auto5);
    autoChooser.addOption(auto6, auto6);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto7, auto7);
    autoChooser.addOption(auto1, auto1);
    autoChooser.addOption(auto8, auto8);
    autoChooser.addOption(auto9, auto9);
    autoChooser.addOption(auto10, auto10);
    SmartDashboard.putData("Autos", autoChooser);

    coralTimer.restart();
    swerve.loadPath("Test", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    elevator.updateDash();
    coralSpitter.updateDash();
    algaeYeeter.updateDash();
    climber.updateDash();
    updateDash();

    // Causes the candleStrobeState variable to oscillate between true and false every 5 periods (0.1 seconds).
    period++;
    if (period % 2 == 0) candleStrobeState = !candleStrobeState;

    if (swerve.getAccurateCalibrationTimer() < 0.2 && candleStrobeState) { // Strobes the lights for 1 second after an accurate vision calibration is made.
      currentColor = offColor;
    } else if (currScoreMode == scoreMode.Branch) { // Blue color for Branch mode.
      currentColor = blueColor;
    } else if (currScoreMode == scoreMode.L1) { // Red color for L1 mode.
      currentColor = redColor;
    } else  if (currScoreMode == scoreMode.Algae) { // Green color for Algae Mode.
      currentColor = greenColor;
    } 
    
    frontCandle.setControl(SolidColorRequest.withColor(currentColor));
    backCandle.setControl(SolidColorRequest);
  }

  public void autonomousInit() {
    autoCompleted = true;
    coralSpitter.init(); // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
    algaeYeeter.init(); // Should be called in autoInit() and teleopInit(). Required for the algaeYeeter to function correctly.
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(scoringHeadings[3]); // Prepares the robot to drive to the reef.
      break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.pushCalibration(true, 180.0); // Updates the robot's position on the field.
        swerve.resetDriveController(scoringHeadings[4]); // Prepares the robot to drive to the reef.
      break;

      case auto4:
        // AutoInit 4 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(scoringHeadings[2]); // Prepares the robot to drive to the reef.
      break;

      case auto5:
        // AutoInit 5 code goes here.
        swerve.pushCalibration(true, 180.0); // Updates the robot's position on the field.
        swerve.resetDriveController(scoringHeadings[4]); // Prepares the robot to drive to the reef.
      break;

      case auto6:
        // AutoInit 6 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(scoringHeadings[2]); // Prepares the robot to drive to the reef.
      break;

      case auto7:
        // AutoInit 7 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetDriveController(90);//prepare robot
      break;

      case auto8:
        // AutoInit 8 code goes here.
      break;

      case auto9:
        // AutoInit 9 code goes here.
      break;

      case auto10:
        // AutoInit 10 code goes here.
      break;
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }
    coralSpitter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
    algaeYeeter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the algaeYeeter to function correctly.
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
          break;
        }
      break;

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[3],scoringPositionsY[3],scoringHeadings[3]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2; // Advances to the next stage once the robot is at the correct location.
            }
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 3; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 3:
            // Auto 2, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 4; 
            }
          break;

          case 4:
          swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
          elevator.setLevel(Level.bottom); // This moves the elevator to the second level.
        }
      break;

      case auto3:
        switch (autoStage) {
          case 1:
            // Auto 3, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[4],scoringPositionsY[4],scoringHeadings[4]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2; // Advances to the next stage once the robot is at the correct location.
            }
          break;

          case 2:
            // Auto 3, Stage 2 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L2); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 3; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 3:
            // Auto 3, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 4; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 4:
            // Auto 3, Stage 4 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.
            if (elevator.atSetpoint()) {
              swerve.resetDriveController(scoringHeadings[27]);
              coralTimer.restart();
              autoStage = 5; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 5:
            // Auto 3, Stage 5 code goes here.
            swerve.driveTo(scoringPositionsX[27],scoringPositionsY[27],scoringHeadings[27]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[5]);
              autoStage = 6; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 6:
            // Auto 3, Stage 6 code goes here.
            swerve.driveTo(scoringPositionsX[5],scoringPositionsY[5],scoringHeadings[5]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 7; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 7:
            // Auto 3, Stage 7 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 8; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 8:
            // Auto 3, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 9; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 9:
            // Auto 3, Stage 9 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[27]);
              autoStage = 10; // Advances to the next stage once the elevator is at the correct level.
            } 
          break;

          case 10:
            // Auto 3, Stage 10 code goes here.
            swerve.driveTo(scoringPositionsX[27],scoringPositionsY[27],scoringHeadings[27]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[11]);
              autoStage = 11; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 11:
            // Auto 3, Stage 11 code goes here.
            swerve.driveTo(scoringPositionsX[11],scoringPositionsY[11],scoringHeadings[11]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 12; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 12:
            // Auto 3, Stage 12 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 13; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 13:
            // Auto 3, Stage 13 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 14; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 14:
            // Auto 3, Stage 14 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
          break;
        }   
      break;

      case auto4:
        switch (autoStage) {
          case 1:
            // Auto 4, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[2],scoringPositionsY[2],scoringHeadings[2]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2; // Advances to the next stage once the robot is at the correct location.
            }
          break;

          case 2:
            // Auto 4, Stage 2 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L2); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 3; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 3:
            // Auto 4, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 4; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 4:
            // Auto 4, Stage 4 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[25]);
              autoStage = 5; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 5:
            // Auto 4, Stage 5 code goes here.
            swerve.driveTo(scoringPositionsX[25],scoringPositionsY[25],scoringHeadings[25]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[1]);
              autoStage = 6; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 6:
            // Auto 4, Stage 6 code goes here.
            swerve.driveTo(scoringPositionsX[1],scoringPositionsY[1],scoringHeadings[1]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 7; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 7:
            // Auto 4, Stage 7 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 8; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 8:
            // Auto 4, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 9; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 9:
            // Auto 4, Stage 9 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[25]);
              autoStage = 10; // Advances to the next stage once the elevator is at the correct level.
            } 
          break;

          case 10:
            // Auto 4, Stage 10 code goes here.
            swerve.driveTo(scoringPositionsX[25],scoringPositionsY[25],scoringHeadings[25]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[7]);
              autoStage = 11; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 11:
            // Auto 4, Stage 11 code goes here.
            swerve.driveTo(scoringPositionsX[7],scoringPositionsY[7],scoringHeadings[7]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 12; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 12:
            // Auto 4, Stage 12 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 13; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 13:
            // Auto 4, Stage 13 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 14; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 14:
            // Auto 4, Stage 14 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
          break;
        }
      break;

      case auto5:
        switch (autoStage) {
          case 1:
            // Auto 5, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[4],scoringPositionsY[4],scoringHeadings[4]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2; // Advances to the next stage once the robot is at the correct location.
            }
          break;

          case 2:
            // Auto 5, Stage 2 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 3; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 3:
            // Auto 5, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 4; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 4:
            // Auto 5, Stage 4 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[27]);
              autoStage = 5; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 5:
            // Auto 5, Stage 5 code goes here.
            swerve.driveTo(scoringPositionsX[27],scoringPositionsY[27],scoringHeadings[27]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[5]);
              autoStage = 6; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 6:
            // Auto 5, Stage 6 code goes here.
            swerve.driveTo(scoringPositionsX[5],scoringPositionsY[5],scoringHeadings[5]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 7; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 7:
            // Auto 5, Stage 7 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 8; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 8:
            // Auto 5, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 9; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 9:
            // Auto 5, Stage 9 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[27]);
              autoStage = 10; // Advances to the next stage once the elevator is at the correct level.
            } 
          break;

          case 10:
            // Auto 5, Stage 10 code goes here.
            swerve.driveTo(scoringPositionsX[27],scoringPositionsY[27],scoringHeadings[27]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[11]);
              autoStage = 11; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 11:
            // Auto 5, Stage 11 code goes here.
            swerve.driveTo(scoringPositionsX[11],scoringPositionsY[11],scoringHeadings[11]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 12; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 12:
            // Auto 5, Stage 12 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 13; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 13:
            // Auto 5, Stage 13 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 14; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 14:
            // Auto 5, Stage 14 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
          break;
        }   
      break;

      case auto6:
        switch (autoStage) {
          case 1:
            // Auto 6, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[2],scoringPositionsY[2],scoringHeadings[2]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2; // Advances to the next stage once the robot is at the correct location.
            }
          break;

          case 2:
            // Auto 6, Stage 2 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 3; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 3:
            // Auto 6, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 4; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 4:
            // Auto 6, Stage 4 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[25]);
              autoStage = 5; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 5:
            // Auto 6, Stage 5 code goes here.
            swerve.driveTo(scoringPositionsX[25],scoringPositionsY[25],scoringHeadings[25]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[1]);
              autoStage = 6; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 6:
            // Auto 6, Stage 6 code goes here.
            swerve.driveTo(scoringPositionsX[1],scoringPositionsY[1],scoringHeadings[1]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 7; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 7:
            // Auto 6, Stage 7 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 8; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 8:
            // Auto 6, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 9; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 9:
            // Auto 6, Stage 9 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
            if (elevator.atSetpoint()) {
              coralTimer.restart();
              swerve.resetDriveController(scoringHeadings[25]);
              autoStage = 10; // Advances to the next stage once the elevator is at the correct level.
            } 
          break;

          case 10:
            // Auto 6, Stage 10 code goes here.
            swerve.driveTo(scoringPositionsX[25],scoringPositionsY[25],scoringHeadings[25]); // This moves the robot to the source.
            if (coralSpitter.coralDetected() || coralTimer.get() > 3.8) {
              swerve.resetDriveController(scoringHeadings[7]);
              autoStage = 11; // Advances to the next stage once the robot has driven to the source and coral is detected in the intake.
            }
          break;

          case 11:
            // Auto 6, Stage 11 code goes here.
            swerve.driveTo(scoringPositionsX[7],scoringPositionsY[7],scoringHeadings[7]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 12; // Advances to the next stage once the robot is in the correct location.
            }
          break;

          case 12:
            // Auto 6, Stage 12 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.L4); // This moves the elevator to the second level.
            if (elevator.atSetpoint()) {
              coralSpitter.spit(); // Spits the coral.
              autoStage = 13; // Advances to the next stage once the elevator is at the correct level.
            }
          break;

          case 13:
            // Auto 6, Stage 13 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            if (!coralSpitter.isSpitting()) {
              autoStage = 14; // Advances to the next stage once the coral is ejected.
            }
          break;

          case 14:
            // Auto 6, Stage 14 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            elevator.setLevel(Level.bottom); // This moves the elevator to the bottom position.  
          break;
        }
      break;

      case auto7: //auto algae base on angle this looks like right algaes not sure doe
        switch (autoStage) { 
          case 1://logic: move reef ID10, if arrived, set elevator level to high algae
            speedScaleFactor = 0.90;//if it works it works
            swerve.driveTo(6.0, 4.0, 90.0); //move robot to reef (Improved)
            if(swerve.atDriveGoal()){
            elevator.setLevel(Level.highAlgae); //rise, to high algae
            speedScaleFactor = 0.65;
              autoStage = 2; //Advance to the next stage if location correct
            }
          break;
          case 2://logic: check position angle and get algae arms out and reset timmer for stage 3
            swerve.drive (0.0,0.0,0.0, false, 0.0,0.0); //stop
            swerve.driveTo(swerve.getXPos(), swerve.getYPos(), 90.0);//double check to make sure
            if(swerve.atDriveGoal() && elevator.atSetpoint()){//double condition, should be fine
             algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.algae); //algae down
             coralTimer.reset();//prepare for stage 3
              autoStage = 3;//go to next stage if correct
            }
            break;
          case 3: //logic: stop, lower back down after 2 second
            swerve.drive (0.0,0.0,0.0, false, 0.0,0.0); //stop
            if (coralTimer.get() > 2){ //prediction
              elevator.setLevel(Level.L1);//lower back down
              speedScaleFactor = 0.45;//"slow it down" -brendan
              autoStage = 4; //to the next stage
            }
            break;
          case 4: //logic: take algae to barge
            swerve.driveTo(scoringPositionsX[29],scoringPositionsY[29], scoringHeadings[29]);//move to scoring point - barge
            if (elevator.atSetpoint()){
              speedScaleFactor = 0.90;//catch up the wasted time
            }
            if(swerve.atDriveGoal()){
              elevator.setLevel(Level.L4); //rise, pretty sure the highest is fine
              algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.barge); //algae down
              autoStage = 5; //Advance to the next stage if location correct
                  }
              break;
          case 5:
          swerve.drive (0.0,0.0,0.0, false, 0.0,0.0); //stop
            if(algaeYeeter.armAtSetpoint()&&elevator.atSetpoint()){
              algaeYeeter.yeet(); //yeet algae
              autoStage = 6;
                }
            break;
          case 6: //logic: close algae mode, lower back down
            swerve.drive (0.0,0.0,0.0, false, 0.0,0.0); //stop
            if(swerve.atDriveGoal()){
              algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); //algae up
              elevator.setLevel(Level.bottom); //go back down
              speedScaleFactor = 0.65; //quto complete, back to default
                }
            break;
        }
      break;     
        //                   ٩(•̤̀ᵕ•̤́๑)ᵒᵏᵎᵎᵎᵎ    <---  El manqi can't go without an emoji

      case auto8:
        switch (autoStage) {
          case 1:
            // Auto 8, Stage 1 code goes here.
          break;

          case 2:
            // Auto 8, Stage 2 code goes here.
          break;
        }
      break;  

      case auto9:
        switch (autoStage) {
          case 1:
            // Auto 9, Stage 1 code goes here.
          break;

          case 2:
          // Auto 9, Stage 2 code goes here.
          break;
        }
      break;  

      case auto10:
        switch (autoStage) { 
          case 1:
            // Auto 10, Stage 1 code goes here.
          break;

          case 2:
          // Auto 10, Stage 2 code goes here.
          break;
        }
      break;     
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.
    coralSpitter.init(); // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
    algaeYeeter.init(); // Should be called in autoInit() and teleopInit(). Required for the algaeYeeter to function correctly.
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    // Sets both controllers to rumble for 0.7 seconds if a coral or algae has just been intaked or exhuasted.
    if ((coralSpitter.getExhaustTimer() > 0.1 && coralSpitter.getExhaustTimer() < 0.8) || (coralSpitter.getIntakeTimer() > 0.1 && coralSpitter.getIntakeTimer() < 0.8)
      || (algaeYeeter.getExhaustTimer() > 0.1 && algaeYeeter.getExhaustTimer() < 0.8) || (algaeYeeter.getIntakeTimer() > 0.1 && algaeYeeter.getIntakeTimer() < 0.8)) {
      operator.setRumble(RumbleType.kBothRumble, 1.0);
      driver.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      operator.setRumble(RumbleType.kBothRumble, 0.0);
      driver.setRumble(RumbleType.kBothRumble, 0.0);
    }

    if (driver.getPOV() == 0) currScoreMode = scoreMode.Algae; // D-Pad up
    if (driver.getPOV() == 90) currScoreMode = scoreMode.Branch; // D-pad left
    if (driver.getPOV() == 180) currScoreMode = scoreMode.L1; // D-pad down
    
    coralSpitter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
    algaeYeeter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the algaeYeeter to function correctly.

    if (driver.getRawButtonPressed(1)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(2)) boostMode = false; // B Button sets default mode (60% of full speed).
    if (elevator.getPosition() > 10.0) {
      speedScaleFactor = 0.15;
      boostMode = false;
    } else if (boostMode) {
      speedScaleFactor = 1.0;
    } else {
      speedScaleFactor = 0.65;
    }
    
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*rotationScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) { // X button
      swerveLock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (driver.getRawButtonPressed(6)) { // Right bumper button
      calcNearestScoringPose(); // Calculates the closest scoring position.
      swerve.resetDriveController(scoringHeadings[nearestScoreIndex]); // Prepares the robot to drive to the closest scoring position.
    } else if (driver.getRawButton(6)) { // Right bumper button
      if (nearestScoreIndex < 18) {
        swerve.driveTo(scoringPositionsX[nearestScoreIndex], scoringPositionsY[nearestScoreIndex], scoringHeadings[nearestScoreIndex]); // Drives to the closest scoring position.
      } else {
        swerve.aimDrive(xVel, yVel, scoringHeadings[nearestScoreIndex], true);
      }
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex();
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Left center button
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Updates the position of the robot on the field based on previous calculations.  

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Right center button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    
    // Controls the level of the elevator.
    if (Math.abs(MathUtil.applyDeadband(operator.getRightY(), 0.1)) >= 0.1) elevator.adjust(-operator.getRightY()); // Allows the operator to adjust the height of the elevator.
    if (operator.getRawButtonPressed(1)) { // A button
      elevator.setLevel(Level.L1);
      if (!algaeYeeter.algaeDetected()) {
        algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); 
        elevator.setLowLimit(0.5);
      }
    }
    if (operator.getRawButtonPressed(2)) { // B button
      elevator.setLevel(Level.L2); 
      if (!algaeYeeter.algaeDetected()) {
        algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); 
        elevator.setLowLimit(0.5);
      }
    }
    if (operator.getRawButtonPressed(3)) { // X button
      elevator.setLevel(Level.L3); 
      if (!algaeYeeter.algaeDetected()) {
        algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); 
        elevator.setLowLimit(0.5);
      }
    }
    if (operator.getRawButtonPressed(4)) { // Y button 
      elevator.setLevel(Level.L4);
      if (algaeYeeter.algaeDetected()) { // Automatically moves the algae yeeter to the barge position if algae is detected.
        algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.barge);
        elevator.setLowLimit(7.5);
      } else {
        algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); 
        elevator.setLowLimit(0.5);
      }
    }
    if (operator.getRawButtonPressed(5)) elevator.setLevel(Level.bottom); // Left bumper button
    if (operator.getLeftTriggerAxis() > 0.25) { // Left Trigger
      elevator.setLevel(Level.lowAlgae); 
      algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.algae); // Automatically extends the algae yeeter.
      elevator.setLowLimit(7.5);
    }
    if (operator.getRightTriggerAxis() > 0.25) { // Right Trigger
      elevator.setLevel(Level.highAlgae); 
      algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.algae); // Automatically extends the algae yeeter.
      elevator.setLowLimit(7.5);
    }

    // Controls the algae yeeter.
    if (operator.getPOV() == 180) {
      algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.algae); // D pad down
      elevator.setLowLimit(7.5);
      if (elevator.getPosition() < 7.5) {
        elevator.setLevel(Level.bottom);
      }
    }
    if (operator.getPOV() == 90) {
      algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.barge); // D pad left
      elevator.setLowLimit(7.5);
      if (elevator.getPosition() < 7.5) {
        elevator.setLevel(Level.bottom);
      }
    }
    if (operator.getPOV() == 0) {
      algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); // D pad up
      elevator.setLowLimit(0.5);
      if (elevator.getPosition() < 8.5) {
        elevator.setLevel(Level.bottom);
      }
    }
    if (operator.getPOV() == 270) algaeYeeter.yeet(); // D pad right

    // Controls the spitter
    if (operator.getRawButton(6)) coralSpitter.spit(); // Right bumper button

    // Controls the climber
    climber.setSpeed(MathUtil.applyDeadband(-operator.getLeftY(), 0.1)); // Left stick Y
    if (operator.getRawButtonPressed(8)) { // Right center button
      if (climber.isLatched()) {
        climber.openLatch();
      } else {
        climber.closeLatch();
      }
    }
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    autoSelected = autoChooser.getSelected();
    if (!autoCompleted) {
      if (autoSelected == auto2 || autoSelected == auto4 || autoSelected == auto6) {
        swerve.updateVisionHeading(true, 90.0);
      } else {
        swerve.updateVisionHeading(true, 180.0);
      }
    } else {
      swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    }
    swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), true);
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    SmartDashboard.putBoolean("Boost Mode", boostMode);
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    //SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Updates nearestScoreIndex to reflect the closest scoring location to the robot.
  public void calcNearestScoringPose() {
    // Calculates the distance to each scoring location using the distance formula.
    for (int i = 0 ; i < scoringPositionsX.length; i++) {
      scoreDistances[i] = Math.sqrt(Math.pow(scoringPositionsY[i] - swerve.getYPos(), 2) + Math.pow(scoringPositionsX[i] - swerve.getXPos(), 2));
    }

    double shortestDistance = scoreDistances[scoreDistances.length - 1]; // Stores the value of the shortest distance in the scoreDistances[] array.
    nearestScoreIndex = scoreDistances.length - 1; // Stores the index of the shortest distance in the scoreDistances[] array.

    // Finds the shortest distance in the scoreDistances[] array and updates nearestScoreIndex
    for (int i = 0; i < scoreDistances.length - 1; i++) {
      if ((currScoreMode == scoreMode.Branch) && (i <= 11 || i >= 24)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
      if ((currScoreMode == scoreMode.L1) && ((i >= 12 && i <= 17) || i >= 24)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
      if ((currScoreMode == scoreMode.Algae) && (i >= 18)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
    }
  }

  // Calculates all of the scoring locations on the field.
  public void calcScoringPoses() {
    scoringPositionsX[0] = (144.0-26.0/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.
    scoringPositionsY[0] = Drivetrain.fieldWidth/2.0 + 12.94*0.0254/2.0 - 10.25*0.0254; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.
    scoringHeadings[0] = 0.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 0th index by 60 degrees.
    for (int index = 1; index <= 5; index++) {
      scoringPositionsX[index] = (scoringPositionsX[0] - reefX)*Math.cos(Math.toRadians(index*60.0)) - (scoringPositionsY[0] - reefY)*Math.sin(Math.toRadians(index*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[0] - reefX)*Math.sin(Math.toRadians(index*60.0)) + (scoringPositionsY[0] - reefY)*Math.cos(Math.toRadians(index*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[0] + index*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // Calculates the scoring position of the other scoring position on the same face of the reef by using an x-offset and a y-offset.
    scoringPositionsX[6] = (144.0-26.0/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringPositionsY[6] = Drivetrain.fieldWidth/2.0 - 12.94*0.0254/2.0 - 10.25*0.0254; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringHeadings[6] = 0.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 6th index by 60 degrees.
    for (int index = 7; index <= 11; index++) {
      scoringPositionsX[index] = (scoringPositionsX[6] - reefX)*Math.cos(Math.toRadians((index-6)*60.0)) - (scoringPositionsY[6] - reefY)*Math.sin(Math.toRadians((index-6)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[6] - reefX)*Math.sin(Math.toRadians((index-6)*60.0)) + (scoringPositionsY[6] - reefY)*Math.cos(Math.toRadians((index-6)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[6] + (index-6)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // L1 scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[12] = 3.166;
    scoringPositionsY[12] = 3.640;
    scoringHeadings[12] = -15.0;

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 12th index by 60 degrees.
    for (int index = 13; index <= 17; index++) {
      scoringPositionsX[index] = (scoringPositionsX[12] - reefX)*Math.cos(Math.toRadians((index-12)*60.0)) - (scoringPositionsY[12] - reefY)*Math.sin(Math.toRadians((index-12)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[12] - reefX)*Math.sin(Math.toRadians((index-12)*60.0)) + (scoringPositionsY[12] - reefY)*Math.cos(Math.toRadians((index-12)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[12] + (index-12)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // Algae scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[18] = (144.0-33.5/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringPositionsY[18] = Drivetrain.fieldWidth/2.0; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringHeadings[18] = -90.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 18th index by 60 degrees.
    for (int index = 19; index <= 23; index++) {
      scoringPositionsX[index] = (scoringPositionsX[18] - reefX)*Math.cos(Math.toRadians((index-18)*60.0)) - (scoringPositionsY[18] - reefY)*Math.sin(Math.toRadians((index-18)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[18] - reefX)*Math.sin(Math.toRadians((index-18)*60.0)) + (scoringPositionsY[18] - reefY)*Math.cos(Math.toRadians((index-18)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[18] + (index-18)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // These 4 scoring locations correspond to the source. There are 2 scoring locations at each of the 2 sources, for a total of 4. 
    scoringPositionsX[24] = 1.037; // X-position of the first scoring location at the source nearest the origin in meters.
    scoringPositionsY[24] = 0.885; // Y-position of the first scoring location at the source nearest the origin in meters.
    scoringHeadings[24] = 144.0; // Heading of the first scoring location at the source nearest the origin. The source makes 54 and 36 degree angles with the coordinate axes.

    scoringPositionsX[25] = 1.037; // X-position of the second scoring location at the source nearest the origin in meters.
    scoringPositionsY[25] = 0.885; // Y-position of the second scoring location at the source nearest the origin in meters.
    scoringHeadings[25] = scoringHeadings[24]; // This is automatically calculated. Does not need to be edited.

    scoringPositionsX[26] = scoringPositionsX[24]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[26] = Drivetrain.fieldWidth - scoringPositionsY[24]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[26] = 36.0; // Heading of the first scoring location at the source furthest from the origin.

    scoringPositionsX[27] = scoringPositionsX[25]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[27] = Drivetrain.fieldWidth - scoringPositionsY[25]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[27] = scoringHeadings[26]; // This is automatically calculated. Does not need to be edited.

    // The scoring location of the processor.
    scoringPositionsX[28] = 11.561; // X-position of the processor scoring location in meters.
    scoringPositionsY[28] = Drivetrain.fieldWidth - 33.5*0.0254/2; // Y-position of the processor scoring location in meters.
    scoringHeadings[28] = 0.0; // Heading of the processor scoring location.

    // The scoring location of the barge.
    scoringPositionsX[29] = 7.697; // X-position of the barge scoring location in meters.
    scoringPositionsY[29] = 4.910; // Y-position of the barge scoring location in meters.
    scoringHeadings[29] = -90.0; // Heading of the barge scoring location.
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() { 
    swerve.resetDriveController(0.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.addCalibrationEstimate(0, false);
    swerve.pushCalibration(true, 180.0);
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading(true, 180.0);
    swerve.addVisionEstimate(0, true);
    swerve.updateOdometry();
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getAccurateCalibrationTimer: " + swerve.getAccurateCalibrationTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getGyroRoll: " + swerve.getGyroRoll());
    System.out.println("swerve getPathAngleError: " + swerve.getPathAngleError());
    System.out.println("swerve getPathPosError: " + swerve.getPathPosError());
    System.out.println("swerve getXPos: " + swerve.getXPos());
    System.out.println("swerve getXVel: " + swerve.getXVel());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVel: " + swerve.getYVel());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getAngleDist: " + swerve.getAngleDistance(30.0, -120.0));
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    swerve.updateDash();

    coralSpitter.init();
    coralSpitter.periodic();
    coralSpitter.spit();
    System.out.println("spitter coralDetected: " + coralSpitter.coralDetected());
    System.out.println("spitter getIntakeSensor: " + coralSpitter.getIntakeSensor());
    System.out.println("spitter getExhaustSensor: " + coralSpitter.getExhaustSensor());
    System.out.println("spitter isSpitting: " + coralSpitter.isSpitting());
    System.out.println("spitter intakeTimer: " + coralSpitter.getIntakeTimer());
    System.out.println("spitter exhaustTimer: " + coralSpitter.getExhaustTimer());
    coralSpitter.updateDash();

    elevator.setLevel(Level.bottom);
    if (elevator.getLevel() == Level.bottom) System.out.println("elevator at bottom");
    System.out.println("elevator atSetpoint: " + elevator.atSetpoint());
    System.out.println("elevator getPosition: " + elevator.getPosition());
    elevator.setLowLimit(0.5);
    elevator.updateDash();

    climber.setSpeed(0.0);
    climber.closeLatch();
    climber.openLatch();
    System.out.println("climber position: " + climber.getPosition());
    System.out.println("climber isLatched: " + climber.isLatched());
    climber.updateDash();

    algaeYeeter.init();
    algaeYeeter.periodic();
    algaeYeeter.setArmPosition(ArmPosition.stow);
    algaeYeeter.yeet();
    if (algaeYeeter.getArmPosition() == ArmPosition.stow) System.out.println("algae yeeter stowed");
    System.out.println("algae yeeter getArmAngle: " + algaeYeeter.getArmAngle());
    System.out.println("algae yeeter isYeeting: " + algaeYeeter.isYeeting());
    System.out.println("algae yeeter algaeDetected: " + algaeYeeter.algaeDetected());
    System.out.println("algae yeeter armAtSetpoint: " + algaeYeeter.armAtSetpoint());
    System.out.println("algae yeeter intakeTimer: " + algaeYeeter.getIntakeTimer());
    System.out.println("algae yeeter exhaustTimer: " + algaeYeeter.getExhaustTimer());
    algaeYeeter.updateDash();

    updateDash();
    calcScoringPoses();
    calcNearestScoringPose();
  }
}
