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
