 public void teleopPeriodic() {
    refreshStatusSignals();
    swerve.updateOdometry();
    swerve.addVisionEstimate(0.04, 0.04, 10); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    if (driver.getRawButtonPressed(4)) { // Y Button
      speedScaleFactor = 1.0;
    }
    if (driver.getRawButtonPressed(2)) { // B button
      speedScaleFactor = 0.6;
    }
    if (driver.getRawButtonPressed(1)) { // A button
      speedScaleFactor = 0.15;
    }

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05) * speedScaleFactor)
        * Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05) * speedScaleFactor)
        * Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05) * speedScaleFactor)
        * Drivetrain.maxAngularVelTeleop;

    // Auto Rotate to Aim Heading
    boolean rightTriggerPressed = driver.getRightTriggerAxis() > 0.25;
    boolean leftTriggerPressed = driver.getLeftTriggerAxis() > 0.25;
    if (driver.getRawButtonPressed(6)) { // Right Bumper
      swerve.resetDriveController(getAimHeading());
    } else if (driver.getRawButtonPressed(5)) { // Left Bumper
      swerve.resetDriveController(swerve.isBlueAlliance() ? -90.0 : 90.0); // Rotate to amp.
    } else if (rightTriggerPressed && !rightTriggerWasPressed) {
      swerve.resetDriveController(180.0);
    } else if (leftTriggerPressed && !leftTriggerWasPressed) {
      swerve.resetDriveController(swerve.isBlueAlliance() ? -90.0 : 90.0); // Rotate to amp.
    }
    rightTriggerWasPressed = rightTriggerPressed;
    leftTriggerWasPressed = leftTriggerPressed; 

    if (driver.getRawButton(6)) { // Right Bumper
      swerve.driveTo(1.89, (swerve.isBlueAlliance() ? 5.56 : Drivetrain.fieldWidth - 5.56), getAimHeading()); // Snap to speaker.
    } else if (driver.getRawButton(5)) { // Left Bumper
      swerve.driveTo(1.8, (swerve.isBlueAlliance() ? 7.42 : Drivetrain.fieldWidth - 7.42), (swerve.isBlueAlliance() ? -90.0 : 90.0)); // Snap to amp.
    } else if (rightTriggerPressed) {
      swerve.aimDrive(xVel, yVel, 180.0, true);
    } else if (leftTriggerPressed) {
      swerve.aimDrive(xVel, yVel, swerve.isBlueAlliance() ? -90.0 : 90.0, true);
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary.
    if (driver.getRawButtonPressed(7)) {
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) {
      swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButtonReleased(7)) {
      swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.
    }

    arm.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the arm.
    if (climber.getUserLockout()) { // Climber is not active.
      if (operator.getRawButtonPressed(1)) { // A Bytton
        currArmState = ArmState.DRIVE;
      }
      if (operator.getRawButtonPressed(2)) { // B Button
        currArmState = ArmState.INTAKE;
      }
      if (operator.getRawButtonPressed(3)) { // X Button
        currArmState = ArmState.SHOOT;
      }
      if (operator.getRawButtonPressed(4)) { // Y Button
        currArmState = ArmState.AMP;
      }
      if (operator.getRawButtonPressed(8)) { // Menu Button
        currArmState = ArmState.MANUAL_SHOOT;
      }

      switch (currArmState) {
        case INTAKE:
          arm.updateSetpoint(armIntakeSetpoint);
          thrower.setDisableFlywheel(true);
          lastIsAmpScoring = false;
          break;

        case DRIVE:
          arm.updateSetpoint(armDriveSetpoint);
          thrower.setDisableFlywheel(true);
          lastIsAmpScoring = false;
          break;

        case SHOOT:
          arm.updateSetpoint(getAimArmAngle());
          thrower.setDisableFlywheel(false);
          lastIsAmpScoring = false;
          break;

        case AMP:
          if (thrower.isAmpScoring()) {
            if (!lastIsAmpScoring) {
              ampTimer.restart(); // This timer measures the time since the arm has begun the amp scoring process.
            }
            arm.updateSetpoint(armAmpSetpoint + armAmpRaiseRate * ampTimer.get()); // Raises the arm at 6 deg/sec.
            lastIsAmpScoring = true;
          } else {
            lastIsAmpScoring = false;
            arm.updateSetpoint(armAmpSetpoint);
            thrower.setDisableFlywheel(true);
          }
          break;

        case MANUAL_SHOOT:
          arm.updateSetpoint(armManualSetpoint);
          thrower.setDisableFlywheel(false);
          lastIsAmpScoring = false;
          break;

        default:
          break;

      }
    }

    boolean hasNote = thrower.getSensor1() || thrower.getSensor2() || thrower.getSensor3(); // Rumble cue when the robot intakes a note.
    if ((hasNote && !hadNote) || (!hasNote && hadNote)) { // Note Pickup Rumble Cue
      rumbleTimer.restart();
      driver.setRumble(RumbleType.kBothRumble, 0.2);
      operator.setRumble(RumbleType.kBothRumble, 0.2);
    }
    if (rumbleTimer.get() > 0.4) {
      driver.setRumble(RumbleType.kBothRumble, 0.0);
      operator.setRumble(RumbleType.kBothRumble, 0.0);
    }
    hadNote = hasNote;

    thrower.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the thrower.
    if (climber.getUserLockout()) {
      if (operator.getRawButton(6)) { // Right Bumper
        if (currArmState == ArmState.SHOOT || currArmState == ArmState.MANUAL_SHOOT && arm.atSetpoint()) {
          thrower.commandThrow(); // Commands the thrower to throw a note with the commanded flywheel velocity in rotations per second.
        } else if (currArmState == ArmState.AMP && arm.atSetpoint()) {
          thrower.commandAmpScore();
        }
      }
    } else {
      thrower.setDisableFlywheel(true);
    }

    if (operator.getRawButtonPressed(7) && arm.getEncoderLeft() < 10.0) { // Mode Button
      climber.disableUserLockout();
    }
    climber.setManual(MathUtil.applyDeadband(-operator.getLeftY(), 0.1), MathUtil.applyDeadband(-operator.getRightY(), 0.1));
  }
