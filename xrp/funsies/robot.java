
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.XRPDrivetrain;

public class Robot extends TimedRobot {

  // Xbox controller on USB port 0
  private final CommandXboxController m_controller =
      new CommandXboxController(0);

  private final RobotContainer m_robotContainer;
  private XRPDrivetrain m_drive;
  private Command m_autonomousCommand;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_drive = m_robotContainer.getDrive();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Left stick Y: forward/reverse
    double forward = -m_controller.getLeftY();

    // Right stick X: turn left/right
    double rotation = -m_controller.getRightX();

    // Arcade drive
    m_drive.arcadeDrive(forward, rotation);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_drive.arcadeDrive(0, 0);
  }
}
