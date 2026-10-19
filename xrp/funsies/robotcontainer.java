
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final XRPDrivetrain m_xrpDrivetrain =
      new XRPDrivetrain();

  private final ExampleCommand m_autoCommand =
      new ExampleCommand(m_xrpDrivetrain);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  // Return the existing drivetrain; do not create another one.
  public XRPDrivetrain getDrive() {
    return m_xrpDrivetrain;
  }
}
