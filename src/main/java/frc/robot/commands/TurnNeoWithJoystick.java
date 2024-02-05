// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class TurnNeoWithJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;

  private static I2C.Port i2cPort = I2C.Port.kOnboard;

  public static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnNeoWithJoystick(Drivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting joystick drive command...");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get left forward value on xbox controller
    double speed = RobotContainer.m_driverController.getLeftY();
    while(hexToPrimaryColor(TurnNeoWithJoystick.m_colorSensor.getColor().toString()) == "blue") {
      // run 50%
      m_subsystem.testNeo.set(ControlMode.PercentOutput, (speed / 2.0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // convert hex values to rgb
  public static String hexToPrimaryColor(String hex) {
    // remove any leading #
    hex = hex.replaceAll("^#", "");

    // Parse hex string to rgb values
    int r = Integer.parseInt(hex.substring(0,2), 16);
    int g = Integer.parseInt(hex.substring(2, 4), 16);
    int b = Integer.parseInt(hex.substring(4, 6), 16);
    
    // Calculate the distance to each primary color
    double distanceToRed = calculateColorDistance(r, 0, 0);
    double distanceToGreen = calculateColorDistance(0, g, 0);
    double distanceToBlue = calculateColorDistance(0, 0, b);
    
    // Determine the closest primary color
    if (distanceToRed < distanceToGreen && distanceToRed < distanceToBlue) {
      System.out.println("red");
      return "red";
    } else if (distanceToGreen < distanceToRed && distanceToGreen < distanceToBlue) {
      System.out.println("green");
      return "green";
    } else {
      System.out.println("blue");
      return "blue";
    }
  }

  // Calculate the Euclidean distance between two colors
  public static double calculateColorDistance(int r1, int g1, int b1) {
    int r2 = 255; // Red component of the primary color
    int g2 = 255; // Green component of the primary color
    int b2 = 255; // Blue component of the primary color

    return Math.sqrt(Math.pow(r2 - r1, 2) + Math.pow(g2 - g1, 2) + Math.pow(b2 - b1, 2));
  }

}
