// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnNeoWithJoystick;

public class Drivetrain extends SubsystemBase {
  public TalonSRX testNeo = new TalonSRX(11);

  // Create new DutyCycleEncoder to test
  DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(0);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left y-axis", RobotContainer.m_driverController.getLeftY());
    SmartDashboard.putString("Color Value", TurnNeoWithJoystick.m_colorSensor.getColor().toString());
    SmartDashboard.putString("Estimated Color", TurnNeoWithJoystick.hexToPrimaryColor(TurnNeoWithJoystick.m_colorSensor.getColor().toString()));
    SmartDashboard.putNumber("ThruBoreEncoder Absolute Pos", throughBoreEncoder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
