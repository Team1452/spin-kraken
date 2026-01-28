// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Shooter;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private final Shooter shooter = new Shooter();
  double targetRPM = 500.0;
  double targetUnitsPer100ms = (targetRPM * 2048.0) / 600.0;
  private final CommandXboxController controller = new CommandXboxController(0);


  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    //m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    controller.a().onTrue(shooter.simpleShoot());
    controller.b().onTrue(shooter.controllerShoot(targetRPM));
  }
}
