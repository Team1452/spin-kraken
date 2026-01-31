package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simple PWM linear-actuator controller.
 *
 * Features:
 * - Open-loop PWM control via {@link edu.wpi.first.wpilibj.Servo} (0.0..1.0).
 * - Optional position feedback via a {@link DoubleSupplier} that returns position in centimeters.
 * - Convenience command factories (one-shot and wait-until-on-target when feedback is present).
 *
 * Notes / assumptions:
 * - If you provide a feedback supplier, it must return the actuator position in the same
 *   units as the min/max meters passed to the constructor (meters by convention here).
 * - If no feedback supplier is provided, commands are open-loop (they issue a PWM setpoint
 *   but cannot detect when the mechanism is on-target).
 */
public class Hood extends SubsystemBase {
  private final Servo actuator;
  private final boolean hasFeedback;
  private final DoubleSupplier feedbackCentimeters; // may be null if no feedback

  // Last commanded percent (0..1) used to estimate position when feedback is absent

  /**
   * Create an open-loop linear actuator controller on the given PWM channel.
   *
   * @param pwmChannel PWM channel for the actuator (port number)
   * @param minCentimeters minimum extension (cm) corresponding to PWM=0.0
   * @param maxCentimeters maximum extension (cm) corresponding to PWM=1.0
   */
  public Hood(int pwmChannel) {
    this.actuator = new Servo(pwmChannel);
    this.hasFeedback = false;
    this.feedbackCentimeters = null;
  }

  public void setPosition(double position) {
    actuator.set(position);
  }

  public void setSpeed(double speed) {
    actuator.set(speed);
  }
  
  public double getPositionCentimeters() {
    return actuator.getPosition();
  }

  public Command setPositionCommand(double position) {
    System.out.println("Setting hood position to: " + position);
    return Commands.runOnce(() -> setPosition(position), this);
  }

  public Command setSpeedCommand(double speed) {  
    System.out.println("Setting hood speed to: " + speed);
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

@Override
  public void periodic() {
    System.out.println("Hood Position: " + getPositionCentimeters() + " cm");
  }

}
