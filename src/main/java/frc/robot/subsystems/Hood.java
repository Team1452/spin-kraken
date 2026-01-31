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
  private final double minCentimeters;
  private final double maxCentimeters;

  // Last commanded percent (0..1) used to estimate position when feedback is absent
  private double lastCommandPercent = 0.0;

  /**
   * Create an open-loop linear actuator controller on the given PWM channel.
   *
   * @param pwmChannel PWM channel for the actuator (port number)
   * @param minCentimeters minimum extension (cm) corresponding to PWM=0.0
   * @param maxCentimeters maximum extension (cm) corresponding to PWM=1.0
   */
  public Hood(int pwmChannel, double minCentimeters, double maxCentimeters) {
    this.actuator = new Servo(pwmChannel);
    this.hasFeedback = false;
    this.feedbackCentimeters = null;
    this.minCentimeters = minCentimeters;
    this.maxCentimeters = maxCentimeters;
  }

  public void setPosition(double position) {
    actuator.set(position);
  }

  public void setSpeed(double speed) {
    actuator.set(speed);
  }

  /**
   * Set actuator by normalized percent (0.0..1.0). Clamped to [0,1].
   */
  public void setPositionPercent(double percent) {
    double p = MathUtil.clamp(percent, 0.0, 1.0);
    actuator.set(p);
    lastCommandPercent = p;
    Logger.recordOutput("LinearActuator/CommandedPercent", p);
  }

  /**
   * Set actuator by physical position in centimeters (mapped linearly between min/max centimeters).
   */
  public void setPositionCentimeters(double centimeters) {
    if (Double.compare(maxCentimeters, minCentimeters) == 0) {
      // avoid division by zero; treat as 0.0
      setPositionPercent(0.0);
      return;
    }
    double clamped = MathUtil.clamp(centimeters, minCentimeters, maxCentimeters);
    double percent = (clamped - minCentimeters) / (maxCentimeters - minCentimeters);
    setPositionPercent(percent);
    Logger.recordOutput("LinearActuator/CommandedCentimeters", clamped);
  }

  /**
   * Get the current position in centimeters. If feedback is not available, this returns an
   * estimate based on the last commanded percent.
   */
  public double getPositionCentimeters() {
    if (hasFeedback) {
      return feedbackCentimeters.getAsDouble();
    }
    // estimate
    return lastCommandPercent * (maxCentimeters - minCentimeters) + minCentimeters;
  }

  /**
   * Return a one-shot command that sets the actuator to the requested position (centimeters).
   * If feedback is available, the returned command will wait until the actuator reaches the
   * requested position within the given tolerance (centimeters).
   */
  public Command moveToCentimetersCommand(double centimeters, double toleranceCentimeters) {
    if (hasFeedback) {
      return Commands.sequence(
          Commands.runOnce(() -> setPositionCentimeters(centimeters), this),
          Commands.waitUntil(() -> Math.abs(getPositionCentimeters() - centimeters) <= toleranceCentimeters));
    } else {
      // Open-loop: just command the setpoint
      return Commands.runOnce(() -> setPositionCentimeters(centimeters), this);
    }
  }

  /**
   * Convenience: one-shot move to percent command.
   */
  public Command moveToPercentCommand(double percent) {
    return Commands.runOnce(() -> setPositionPercent(percent), this);
  }

  public Command setPositionCommand(double position) {
    return Commands.runOnce(() -> setPosition(position), this);
  }

  public Command setSpeedCommand(double speed) {  
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

@Override
  public void periodic() {
    System.out.println("Hood Position: " + getPositionCentimeters() + " cm");
  }

}
