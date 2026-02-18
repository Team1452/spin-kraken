package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Climber extends SubsystemBase {

	/** Simple config placeholders — replace with real IDs. */
	public static final class Config {
		public static final int TALON_CAN_ID = 11; // TODO: change to real CAN ID
		public static final int LIMIT_ANALOG_CHANNEL = 0; // TODO: change to real analog channel
		public static final double LIMIT_VOLTAGE_THRESHOLD = 2.0; // voltage above/below which the limit is considered pressed
		public static final double DEFAULT_SPEED = 0.5;

		// Absolute positions (in rotations) for the two mechanical endpoints. Adjust to
		// match your climber geometry. Commonly, retracted = 0.0 and extended = some
		// positive number of rotations.
		public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
		public static final double EXTENDED_POSITION_ROTATIONS = 10.0; // TODO: set to real value
	}

	private final TalonFX motor;
	//private final AnalogInput limitSwitch;

	private boolean zeroed = false;

	private double lastCommandedSpeed = 0.0;

	private boolean limitPreviouslyClosed = false;

	/**
	 * Create a Climber subsystem using explicit hardware IDs.
	 *
	 * @param talonCanId CAN ID for the TalonFX
	 * @param limitAnalogChannel analog channel for the limit switch
	 */

    public Climber(int talonCanId) {
        this.motor = new TalonFX(talonCanId);
    }

	public void setSpeed(double speed){
        motor.set(speed);
    }

	/** Stop the climber motor. */
	public void stop() {
		lastCommandedSpeed = 0.0;
		motor.stopMotor();
	}

	public double getPosition() {
		return motor.getPosition().getValueAsDouble();
	}

	
	public void zeroPosition() {
		double raw = motor.getPosition().getValueAsDouble();
		zeroed = true;
		// Also update TalonFX internal position (timeout 0.25s)
		motor.setPosition(0.0, 0.25);
	}

	public boolean isZeroed() {
		return zeroed;
	}

	private double clampSpeed(double speed) {
		if (speed > 1.0) return 1.0;
		if (speed < -1.0) return -1.0;
		return speed;
	}

	@Override
	public void periodic() {
		// Put subsystem periodic code here. E.g. telemetry for tuning/debug.
	}

    public Command setSpeedCommand(double speed) {
        return Commands.run(() -> setSpeed(speed), this);
    }

}
