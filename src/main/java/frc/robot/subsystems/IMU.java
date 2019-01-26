package frc.robot.subsystems;

import frc.robot.utilities.MathUtils;

import com.kauailabs.navx.frc.AHRS; // http://www.pdocs.kauailabs.com/navx-mxp/

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * Communicates with navX-MXP
 */
public class IMU {
	private AHRS ahrs;

	private double previousAccelX;
	private double previousAccelY;

	private double offset = 0;

	/**
	 * Connects to the navX
	 */
	public void IMUInit() {
		try {
			/*
			 * Communicate w/navX MXP via the MXP SPI Bus. Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
			 */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}

	/**
	 * @return Angle in degrees
	 */
	public double getAngle() {
		return MathUtils.resolveDeg(ahrs.getYaw() - offset);
	}

	/**
	 * @return If a collision has been detected
	 */
	public boolean collisionDetected() {
		double currentAccelX;
		double currentAccelY;

		double jerkX;
		double jerkY;

		double collisionThreshold = 2.1;

		currentAccelX = ahrs.getWorldLinearAccelX();
		jerkX = currentAccelX - previousAccelX;
		previousAccelX = currentAccelX;
		currentAccelY = ahrs.getWorldLinearAccelY();
		jerkY = currentAccelY - previousAccelY;
		previousAccelY = currentAccelY;

		return ((Math.abs(jerkX) > collisionThreshold) || (Math.abs(jerkY) > collisionThreshold));
	}

	/**
	 * Resets IMU angle to 0
	 */
	public void reset(int offset) {
		ahrs.reset();
		setOffset(offset);
	}

	/**
	 * sets this.offset to offset
	 * @param offset
	 */
	public void setOffset(double offset) {
		this.offset = offset;
	}
}
