package frc.team1706.robot.subsystems;

import frc.team1706.robot.utilities.MathUtils;

import com.kauailabs.navx.frc.AHRS; // http://www.pdocs.kauailabs.com/navx-mxp/

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class IMU {
	private static AHRS ahrs;

	private double previousAccelX;
	private double previousAccelY;

	private double currentAccelX;
	private double currentAccelY;

	private double jerkX;
	private double jerkY;

	private double collisionThreshold = 2.1;

	private double offset = 0;

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

	public double getAngle() {
		//This returns degrees (0 to 360)
		return MathUtils.resolveDeg(ahrs.getYaw() + offset);
	}

	public static double getVelocity() {
		return Math.sqrt((Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2)));
	}

	public boolean collisionDetected() {
		boolean collisionDetected = false;

		currentAccelX = ahrs.getWorldLinearAccelX();
		jerkX = currentAccelX - previousAccelX;
		previousAccelX = currentAccelX;
		currentAccelY = ahrs.getWorldLinearAccelY();
		jerkY = currentAccelY - previousAccelY;
		previousAccelY = currentAccelY;

		if ((Math.abs(jerkX) > collisionThreshold) || (Math.abs(jerkY) > collisionThreshold)) {
			collisionDetected = true;
		}

		return collisionDetected;
	}

	public double getVelocityX() {
		return ahrs.getVelocityX();
	}

	public double getVelocityY() {
		return ahrs.getVelocityY();
	}

	public void reset() {
		ahrs.reset();
		setOffset(0);
	}

	public void setOffset(double offset) {
		this.offset = offset;
	}
}
