package frc.team1706.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.Vector;

/**
 * A swerve module.
 */
public class SwerveModule {
	private Vector position;
	private double offset;
	private double speedCommand;
	private double angleCommand;
	private double distance;
	private double previousDistance = 0.0;
	private double previousRobotDistance = 0.0;
	private double delta;
	private double angleError;
	private double rightSum = 0;
	private double forwardSum = 0;
	private Talon translationMotor;
	private TalonSRX rotationMotor;
	private Encoder encoder;
	private boolean wheelReversed;
	private double rawError;
	private double rightDelta;
	private double forwardDelta;
	private double currentAngle;

	private int encoderCheck = 0;

	private double i;
	private double j;
	private double k;
	private double z;

	private int id;

	private boolean encoderAlive = true;

	/**
	 */
	SwerveModule(int pwmPortT, int pwmPortR, int encoderPort1, int encoderPort2) {
		super();

		translationMotor = new Talon(pwmPortT);
		rotationMotor = new TalonSRX(pwmPortR);
		rotationMotor. configSelectedFeedbackSensor( FeedbackDevice.Analog, 0, 0);
		rotationMotor.config_kF(0, 0.0, 0);
		rotationMotor.config_kP(0, 22, 0);
		rotationMotor.config_kI(0, 0.0, 0);
		rotationMotor.config_kD(0, 0.0, 0);
		rotationMotor. configAllowableClosedloopError (0, 2, 0);
		rotationMotor.setSensorPhase(true);

		encoder = new Encoder(encoderPort1, encoderPort2, false, Encoder.EncodingType.k4X);
		encoder.setDistancePerPulse(0.04);
	}

	void drive() {

		currentAngle = rotationMotor.getSensorCollection().getAnalogIn();

		distance = encoder.getDistance();

		angleError = rotationMotor.getClosedLoopError(0);

		rawError = angleError;

		delta = distance - previousDistance;

		if (speedCommand == 0) {
			angleError = 0;
		}

		//Count rotation cycles of wheel
		i = Math.floor(currentAngle / 1024);
		//Set command + rotations (wrap command)
		j = this.angleCommand + i * 1024;
		//Set wrapped command - current position (error)
		k = j - currentAngle;

		/*
		 * If the error is greater than 512 units (180 degrees), have wheel go to next
		 * cycle so it doesn't jump back to beginning of current cycle
		 */
		if (Math.abs(k) > 512) {
			z = -(j - Math.signum(k) * 1024);
		} else {
			z = -j;
		}

		/*
		 * If the wheel has to move over 256 units (90 degrees)
		 * go opposite to command and reverse translation
		 */
		if (Math.abs(MathUtils.getDelta(-z, currentAngle)) > 256) {
			z += Math.signum(MathUtils.getDelta(-z, currentAngle)) * 512;
			wheelReversed = true;
			this.speedCommand *= -1;

		} else {
			wheelReversed = false;
		}

		/*
		 * If wheel direction has to change more than 128 units (45 degrees)
		 * then set wheel speed command to 0 while wheel is turning.
		 */
		if (Math.abs(angleError) < 128) {
			translationMotor.set(this.speedCommand);
		} else {
			translationMotor.set(0.0);
		}

		if (Math.abs(this.speedCommand) >= 0.1) {
			rotationMotor.set(ControlMode.Position, z);
		}

		if ((SwerveDrivetrain.getRobotDistance() != previousRobotDistance) && (distance == previousDistance)) {
			encoderCheck++;
		} else {
			encoderCheck = 0;
		}

		if (encoderCheck == 5) {
			encoderAlive = false;
		}

		//Debugging
		if (id == 1) {
			SmartDashboard.putNumber("Motor AngleFR", Math.toDegrees(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset)))));
		} else if (id == 2) {
			SmartDashboard.putNumber("Motor AngleFL", Math.toDegrees(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset)))));
		} else if (id == 3) {
			SmartDashboard.putNumber("Motor AngleBL", Math.toDegrees(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset)))));
		} else {
			SmartDashboard.putNumber("Motor AngleBR", Math.toDegrees(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset)))));
		}

		rightDelta = delta * Math.sin(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset))));
		forwardDelta = delta * Math.cos(MathUtils.resolveAngle(Math.toRadians(currentAngle / 1024 * 360 - Math.toDegrees(offset))));

		previousDistance = distance;
		previousRobotDistance = SwerveDrivetrain.getRobotDistance();

	}

	public void setID(int id) {
		this.id = id;
	}

	// for testing wiring only
	public void setDirectRotateCommand(double command) {
		rotationMotor.set(ControlMode.PercentOutput, -command);
	}

	// for testing wiring only
	public void setDirectTranslateCommand(double command) {
		translationMotor.set(command);
	}

	public double getAngle() {
		return MathUtils.convertRange(0, 1024, 0, 360, currentAngle);
	}

	void setSpeedCommand(double speedCommand) {
		this.speedCommand = speedCommand;
	}

	void setAngleCommand(double angleCommand) {
		this.angleCommand = angleCommand;
	}

	double getSpeedCommand() {
		return this.speedCommand;
	}

	double getAngleCommand() {
		return this.angleCommand;
	}

	double getOffset() {
		return offset;
	}

	public void setOffset(double offset) {
		this.offset = Math.toRadians(offset);
	}

	Vector getPosition() {
		return position;
	}

	public void setPosition(Vector position) {
		this.position = position;
	}

	public double getAngleError() {
		return this.angleError;
	}

	public double getDistance() {
		return distance;
	}

	public void resetDistance() {
		encoder.reset();
	}

	public boolean getReversed() {
		return this.wheelReversed;
	}

	public double getRightSum() {
		if (wheelReversed) {
			return -rightSum;
		} else {
			return rightSum;
		}
	}

	public double getForwardSum() {
		if (wheelReversed) {
			return -forwardSum;
		} else {
			return forwardSum;
		}
	}

	public void resetDelta() {
		rightSum = 0.0;
		forwardSum = 0.0;
	}

	public double getRawError() {
		return rawError;
	}

	public double[] getXYDist() {
		if (wheelReversed) {
			forwardSum *= -1.0;
			rightSum *= -1.0;
		}
		double[] i = {rightDelta, forwardDelta};
		double[] j = {0.0, 0.0};
		if (encoderAlive) {
			return i;
		} else {
			return j;
		}
	}

	public boolean getEncoderAlive() {
		return encoderAlive;
	}
}
