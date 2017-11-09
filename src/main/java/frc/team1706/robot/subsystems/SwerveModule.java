package frc.team1706.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

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
	private double keepPID;
	private double distance;
	private double previousDistance = 0;
	private double delta;
	private double wheelRotation;
	private double angleError;
	private double actualAngle;
	private double rightSum = 0;
	private double forwardSum = 0;
	private Talon translationMotor;
	private CANTalon rotationMotor;
	private Encoder encoder;
	private boolean wheelReversed;
	private boolean movingRight;
	private boolean movingFor;
	private double rawError;
	private double rac;
	private double rightDelta;
	private double forwardDelta;

	private double i;
	private double j;
	private double k;
	private double z;

	private int id;
	
	/**
	 */
	SwerveModule(int pwmPortT, int pwmPortR, int encoderPort1, int encoderPort2) {
		super();

		translationMotor = new Talon(pwmPortT);
		rotationMotor = new CANTalon(pwmPortR);
		rotationMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
		rotationMotor.changeControlMode(TalonControlMode.Position);
		rotationMotor.setPID(27.5, 0.0, 0.0);//TODO Tune PID
		rotationMotor.setAllowableClosedLoopErr(2);
		rotationMotor.reverseSensor(true);
		rotationMotor.enable();

		encoder = new Encoder(encoderPort1, encoderPort2, false, Encoder.EncodingType.k4X);
		encoder.setDistancePerPulse(0.04);
	}

	void drive() {
		
		rotationMotor.changeControlMode(TalonControlMode.Position);
		
		distance = encoder.getDistance();
		
//		rotationMotor.setAnalogPosition(MathUtils.unwrapCAN(rotationMotor.getAnalogInPosition()));

		angleError = rotationMotor.getError();//MathUtils.calculateError(this.angleCommand, MathUtils.convertRange(0, 1023, 0, Math.PI * 2, rotationMotor.get()));

		/*
		 * If the error is greater than 90 then change direction by 180
		 * and reverse the wheel speed command.
		 * Also adjust error now to match the new wheel command.
		 */
		rawError = angleError;
		
		if (wheelReversed) {
			delta = previousDistance - distance;
		} else {
			delta = distance - previousDistance;
		}
		
		if (speedCommand == 0) {
			angleError = 0;
		}

		i = Math.floor(rotationMotor.getAnalogInPosition()/1024);
		j = this.angleCommand + i*1024;
		k = j - rotationMotor.getAnalogInPosition();
		
		if (Math.abs(k) > 512) {
			z = -(j - Math.signum(k) * 1024);
		} else {
			z = -j;
		}
		
		if (Math.abs(MathUtils.getDelta(-z, rotationMotor.getAnalogInPosition())) > 256) {
			z += Math.signum(MathUtils.getDelta(-z, rotationMotor.getAnalogInPosition())) * 512;
			wheelReversed = true;
			this.speedCommand *= -1;

		} else {
			wheelReversed = false;
		}
		
		/*
		 * If wheel direction has to change more than 45 degrees
		 * then set wheel speed command to 0 while wheel is
		 * turning.
		 */
		if (Math.abs(angleError) < 128) {
			translationMotor.set(this.speedCommand);
		} else {
			translationMotor.set(0.0);
		}
		
		if (Math.abs(this.speedCommand) >= 0.1) {
			rotationMotor.setSetpoint(z);//this.angleCommand);SmartDashboard.getNumber("Manual CAN", 0);//SmartDashboard.getNumber("Manual CAN", 0));//
		}
		
		rightDelta = delta * Math.sin(MathUtils.degToRad(rac));
		forwardDelta = delta * Math.cos(MathUtils.degToRad(rac));
		
		if (Math.sin(MathUtils.degToRad(rac - MathUtils.radToDeg(offset))) > 0) {
			movingRight = true;
		} else {
			movingRight = false;
		}
		
		if (Math.cos(MathUtils.degToRad(rac - MathUtils.radToDeg(offset))) > 0) {
			movingFor = true;
		} else {
			movingFor = false;
		}

		rightSum += rightDelta;
		forwardSum += forwardDelta;
		
		previousDistance = distance;
		
	}
	
	public void setID(int id) {
		this.id = id;
	}

	// for testing wiring only
	public void setDirectRotateCommand(double command) {
		rotationMotor.changeControlMode(TalonControlMode.PercentVbus);
		rotationMotor.set(-command);
	}

	// for testing wiring only
	public void setDirectTranslateCommand(double command) {
		translationMotor.set(command);
	}

	public void setSpeedCommand(double speedCommand) {
		this.speedCommand = speedCommand;
	}

	public void setAngleCommand(double angleCommand) {
		this.angleCommand = angleCommand;
	}

	public double getSpeedCommand() {
		return this.speedCommand;
	}

	public double getAngleCommand() {
		return this.angleCommand;
	}

	public double getOffset() {
		return offset;
	}

	public void setOffset(double offset) {
		this.offset = MathUtils.degToRad(offset);
	}

	public Vector getPosition() {
		return position;
	}

	public void setPosition(Vector position) {
		this.position = position;
	}

	public double getAngleError() {
		return this.angleError;
	}

	public double getWheelRotation() {
		return this.wheelRotation;
	}

	public double getActualAngle() {
		return actualAngle;
	}

	public double getDistance() {
		return distance;
	}

	public void resetDistance() {
		encoder.reset();
	}
	
	public boolean getReversed(){
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
	public double getrac(){
		return rac;
	}
	
	public boolean getRight(){
		return movingRight;
	}
	
	public boolean getFor(){
		return movingFor;
	}
}
