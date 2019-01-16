package frc.team1706.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.PIDController;
import frc.team1706.robot.utilities.Vector;

/**
 * A swerve module.
 */
public class SwerveModule {
    private static final double TICKS_PER_REVOLUTION = 360.0;

    private Vector position;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double previousDistance = 0;
    private double previousRobotDistance = 0.0;
    private double angleError;
    private double rightSum = 0;
    private double forwardSum = 0;
    private SwerveMotor swerveMotor;
    private PIDController anglePID;
    private double angle;
    private double rightDelta;
    private double forwardDelta;
    private boolean wheelReversed;

    private int id;

    /**
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveModule(int canPortC, int canPortCC) {
        super();

        swerveMotor = new SwerveMotor(canPortC, canPortCC);

        anglePID = new PIDController(1.0, 0.0, 0.0);
        anglePID.setContinuous();
        anglePID.setInputRange(0.0, 360.0);
        anglePID.setOutputRange(-1.0, 1.0);

    }

    void drive() {
        double delta;
        double rightDelta;
        double forwardDelta;

        distance = swerveMotor.getDistance();

        angle = swerveMotor.getAngle();

        anglePID.setInput(angle);
        anglePID.setSetpoint(angleCommand);

        angleError = anglePID.getError();

        if (wheelReversed) {
            delta = previousDistance - distance;
        } else {
            delta = distance - previousDistance;
        }

        if (speedCommand == 0) {
            angleError = 0;
        }

        /*
         * If the wheel has to move over 45 degrees
         * go opposite to command and reverse translation
         */
        if (Math.abs(angleError) > TICKS_PER_REVOLUTION/8.0) {
            this.angleCommand = MathUtils.reverseWheelDirection(this.angleCommand);
            this.speedCommand *= -1;
            angleError = MathUtils.reverseErrorDirection(angleError);

            wheelReversed = true;
        } else {
            wheelReversed = false;
        }

        /*
         * If wheel direction has to change more than 22.5 degrees
         * then set wheel speed command to 0 while wheel is turning.
         */
        if (Math.abs(angleError) > TICKS_PER_REVOLUTION/16) {
            speedCommand = 0.0;
        }

        /*
         * If wheel is not translating, don't rotate
         */
        if (Math.abs(this.speedCommand) >= 0.1) {
            anglePID.setSetpoint(angle);
        }

        swerveMotor.set(speedCommand, anglePID.performPID());

        // Debug
//		if (id == 1) {
//			SmartDashboard.putNumber("Error", angleError);
//			SmartDashboard.putNumber("Motor Angle", angle);
//			SmartDashboard.putNumber("Joystick Command", this.angleCommand);
//
//		}

        rightDelta = delta * Math.sin(MathUtils.resolveAngle(Math.toRadians(angle)));
        forwardDelta = delta * Math.cos(MathUtils.resolveAngle(Math.toRadians(angle)));

        previousDistance = distance;
        previousRobotDistance = SwerveDrivetrain.getRobotDistance();

    }

    public void setID(int id) {
        this.id = id;
    }

    public double getAngle() {
        return this.angle;
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
        swerveMotor.reset();
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

	public double[] getXYDist() {
		if (wheelReversed) {
			forwardSum *= -1.0;
			rightSum *= -1.0;
		}
		double[] i = {rightDelta, forwardDelta};
		return i;
	}
}
