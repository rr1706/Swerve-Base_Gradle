package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechController extends Joystick {
	private Joystick stick;

	public LogitechController(int port) {
		super(port);
		stick = new Joystick(port);
	}

	public boolean X() {
		return stick.getRawButton(1);
	}

	public boolean A() {
		return stick.getRawButton(2);
	}

	public boolean B() {
		return stick.getRawButton(3);
	}

	public boolean Y() {
		return stick.getRawButton(4);
	}

	public boolean LB() {
		return stick.getRawButton(5);
	}

	public boolean RB() {
		return stick.getRawButton(6);
	}

	public boolean LTrig() {
		return stick.getRawButton(7);
	}

	public boolean RTrig() {
		return stick.getRawButton(8);
	}

	public boolean Back() {
		return stick.getRawButton(9);
	}

	public boolean Start() {
		return stick.getRawButton(10);
	}

	public boolean LStickButton() {
		return stick.getRawButton(11);
	}

	public boolean RStickButton() {
		return stick.getRawButton(12);
	}

	public double LStickX() {
		return stick.getRawAxis(0);
	}

	public double LStickY() {
		return stick.getRawAxis(1);
	}

	public double RStickX() {
		return stick.getRawAxis(2);
	}

	public double RStickY() {
		// System.out.println(stick.getRawAxis(3));
		if (Math.abs(stick.getRawAxis(3)) >= 0.1) {
			// System.out.println("1");
			return stick.getRawAxis(3);
		} else {
			// System.out.println("2");
			return 0.0;

		}
	}

	public int DPad() {
		return stick.getPOV();
	}

}
