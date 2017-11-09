package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick {
	private Joystick stick;

	public XboxController(int port) {
		super(port);
		stick = new Joystick(port);
	}

	public boolean A() {
		return stick.getRawButton(1);
	}

	public boolean B() {
		return stick.getRawButton(2);
	}

	public boolean X() {
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

	public boolean Back() {
		return stick.getRawButton(7);
	}

	public boolean Start() {
		return stick.getRawButton(8);
	}

	public boolean LStickButton() {
		return stick.getRawButton(9);
	}

	public boolean RStickButton() {
		return stick.getRawButton(10);
	}

	public double LStickX() {
		return stick.getRawAxis(0);
	}

	public double LStickY() {
		return stick.getRawAxis(1);
	}

	public double LTrig() {
		return stick.getRawAxis(2);
	}

	public double RTrig() {
		return stick.getRawAxis(3);
	}

	public double RStickX() {
		return stick.getRawAxis(4);
	}

	public double RStickY() {
		return stick.getRawAxis(5);
	}

	public int DPad() {
		return stick.getPOV();
	}

	public void rumble() {
		stick.setRumble(RumbleType.kRightRumble, 1);
		stick.setRumble(RumbleType.kLeftRumble, 1);
	}

	public void stopRumble() {
		stick.setRumble(RumbleType.kRightRumble, 0);
		stick.setRumble(RumbleType.kLeftRumble, 0);
	}
}
