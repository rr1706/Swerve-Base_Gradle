package frc.team1706.robot.utilities;

/**
 * Represents a vector in 2d space.
 */
public class Vector {
	private double x;
	private double y;

	/**
	 * Initialize with rectangular coordinates.
	 *
	 * @param x Vector length in the X direction.
	 * @param y Vector length in the Y direction.
	 */
	public static Vector createWithRectangularCoordinates(double x, double y) {
		return new Vector(x, y);
	}

	/**
	 * Set the vector components based on a direction and magnitude.
	 *
	 * @param dir Direction, in radians, from the positive X axis.
	 * @param mag Magnitude of the vector.
	 */
	public static Vector createWithMagnitudeAndDirection(double dir, double mag) {
		return new Vector(Math.cos(dir) * mag, Math.sin(dir) * mag);
	}

	/**
	 * Load a vector from a string.
	 *
	 * @param string String in format x,y
	 * @return vector
	 */
	public static Vector load(String string) {
		String[] argv = string.split(",");
		double x = Double.parseDouble(argv[0]);
		double y = Double.parseDouble(argv[1]);
		return new Vector(x, y);
	}

	public Vector(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Get the magnitude of the vector.
	 *
	 * @return magnitude (hypotenuse).
	 */
	public double getMagnitude() {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	/**
	 * Get the direction of the vector.
	 *
	 * @return direction in radians from the positive X axis
	 */
	public double getDirection() {
		return Math.atan2(y, x);
	}

	/**
	 * Get the X component
	 *
	 * @return x component
	 */
	public double getX() {
		return x;
	}

	/**
	 * Get the Y component
	 *
	 * @return y component
	 */
	public double getY() {
		return y;
	}

	/**
	 * Returns the sum of two vectors. Vectors can cancel.
	 *
	 * @param two Second vector.
	 * @return Sum of this and the second vector.
	 */
	public Vector add(Vector two) {
		return new Vector(this.x + two.x, this.y + two.y);
	}
}
