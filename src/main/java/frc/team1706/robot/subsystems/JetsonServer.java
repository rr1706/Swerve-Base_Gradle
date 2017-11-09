package frc.team1706.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.concurrent.Semaphore;

/**
 * Code to receive UDP packets from the Jetson
 */
public class JetsonServer implements Runnable {

	private DatagramSocket socket;
	private Semaphore mutex;

	private double liftxrotdata = 9999;
	private double liftdistancedata = 9999;
	private double liftskewdata = 9999;

	private double boilerxrotdata = 9999;
	private double boilerdistancedata = 9999;

	private long lastlifttime;
	private long lastboilertime;

	public JetsonServer(short port) throws IOException {
		socket = new DatagramSocket(port);
		mutex = new Semaphore(1);
	}

	@Override
	public void run() {
		while (true) {
			this.receiveMessage();
		}
	}

	protected void receiveMessage() {
		byte[] receiveData = new byte[1024];
		DatagramPacket packet = new DatagramPacket(receiveData, receiveData.length);
		try {
			socket.receive(packet);
		} catch (IOException e) {
			e.printStackTrace();
			return;
		}
		String data = new String(packet.getData());
		if (data.length() < 4) {
			System.err.println("Bad UDP message from Jetson!*");
			return;
		}
		String[] parts = data.split(" ");
		if (parts.length < 1) {
			System.err.println("Bad UDP message from Jetson!&");
			return;
		}

		try {
			mutex.acquire();
			String name = parts[0].toLowerCase().trim();
			if (name.equals("lift")) {
				liftxrotdata = Double.parseDouble(parts[1]);
				liftdistancedata = Double.parseDouble(parts[2]);
				liftskewdata = Double.parseDouble(parts[3]);
				lastlifttime = System.currentTimeMillis();
			} else if (name.equals("boiler")) {
				boilerxrotdata = Double.parseDouble(parts[1]);
				boilerdistancedata = Double.parseDouble(parts[2]);
				lastboilertime = System.currentTimeMillis();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (NumberFormatException e) {
			System.err.println("Bad UDP message from Jetson!~");
		}
		mutex.release();
	}

	public boolean isBoilerDataRecent() {
		boolean result = false;
		try {
			mutex.acquire();
			result = System.currentTimeMillis() - lastboilertime < 500;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return result;
	}

	public boolean isBoilerValidSolution() {
		boolean result = false;
		try {
			mutex.acquire();
			result = boilerxrotdata != 0;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return result;
	}

	public double getBoilerXrot() {
		double xrot = Double.NaN;
		try {
			mutex.acquire();
			xrot = boilerxrotdata;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return xrot;
	}

	public double getBoilerDistance() {
		double distance = Double.NaN;
		try {
			mutex.acquire();
			distance = boilerdistancedata;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return distance;
	}


	public boolean isLiftDataRecent() {
		boolean result = false;
		try {
			mutex.acquire();
			result = System.currentTimeMillis() - lastlifttime < 500;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return result;
	}

	public boolean isLiftValidSolution() {
		boolean result = false;
		try {
			mutex.acquire();
			result = liftxrotdata != 0;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return result;
	}

	public double getLiftXrot() {
		double xrot = Double.NaN;
		try {
			mutex.acquire();
			xrot = liftxrotdata;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return xrot;
	}

	public double getLiftDistance() {
		double distance = Double.NaN;
		try {
			mutex.acquire();
			distance = liftdistancedata;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return distance;
	}

	public double getLiftSkew() {
		double skew = Double.NaN;
		try {
			mutex.acquire();
			skew = liftskewdata;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		mutex.release();
		return skew;
	}
}
