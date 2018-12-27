package frc.team1706.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * @// FIXME: 12/27/2018 Just remake this entirely
 */
public class RRLogger {

	private ConcurrentLinkedQueue<String> m_PowerBuffer = new ConcurrentLinkedQueue<String>();
	private ConcurrentLinkedQueue<String> m_DataBuffer = new ConcurrentLinkedQueue<String>();
	private PrintWriter m_LogFile;
	private PrintWriter m_DataLogFile;
	private String directory = "/home/lvuser/logs/";
	private String logFileName = "power";
	private String dataDumpFileName = "data";
	private long startTime;

	public void start() {
		startTime = System.nanoTime();
		File f = new File(directory + logFileName + ".csv");
		if (f.exists() && !f.isDirectory()) { // check if file exists

			int i = 1;
			File test = null;
			do {

				test = new File(directory + logFileName + "_" + i + ".csv");
				i++;
			} while (test.exists() && !test.isDirectory());
				f.renameTo(test); // Renames log file if exists instead of rewriting
			try {
				f = new File(directory + logFileName + ".csv");
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else {
			try {
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}


		try {
			m_LogFile = new PrintWriter(new BufferedWriter(new FileWriter(directory + logFileName + ".csv", false)));
		} catch (IOException e) {
			e.printStackTrace();
		}

//		try {
//			m_DataLogFile = new PrintWriter(new BufferedWriter(new FileWriter(directory + dataDumpFileName + ".csv", true)));
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
		System.out.println("Done constructing logger");
	}

//	public void addMessage(String message) {
//
//		// String str = "Time: " + DriverStation.getInstance().getMatchTime() + ";Message: " + message;
//		String str = "Time(s): " + ((System.nanoTime() - startTime) / 1000000000.0) + " Message:" + message;
//		m_MessageBuffer.add(str);
//	}

	public void addData(String dataType, double value, String additionalComment) {

		if (additionalComment == null) {
			additionalComment = new String();
		}
		String sep = ",";
		String str = dataType + sep + value + sep + ((double) (System.nanoTime() - startTime) / 1000000000.0) + sep + additionalComment;
		m_DataBuffer.add(str);

	}

	public void addPower(String dataType, double value) {

		String sep = ",";
		String str = dataType + sep + value + sep + ((double) (System.nanoTime() - startTime) / 1000000000.0);
		m_PowerBuffer.add(str);

	}

	public void newLine() {
		m_DataBuffer.add("\n");
	}

	public void newPowerLine() {
		m_PowerBuffer.add("\n");
	}

	public void writeFromQueue() {

		try {

			if (m_LogFile == null) {
				System.out.println("logfile null");
				return;
			}
			if (!m_PowerBuffer.isEmpty()) {

				String s = m_PowerBuffer.toString();
				System.out.println("messagebuffer");

				m_LogFile.println(s);

			}
			if (!m_DataBuffer.isEmpty()) {

				String s = m_DataBuffer.toString();
				System.out.println("databuffer");

				m_DataLogFile.println(s);

			}
		} catch (NullPointerException e) {

			e.printStackTrace();
		}
	}
}
