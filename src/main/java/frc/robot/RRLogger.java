package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RRLogger {

    private static ConcurrentLinkedQueue<String> m_PowerBuffer = new ConcurrentLinkedQueue<String>();
    private static ConcurrentLinkedQueue<String> m_DataBuffer = new ConcurrentLinkedQueue<String>();
    private static PrintWriter m_LogFile;
    private static PrintWriter m_DataLogFile;
    private static String directory = "/home/lvuser/e/";
    private static String logFileName = "power";
    private static String dataDumpFileName = "data";
    private static long startTime;

    public static void start() {
        startTime = System.nanoTime();

        File f = new File("a.csv");

//        f = new File(directory + logFileName + ".csv");
//        if (f.exists()) { // check if file exists
//
//            int i = 1;
//            File test = null;
//            do {
//
//                test = new File(directory + logFileName + "_" + i + ".csv");
//                i++;
//            } while (test.exists());
//            f.renameTo(test); // Renames log file if exists instead of rewriting
//            try {
//                f = new File(directory + logFileName + ".csv");
//                f.createNewFile();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        } else { {
//            try {
//                f.createNewFile();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }

		f = new File(directory + dataDumpFileName + ".csv");
		if (f.exists()) { // check if file exists
			System.out.println("1");
			System.out.println(f);
			int i = 1;
			File test = null;
			do {

				test = new File(directory + dataDumpFileName + "_" + i + ".csv");
				i++;
			} while (test.exists());
			f.renameTo(test); // Renames log file if exists instead of rewriting
			try {
				f = new File(directory + dataDumpFileName + ".csv");
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else {
			System.out.println("2");
			System.out.println(f);
			try {
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

//        try {
//            m_LogFile = new PrintWriter(new BufferedWriter(new FileWriter(directory + logFileName + ".csv", false)));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

		try {
			m_DataLogFile = new PrintWriter(new BufferedWriter(new FileWriter(directory + dataDumpFileName + ".csv", true)));
            System.out.println(directory + dataDumpFileName + ".csv");
        } catch (IOException e) {
			e.printStackTrace();
		}
        System.out.println("Done constructing logger");
    }

//	public void addMessage(String message) {
//
//		// String str = "Time: " + DriverStation.getInstance().getMatchTime() + ";Message: " + message;
//		String str = "Time(s): " + ((System.nanoTime() - startTime) / 1000000000.0) + " Message:" + message;
//		m_MessageBuffer.add(str);
//	}

    public static void addData(String dataType, double value) {

        String sep = ",";
        String str = dataType + sep + value + sep + ((double) (System.nanoTime() - startTime) / 1000000000.0);
        m_DataBuffer.add(str);

    }

//    public static void addPower(String dataType, double value) {
//
//        String sep = ",";
//        String str = dataType + sep + value + sep + ((double) (System.nanoTime() - startTime) / 1000000000.0);
//        m_PowerBuffer.add(str);
//
//    }

    public static void newLine() {
        m_DataBuffer.add("\n");
    }

    public static void newPowerLine() {
        m_PowerBuffer.add("\n");
    }

    public static void writeFromQueue() {

        try {

//            if (m_LogFile == null) {
//                System.out.println("logfile null");
//                return;
//            }

            if (!m_PowerBuffer.isEmpty()) {

                String s = m_PowerBuffer.toString();
                System.out.println("messagebuffer");

                m_LogFile.println(s);

            }
            if (!m_DataBuffer.isEmpty()) {

                String s = m_DataBuffer.toString();
                m_DataBuffer.clear();
//                System.out.println("databuffer");

                m_DataLogFile.println(s);

            }
        } catch (NullPointerException e) {

            e.printStackTrace();
        }
    }
}