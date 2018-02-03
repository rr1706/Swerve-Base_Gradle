package frc.team1706.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.text.MessageFormat;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Code to send/receive UDP packets to/from the Jetson
 * @author Connor Monahan
 */
public class JetsonServer implements Runnable {

    private final short port;
    private DatagramSocket socket;
    private Lock mutex;

    private double boxxrotdata = Double.NaN;
    private double boxdistancedata = Double.NaN;
    private double boxskewdata = Double.NaN;
    private long lastboxsendtime = 0;
    private long lastboxrecvtime = 0;
    private Logger logger;

    /**
     * Create listening sockets and configures the server.
     * @param server_port Listening port on the RoboRIO. Default is 5800.
     * @param client_port Listening port on the Jetson. Default is 5801.
     * @throws IOException Socket creation error
     */
    public JetsonServer(short server_port, short client_port) throws IOException {
        this.port = client_port;
        socket = new DatagramSocket(server_port);
        mutex = new ReentrantLock();
        logger = Logger.getLogger("JetsonServer");
        socket.setBroadcast(true);
        socket.setReuseAddress(true);
        socket.setReceiveBufferSize(1024);
    }

    /**
     * Listen for new messages forever.
     */
    @Override
    public void run() {
        while (true) {
            this.receiveMessage();
        }
    }

    private void receiveMessage() {
        byte[] receiveData = new byte[1024];
        DatagramPacket packet = new DatagramPacket(receiveData, receiveData.length);
        try {
            socket.receive(packet);
        } catch (IOException e) {
            logger.logp(Level.WARNING, "JetsonServer", "receiveMessage", "Failed to recv data", e);
            return;
        }
        if (receiveData[0] != 0x17 || receiveData[1] != 0x06) {
            logger.warning("Bad magic value");
            return;
        }
        if (receiveData[2] != 1) {
            logger.warning("Bad protocol version " + (int)(receiveData[2]));
            return;
        }
        ByteBuffer wrap = ByteBuffer.wrap(receiveData, 3, 32);
        try {
            mutex.lockInterruptibly();
            boxxrotdata = wrap.getDouble(3);
            boxdistancedata = wrap.getDouble(3+8);
            boxskewdata = wrap.getDouble(3+16);
            lastboxsendtime = wrap.getLong(3+24);
            lastboxrecvtime = System.currentTimeMillis();
            logger.info(MessageFormat.format("{0} {1} {2} {3} {4} (Network delay {5}ms)",
                    boxxrotdata, boxdistancedata, boxskewdata, lastboxsendtime, lastboxrecvtime,
                    System.currentTimeMillis() - lastboxsendtime));
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "receiveMessage", "Failed to save data", e);
        } catch (IndexOutOfBoundsException e) {
            logger.warning("Bad UDP message format");
        } finally {
            mutex.unlock();
        }
    }

    /**
     * Check if new data was generated and received within the last 500 milliseconds.
     * @return true if data is recent
     */
    public boolean isBoxDataRecent() {
        boolean result = false;
        try {
            mutex.lockInterruptibly();
            result = System.currentTimeMillis() - lastboxsendtime < 500;
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "isBoxDataRecent", "Failed to read data", e);
        } finally {
            mutex.unlock();
        }
        return result;
    }

    /**
     * Check if there is a valid solution for the box (e.g. no values are NaN)
     * @return true if valid solution
     */
    public boolean isBoxValidSolution() {
        boolean result = false;
        try {
            mutex.lockInterruptibly();
            result = !Double.isNaN(boxxrotdata) && !Double.isNaN(boxdistancedata) && !Double.isNaN(boxskewdata);
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "isBoxValidSolution", "Failed to read data", e);
        } finally {
            mutex.unlock();
        }
        return result;
    }

    /**
     * Get the last received x-rotation towards the box. Units: radians
     * @return xrot
     */
    public double getBoxXrot() {
        double xrot = Double.NaN;
        try {
            mutex.lockInterruptibly();
            xrot = boxxrotdata;
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "getBoxXrot", "Failed to read data", e);
        } finally {
            mutex.unlock();
        }
        return xrot;
    }

    /**
     * Get the last received distance to the box. Units: meters
     * @return distance
     */
    public double getBoxDistance() {
        double distance = Double.NaN;
        try {
            mutex.lockInterruptibly();
            distance = boxdistancedata;
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "getBoxDistance", "Failed to read data", e);
        } finally {
            mutex.unlock();
        }
        return distance;
    }


    /**
     * Get the last returned skew (e.g. rotation of the box). Units: radians
     * @return skew
     */
    public double getBoxSkew() {
        double skew = Double.NaN;
        try {
            mutex.lockInterruptibly();
            skew = boxskewdata;
        } catch (InterruptedException e) {
            logger.logp(Level.WARNING, "JetsonServer", "getBoxSkew", "Failed to read data", e);
        } finally {
            mutex.unlock();
        }
        return skew;
    }

    /**
     * Tell the jetson that autonomous mode has started. Also synchronizes the clocks.
     */
    public void setAuto() {
        ByteBuffer buffer = ByteBuffer.allocate(12);
        buffer.put((byte) 0x17);
        buffer.put((byte) 0x07);
        buffer.put((byte) 0x01);
        buffer.put((byte) 0x01); // auto
        buffer.putLong(System.currentTimeMillis());
        sendToJetson(buffer.array());
    }

    /**
     * Tell the jetson that the robot has been disabled. Also synchronizes the clocks.
     */
    public void setDisabled() {
        ByteBuffer buffer = ByteBuffer.allocate(12);
        buffer.put((byte) 0x17);
        buffer.put((byte) 0x07);
        buffer.put((byte) 0x01);
        buffer.put((byte) 0x00); // auto
        buffer.putLong(System.currentTimeMillis());
        sendToJetson(buffer.array());
    }

    /**
     * Tell the jetson that teleoperated mode has started. Also synchronizes the clocks.
     */
    public void setTeleop() {
        ByteBuffer buffer = ByteBuffer.allocate(12);
        buffer.put((byte) 0x17);
        buffer.put((byte) 0x07);
        buffer.put((byte) 0x01);
        buffer.put((byte) 0x02); // auto
        buffer.putLong(System.currentTimeMillis());
        sendToJetson(buffer.array());
    }

    private void sendToJetson(byte[] data) {
        try {
            InetAddress address = InetAddress.getByName("255.255.255.255");
            DatagramPacket packet = new DatagramPacket(data, data.length, address, port);
            socket.send(packet);
        } catch (IOException e) {
            logger.logp(Level.WARNING, "JetsonServer", "sendToJetson", "Failed to send data to jetson", e);
        }
    }


}
