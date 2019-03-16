import java.awt.*;
import java.awt.event.ActionListener;
import java.io.*;
import java.net.*;
import java.util.*;
import javax.swing.*;

public class BroadcastGUI {

    final static String BROADCAST_ADDRESS = "255.255.255.255";
    final static int BROADCAST_PORT = 10900;

    public static void broadcast(String message) throws IOException {
        DatagramSocket socket = new DatagramSocket();
        try {
            InetAddress address = InetAddress.getByName(BROADCAST_ADDRESS);
            socket.setBroadcast(true);

            byte[] buffer = message.getBytes();

            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, BROADCAST_PORT);
            socket.send(packet);
        } finally {
            socket.close();
        }
    }

    public static void main(String args[]) {
        final String title = "Broadcasting Test";

        //
        // Create our main frame
        //
        final JFrame win = new JFrame(title) {
            private static final long serialVersionUID = 1;

            public Dimension getMinimumSize() {
                return getPreferredSize();
            }

            public Dimension getMaximumSize() {
                return getPreferredSize();
            }
        };
        win.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        Container content = win.getContentPane();
        content.setLayout(new BorderLayout());

        final JLabel label = new JLabel("Message:");
        final JTextField message = new JTextField(20);
        final JButton send = new JButton("Send");
        final ActionListener action = event -> {
            String text = message.getText();
            message.selectAll();
            // Note: we'll tack a null character on at the end as a marker
            // to make it easier for any C/C++ code.
            text += '\000';
            try {
                broadcast(text);
            } catch (Exception e) {
                JOptionPane.showMessageDialog(win, e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        };
        send.addActionListener(action);
        message.addActionListener(action);

        content.add(label, BorderLayout.WEST);
        content.add(message, BorderLayout.CENTER);
        content.add(send, BorderLayout.EAST);

        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                win.pack();

                // Make the window show up centered on the (primary) monitor.
                Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
                win.setLocationRelativeTo(null);
                win.setLocation(dim.width / 2 - win.getSize().width / 2, dim.height / 2 - win.getSize().height / 2);
                win.setVisible(true);
            }
        });
    }
}
