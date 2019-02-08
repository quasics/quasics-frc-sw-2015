import java.awt.*;
import java.io.*;
import java.net.*;
import java.util.*;
import javax.swing.*;

public class BroadcastGUI {

	final static String BROADCAST_ADDRESS = "0.0.0.0";
	final static int BROADCAST_PORT = 10900;

    public static void broadcast(String message) throws IOException {
        DatagramSocket socket = new DatagramSocket();
		try {
			InetAddress address = InetAddress.getByName("0.0.0.0");
            socket.setBroadcast(true);
 
            byte[] buffer = message.getBytes();
 
            DatagramPacket packet 
              = new DatagramPacket(buffer, buffer.length, address, BROADCAST_PORT);
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
        final JFrame win = new JFrame( title ) {
			private static final long serialVersionUID = 1;
		    public Dimension getMinimumSize() { return getPreferredSize(); }
		    public Dimension getMaximumSize() { return getPreferredSize(); }
		};
		win.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		Container content = win.getContentPane();
		content.setLayout(new BorderLayout());

		final JLabel label = new JLabel("Message:");
		final JTextField message = new JTextField(20);
		final JButton send = new JButton("Send");
		send.addActionListener(event -> {
			String text = message.getText();
			// Note: we'll tack a null character on at the end as a marker
			// to make it easier for any C/C++ code.
			text += '\000';
			try {
				broadcast(text);
			} catch(Exception e) {
				JOptionPane.showMessageDialog(
					win, e.getMessage(),
					"Error", JOptionPane.ERROR_MESSAGE);
			}
		});

	    content.add(label, BorderLayout.WEST);
		content.add(message, BorderLayout.CENTER);
		content.add(send, BorderLayout.EAST);

		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				win.pack();
				win.setVisible(true);
			}
		});
	}
}
