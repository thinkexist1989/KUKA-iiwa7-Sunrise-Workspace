package myApplications;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.Socket;

public class Handler extends Thread {
	Socket sock;
	
	public Handler(Socket sock) {
		this.sock = sock;
	}
	
	@Override
	public void run() {
        try {
        	InputStream input = this.sock.getInputStream();
        	OutputStream output = this.sock.getOutputStream();
//                handle(input, output);
        	BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(output));
        	BufferedReader reader = new BufferedReader(new InputStreamReader(input));
        	
        	while(true) {
        		String s = reader.readLine();
        		writer.write("IIWA received msg: " + s + "\n");
        		writer.flush();
        	}
        	
        } catch (Exception e) {
            try {
                this.sock.close();
            } catch (IOException ioe) {
            	
            }
//            getLogger().info("client disconnected.");
        }
	}
}
