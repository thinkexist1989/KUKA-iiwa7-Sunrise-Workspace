package myApplications;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class TcpServer extends RoboticsAPIBackgroundTask {
	Controller kUKA_Sunrise_Cabinet_1;
	
	ServerSocket ss = null;

	@Override
	public void initialize() {
		// initialize your task here
		kUKA_Sunrise_Cabinet_1 = (Controller) getContext().getControllers().toArray()[0];
		try {
			ss = new ServerSocket(30002);	
			getLogger().info("Server is running on 30002 port");
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
//		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,
//				CycleBehavior.BestEffort);
	}
	
	@Override
	public void run() {
		// your task execution starts here
		try {
			while(true) {
				Socket socket = ss.accept();
				getLogger().info("connect from" + socket.getRemoteSocketAddress());
				
				Thread t = new Handler(socket);
				t.start();
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}