package myApplications;


import java.util.Random;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RotateAroundOnePoint extends RoboticsAPIApplication {
	private Controller cabinet;
	private LBR lbr;
	private MediaFlangeIOGroup led;
	
	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6, Math.toRadians(90)};

	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet,
				"LBR_iiwa_7_R800_1");
		led = new MediaFlangeIOGroup(cabinet);
	}

	public void run() {
		//Move to special point
		lbr.move(ptp(loopCenterPosition).setJointVelocityRel(0.25));
		
		Random r = new Random();
		
		boolean b = true;
		
		while(true)
		{
//			double alpha = r.nextDouble()*2; // 生成[0,1.0]区间的小数
			
			Frame pos = lbr.getCurrentCartesianPosition(lbr.getFlange());
			
			pos.setBetaRad(r.nextDouble()/2.0);
//			pos.setGammaRad(r.nextDouble()/2.0); //在这个位形下无法绕x轴转动
			
			lbr.move(lin(pos).setJointVelocityRel(0.25));
			led.setLEDBlue(!b);
		}
		
		
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		RotateAroundOnePoint app = new RotateAroundOnePoint();
		app.runApplication();
	}
}
