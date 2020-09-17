package myApplications;


import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

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
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class Led extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private MediaFlangeIOGroup _led11;

	@Override
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		_led11 = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		
	}

	@Override
	public void run() {
		//lbr_iiwa_7_R800_1.move(ptpHome());
		//int count=1;
//		while(true)
//		{
//			if(_led11.getLEDBlue())
//			{
//				_led11.setLEDBlue(false);
//			}
//			else
//				_led11.setLEDBlue(true);
//			ThreadUtil.milliSleep(500);
			
		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
		
		_led11.setLEDBlue(true);
		lbr.move(lin(getApplicationData().getFrame("/base1/P2")));
		

		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		Led app = new Led();
		app.runApplication();
	}
}
