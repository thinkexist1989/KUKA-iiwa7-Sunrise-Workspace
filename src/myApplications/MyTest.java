package myApplications;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
//import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

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
public class MyTest extends RoboticsAPIApplication {
	private LBR lbr;
	
//	private static double[] startPosition=new double[]{0,Math.toRadians(10),0,Math.toRadians(-80),0,Math.toRadians(90),0};

	@Override
	public void initialize() {
		// initialize your application here
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	@Override
	public void run() {
		// your application execution starts here
//		PTP workPos = ptp(startPosition).setJointVelocityRel(0.25);
//		lbr.move(workPos);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// Auto-generated catch block
			e.printStackTrace();
		}

		lbr.move(ptpHome().setJointVelocityRel(0.25));
		
//		getLogger().info("Moving to start position");
//		lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0).setJointVelocityRel(0.25));

		// set up impedance control
		// high translational/rotational stiffness, low null-space stiffness
		getLogger().info("Hold position in impedance control mode");
		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();

		final double stiffnessTrans = 100.0; // N
		final double stiffnessRot = 30.0; // Nm
		final double stiffnessNull = 5.0;

		controlMode.parametrize(CartDOF.TRANSL).setStiffness(stiffnessTrans);
		controlMode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot);
		controlMode.setNullSpaceStiffness(stiffnessNull);

		// hold impedance control until dialog is closed by user
//		final IMotionContainer motionContainer = lbr.moveAsync((new PositionHold(controlMode, -1, null)));
		lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0).setJointVelocityRel(0.1).setMode(controlMode));
		final IMotionContainer motionContainer = lbr.moveAsync(new PositionHold(controlMode, -1, null));
		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.",
				"OK");
		motionContainer.cancel();
		getLogger().info("App finished");
	}
}