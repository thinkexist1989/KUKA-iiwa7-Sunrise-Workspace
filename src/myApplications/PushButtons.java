package myApplications;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
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
public class PushButtons extends RoboticsAPIApplication {
	private LBR lbr;

	@Override
	public void initialize() {
		// initialize your application here
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	@Override
	public void run() {
		// your application execution starts here
		lbr.move(ptpHome().setJointVelocityRel(0.25));
//		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to run this app", "OK");
		getLogger().info("App Started");
		
		pushbutton("/P3");
		
		pushbutton("/P4");
		
		pushbutton("/P5");
		
		getLogger().info("App finished");
	}

	private void pushbutton(String s) {
		lbr.move(ptp(getApplicationData().getFrame(s)).setJointVelocityRel(0.1)); //move to P4 button

//		lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0).setJointVelocityRel(0.25));
//		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
//		controlMode.parametrize(CartDOF.Z).setStiffness(2000.0);
//		controlMode.parametrize(CartDOF.Z).setAdditionalControlForce(-15.0);
//		controlMode.setNullSpaceStiffness(10.0);
		final CartesianSineImpedanceControlMode controlMode = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.Z, 15, 20);
		
		final IMotionContainer motionContainer = lbr.moveAsync(linRel(0.0, 0.0, 150).setCartVelocity(20).setMode(controlMode));
		
//		final IMotionContainer motionContainer = lbr.moveAsync(new PositionHold(controlMode, -1, null));
		

		while(Math.abs(lbr.getExternalForceTorque(lbr.getFlange()).getForce().getZ()) < 15.0);
			
		getLogger().info("force achieved");
		
		motionContainer.cancel();
		
//		lbr.move(linRel(0.0, 0.0, 150).setCartVelocity(20).setMode(controlMode));
		
		lbr.move(lin(getApplicationData().getFrame(s)).setCartVelocity(30));
	}
}