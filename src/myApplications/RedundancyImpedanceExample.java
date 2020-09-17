package myApplications;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class RedundancyImpedanceExample extends RoboticsAPIApplication {
	private LBR lbr;
	private Controller kuka_Sunrise_Cabinet_1;
	private MediaFlangeIOGroup led;

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		led = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
	}

	public void run() {
		// move to forward starting pose
		getLogger().info("Moving to start position");
		lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0).setJointVelocityRel(0.25));

		// set up impedance control
		// high translational/rotational stiffness, low null-space stiffness
		getLogger().info("Hold position in impedance control mode");
		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();

		final double stiffnessTrans = 5000.0; // N
		final double stiffnessRot = 300.0; // Nm
		final double stiffnessNull = 5.0;

		controlMode.parametrize(CartDOF.TRANSL).setStiffness(stiffnessTrans);
		controlMode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot);
		controlMode.setNullSpaceStiffness(stiffnessNull);

		// hold impedance control until dialog is closed by user
		led.setLEDBlue(true);
		final IMotionContainer motionContainer = lbr.moveAsync((new PositionHold(controlMode, -1, null)));
		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.",
				"OK");
		motionContainer.cancel();
		led.setLEDBlue(false);
		getLogger().info("App finished");
	}
}
