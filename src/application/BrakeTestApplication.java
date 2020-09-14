package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.brakeTest.BrakeTestResult;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.BrakeTest;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.sensorModel.TorqueEvaluator;
import com.kuka.roboticsAPI.sensorModel.TorqueStatistic;

/**
 * This application can be used as template for the LBR iiwa brake test.
 */
public class BrakeTestApplication extends RoboticsAPIApplication {
    private Controller kukaController;
    private LBR lbr_iiwa;
    private static final double relVelocity = 0.2;              // relative velocity
    private final static int[] axes = { 0, 1, 2, 3, 4, 5, 6 };  // axes to be tested

    public void initialize() {
        kukaController = (Controller) getContext().getControllers().toArray()[0];
        lbr_iiwa = (LBR) kukaController.getDevices().toArray()[0];
    }

    public void run() {
        // initial position
        getLogger().info("Moving into initial position.");
        lbr_iiwa.move(ptpHome());

        // start monitoring for maximum torques
        getLogger().info("Start evaluation of torque statistic.");
        TorqueEvaluator evaluator = new TorqueEvaluator(lbr_iiwa);

        // select static model torque values
        evaluator.setTorqueMeasured(false);

        evaluator.startEvaluation();

        getLogger().info("Perform desired robot motion.");
        lbr_iiwa.move(new PTP(new JointPosition( 0.5,  0.8,  0.2,  1.0, -0.5, -0.5, -1.5)).setJointVelocityRel(relVelocity));
        lbr_iiwa.move(new PTP(new JointPosition(-0.5, -0.8, -0.2, -1.0,  0.5,  0.5,  1.5)).setJointVelocityRel(relVelocity));

        // stop monitoring for maximum torques and store results
        getLogger().info("Stop evaluation of torque statistic.");
        TorqueStatistic maxTorqueData = evaluator.stopEvaluation();
        getLogger().info("The result of evaluation:\n" + maxTorqueData.toString());

        // start brake test
        getLogger().info("Executing brake test.");

        boolean allAxesOk = true;
        for (int axis : axes) {
            try {
                BrakeTest brakeTest = new BrakeTest(axis, maxTorqueData.getMaxAbsTorqueValues()[axis]);
                
                IMotionContainer motionContainer = lbr_iiwa.move(brakeTest);
                BrakeTestResult brakeTestResult = BrakeTest.evaluateResult(motionContainer);

                // Untested
                switch(brakeTestResult.getState().getLogLevel())
                {
                case Info:
                    getLogger().info(brakeTestResult.toString());
                    break;
                case Warning:
                    getLogger().warn(brakeTestResult.toString());
                    break;
                case Error:
                    getLogger().error(brakeTestResult.toString());
                    allAxesOk = false;
                    break;
                default:
                    break;
                }

            } catch (CommandInvalidException ex) {
                getLogger().error(
                        "Brake test for axis A" + (axis + 1)
                        + " aborted because of \"" + ex.getMessage()
                        + "\"");
                allAxesOk = false;
            }
        }
        
        if (allAxesOk)
        {
            getLogger().info("Brake test was successful for all axes.");
        }
        else
        {
            getLogger().error("Brake test failed for at least one axis.");
        }
    }

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     */
    public static void main(String[] args) {
        BrakeTestApplication app = new BrakeTestApplication();
        app.runApplication();
    }
}
