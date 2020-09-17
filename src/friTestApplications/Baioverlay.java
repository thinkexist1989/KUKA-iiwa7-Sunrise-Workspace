package friTestApplications;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * Creates a FRI Session.
 */
public class Baioverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.1.100";    //"192.172.10.100";   //"172.31.1.100";   //The PC ip that run the C++ app.
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(10);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // move to start pose
        //_lbr.move(ptp(Math.toRadians(0),Math.toRadians(50.3),Math.toRadians(0),Math.toRadians(-92.8),Math.toRadians(0),Math.toRadians(-53.1),Math.toRadians(0)).setJointVelocityRel(0.2));
        //_lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0).setJointVelocityRel(0.2));
       // _lbr.move(ptp(Math.toRadians(0), .0, .0, Math.toRadians(0), .0, Math.toRadians(0), .0).setJointVelocityRel(0.2));
        //_lbr.move(ptp(getApplicationData().getFrame("/start")).setJointVelocityRel(0.2));
        // async move with overlay ...
//        _lbr.move(ptp(.0, .0, .0, .0, .0, 0.0, .0).setJointVelocityRel(0.2));
        _lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0).setJointVelocityRel(0.2));
        
       /* _lbr.moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
         
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                .setBlendingRel(0.1)
                );

        // ... blending into sync move with overlay
        _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                );*/
        
        
        
        // start PositionHold with overlay
        JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(200, 200, 200, 200, 200, 200, 200);
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

        _lbr.move(posHold.addMotionOverlay(jointOverlay));

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final Baioverlay app = new Baioverlay();
        app.runApplication();
    }

}
