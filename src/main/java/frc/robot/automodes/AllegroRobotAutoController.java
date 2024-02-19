package frc.robot.automodes;

import java.util.List;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class AllegroRobotAutoController extends AutoController {

    private AllegroTestAutoMode test_mode_;
    
    public AllegroRobotAutoController(XeroRobot robot)
            throws MissingParameterException, BadParameterTypeException {
        super(robot, "AllegroRobotAutoController");

        try {
            test_mode_ = new AllegroTestAutoMode(this);

            addAutoMode(new Start1Shoot4AutoMode(this));
            // addAutoMode(new Start2Shoot3AutoMode(this));
            // addAutoMode(new Start3Shoot3AutoMode(this));

        } catch(Exception e) {
            MessageLogger logger = robot.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Exception thrown creating automodes - ") ;
            logger.add(e.getMessage()).endMessage();                                             
            robot.logStackTrace(e.getStackTrace());                                              
        }
    }

    public void updateAutoMode(int mode, String gamedata) {

        AutoMode modeobj = null ;

        if (isTestMode()) {
            modeobj = test_mode_ ;
        }
        else {
            List<AutoMode> automodes = getAllAutomodes() ;
            if (mode >= 0 && mode < automodes.size()) {

                modeobj = automodes.get(mode) ;
            }
        }

        if (getAutoMode() != modeobj) {
            setAutoMode(modeobj) ;
        }
    }
}
