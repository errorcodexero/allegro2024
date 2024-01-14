package frc.robot.automodes;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class AllegroRobotAutoController extends AutoController {

    // private AllegroTestAutoMode test_mode_;

    public AllegroRobotAutoController(XeroRobot robot)
            throws MissingParameterException, BadParameterTypeException {
        super(robot, "AllegroRobotAutoController");

        try {
            // test_mode_ = new AllegroTestAutoMode(this);

        } catch(Exception e) {
            MessageLogger logger = robot.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Exception thrown creating automodes - ") ;
            logger.add(e.getMessage()).endMessage();                                             
            robot.logStackTrace(e.getStackTrace());                                              
        }
    }
    
}
