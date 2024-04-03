package frc.robot.automodes;

import java.util.List;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllegroRobotAutoController extends AutoController {

    private AllegroTestAutoMode test_mode_;
    private Alliance prev_alliance_ ;
    
    public AllegroRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException {
        super(robot, "AllegroRobotAutoController");
    }

    private String allianceString(Alliance a) {
        return (a == null) ? "null" : a.toString() ;
    }

    @Override
    public void updateAutoMode(int mode, String gamedata, Alliance alliance) {
        MessageLogger logger = getRobot().getMessageLogger() ;
        AutoMode modeobj = null ;

        if (alliance != null && alliance != prev_alliance_) {

            logger.startMessage(MessageType.Info) ;
            logger.add("alliance changed:" + allianceString(prev_alliance_) + " -> " + allianceString(alliance)) ;
            logger.add(": recreated auto modes for new alliance value").endMessage();

            prev_alliance_ = alliance ;            

            try {
                //
                // The alliance has changed, clear the automodes and add the new ones
                // based on the new alliance.
                //
                clearAutomodes() ;
                
                boolean mirror = false ;
                double mvalue = 0.0 ;

                if (alliance == Alliance.Red) {
                    mirror = true ;
                    mvalue = getRobot().getFieldSize().getX() ;
                }

                test_mode_ = new AllegroTestAutoMode(this);

                addAutoMode(new Start2Shoot4DynamicAutoMode(this, mirror, mvalue)) ;   
                addAutoMode(new Start3Shoot2AutoMode(this, 0, mirror, mvalue));
                addAutoMode(new Start3Shoot2AutoMode(this, 1, mirror, mvalue));                
                addAutoMode(new JustShootAutoMode(this, "subwoofer-center")) ;     
                addAutoMode(new NothingAutoMode(this, mirror, mvalue)) ;                            
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error).add("Exception thrown creating automodes - ") ;
                logger.add(ex.getMessage()).endMessage();                                             
                getRobot().logStackTrace(ex.getStackTrace());
            }
        }

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
