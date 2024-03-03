package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

public class Start2Shoot2AutoMode extends AllegroGameAutoMode {
    public Start2Shoot2AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "Start2Shoot2", mirror, mvalue) ;
        shootFirstNote() ;  
        driveAndCollectClose("S2S2-P1", true, 2.0) ;
        shootFirstNote();
    }
}
