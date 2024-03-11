package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

public class JustShootAutoMode extends AllegroGameAutoMode {

    public JustShootAutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "JustShoot", mirror, mvalue) ;

        shootFirstNote() ;        
    }
}
