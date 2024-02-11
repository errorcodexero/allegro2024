package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

//
// Baseline for the automode
//
// Shoot	    0.5
// P1	        3.16
// P2	        2.22
// Shoot        0.5
// P3	        2.28
// P4	        3.18
// Shoot	    0.5
// P5	        1.58
// Shoot	    0.5
// Total       14.42



public class Start1Shoot4AutoMode extends AllegroGameAutoMode {
    public Start1Shoot4AutoMode(AutoController ctrl) {
        super(ctrl, "Start1Shoot4") ;

        shootFirst() ;                                  // Duration 0.5, Total 0.5
        driveAndCollect("S1S4-P1");                     // Duration 3.16, Total 3.66
        driveAndShoot("S1S4-P2");                       // Duration 2.22 + 0.5, Total 6.38
        driveAndCollect("S1S4-P3");                     // Duration 2.28, Total 8.66
        driveAndShoot("S1S4-P4");                       // Duration 3.18 + 0.5, Total 12.34
        driveAndCollectAndShoot("S1S4-P5") ;            // Duration 1.58 + 0.5, Total 14.42
    }
}
