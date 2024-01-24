package org.xero1425.simulator.models;

public interface ISimMotorController {
    public double ticksPerRev() ;
    public void run(double dt) ;
    public double voltage() ;
    public double position() ;
    public double velocity() ;
}
