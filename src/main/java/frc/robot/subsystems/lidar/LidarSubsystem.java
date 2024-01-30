package frc.robot.subsystems.lidar;

import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LidarSubsystem extends Subsystem {

    private long distance_;
    private NetworkTable nt_;

    public LidarSubsystem(Subsystem parent) {
        super(parent, "LidarSubsystem");

        nt_ = NetworkTableInstance.getDefault().getTable("lidar");

        distance_ = -1;

    }

    // get the distance from the lidar sensor in centimeters, -1 = no value
    public long getDistance_() {
        return distance_;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        distance_ = nt_.getEntry("distance").getInteger(-1);

    }

}
