package frc.robot.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Powercell {
    boolean wasShot;
    Pose2d ballPose = new Pose2d();
    double m_lastTimestamp;
    private int ballState = 0;


    final String m_name;

    public Powercell(String name) {
        m_name = name;
    }

    public String getName() {
        return m_name;
    }

    public int getBallState() {
        return ballState;
    }

    public boolean getBallShotState() {
        return wasShot;
    }

    public void setBallState(int state) {
        ballState = state;
    }

    public void setBallShotState(boolean shotState) {
        wasShot = shotState;
    }

    public void setBallPose(Pose2d pose) {
        ballPose = pose;
    }

    public Pose2d getBallPose() {
        return ballPose;
    }

    public void setLastTimestamp(double timestamp) {
        m_lastTimestamp = timestamp;
    }

    public double getLastTimestamp() {
        return m_lastTimestamp;
    }

    public void simulationPeriodic() {
//        updateBallState();
    }
}