package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.Gyro;
import frc.robot.subsystems.utils.SwerveModule;

import java.util.Arrays;

import static frc.robot.Constants.SwerveChassisConstants.*;

public class Chassis extends SubsystemBase {
    private final SwerveModule[] modules;
    private final Gyro gyro;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    public Chassis() {
        modules = new SwerveModule[] {
                new SwerveModule(), // front left
                new SwerveModule(), // front right
                new SwerveModule(), // back left
                new SwerveModule() // back right
        };
        gyro = new Gyro();

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
        field = new Field2d();
        SmartDashboard.putData(field);
    }

    public void stop() {
        Arrays.stream(modules).forEach(SwerveModule::stop);
    }

    public void setVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, getAngle());
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(newSpeeds);

        for (int i = 0; i < 4; i++) modules[i].setState(states[i]);
        gyro.setVelocity(Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    public Translation2d getVelocity() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            states[i] = new SwerveModuleState(modules[i].getVelocity(), modules[i].getAngle());

        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(states);
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
          res[i] = modules[i].getState();
        }
        return res;
      }

    @Override
    public void periodic() {
        Arrays.stream(modules).forEach(SwerveModule::update);
        gyro.update();

        poseEstimator.update(getAngle(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
}
