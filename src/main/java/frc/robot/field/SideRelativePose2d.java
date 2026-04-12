package frc.robot.field;

import static edu.wpi.first.units.Units.Meters;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.auto.Autos;
import frc.robot.field.FieldDimensions.Side;

public class SideRelativePose2d extends AllianceRelativePose2d {
    public SideRelativePose2d() {
        super();
    }

    @JsonCreator
    public SideRelativePose2d(
        @JsonProperty(required = true, value = "translation") Translation2d translation,
        @JsonProperty(required = true, value = "rotation") Rotation2d rotation
    ) {
        super(translation, rotation);
    }

    public SideRelativePose2d(double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    public SideRelativePose2d(Distance x, Distance y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    public SideRelativePose2d(Matrix<N3, N3> matrix) {
        super(matrix);
    }

    public SideRelativePose2d(AllianceRelativePose2d pose) {
        super(pose.getRawTranslation(), pose.getRawRotation());
    }

    public SideRelativePose2d(Pose2d pose) {
        super(pose.getTranslation(), pose.getRotation());
    }

    @Override
    public SideRelativePose2d plus(Transform2d other) {
        return transformBy(other);
    }

    @Override
    public Translation2d getTranslation() {
        return new Translation2d(getX(), getY());
    }

    @Override
    public double getX() {
        return switch (Robot.getAlliance()) {
            case Blue -> getRawX();
            case Red -> FieldDimensions.FIELD_LENGTH.in(Meters) - getRawX();
        };
    }

    @Override
    public double getY() {
        Alliance alliance = Robot.getAlliance();
        Side side = Autos.side;

        if (alliance == Alliance.Blue && side == Side.Outpost
            || alliance == Alliance.Red && side == Side.Depot) {
            return getRawY();
        } else {
            return FieldDimensions.FIELD_WIDTH.in(Meters) - getRawY();
        }
    }

    @Override
    public Distance getMeasureX() {
        return switch (Robot.getAlliance()) {
            case Blue -> getRawMeasureX();
            case Red -> FieldDimensions.FIELD_LENGTH.minus(getRawMeasureX());
        };
    }

    @Override
    public Distance getMeasureY() {
        Alliance alliance = Robot.getAlliance();
        Side side = Autos.side;

        if (alliance == Alliance.Blue && side == Side.Outpost
            || alliance == Alliance.Red && side == Side.Depot) {
            return getRawMeasureY();
        } else {
            return FieldDimensions.FIELD_WIDTH.minus(getRawMeasureY());
        }
    }

    @Override
    @JsonProperty
    public Rotation2d getRotation() {
        Rotation2d result = switch (Robot.getAlliance()) {
            case Blue -> getRawRotation();
            case Red -> getRawRotation().rotateBy(Rotation2d.kPi);
        };

        if (Autos.side == Side.Depot) {
            result = result.unaryMinus().rotateBy(Rotation2d.kPi);
        }

        return result;
    }

    @Override
    public SideRelativePose2d times(double scalar) {
        return new SideRelativePose2d(getRawTranslation().times(scalar), getRawRotation().times(scalar));
    }

    @Override
    public SideRelativePose2d rotateBy(Rotation2d other) {
        return new SideRelativePose2d(getRawTranslation().rotateBy(other), getRawRotation().rotateBy(other));
    }

    @Override
    public SideRelativePose2d transformBy(Transform2d other) {
        return new SideRelativePose2d(
            getRawTranslation().plus(other.getTranslation().rotateBy(getRawRotation())),
            other.getRotation().rotateBy(getRawRotation())
        );
    }

    @Override
    public SideRelativePose2d relativeTo(Pose2d other) {
        Transform2d transform = new Transform2d(other, this);
        return new SideRelativePose2d(transform.getTranslation(), transform.getRotation());
    }

    @Override
    public SideRelativePose2d rotateAround(Translation2d point, Rotation2d rotation) {
        return new SideRelativePose2d(getRawTranslation().rotateAround(point, rotation), getRawRotation().rotateBy(rotation));
    }

    @Override
    public SideRelativePose2d exp(Twist2d twist) {
        double sinTheta = Math.sin(twist.dtheta);
        double cosTheta = Math.cos(twist.dtheta);
        double s;
        double c;

        if (MathUtil.isNear(0.0, twist.dtheta, 1e-9)) {
            s = 1.0 - 1.0 / 6.0 * twist.dtheta * twist.dtheta;
            c = 0.5 * twist.dtheta;
        } else {
            s = sinTheta / twist.dtheta;
            c = (1 - cosTheta) / twist.dtheta;
        }

        Transform2d transform = new Transform2d(
            new Translation2d(twist.dx * s - twist.dy * c, twist.dx * c + twist.dy * s),
            new Rotation2d(cosTheta, sinTheta)
        );

        return plus(transform);
    }

    @Override
    public Matrix<N3, N3> toMatrix() {
        Vector<N2> vector = getTranslation().toVector();
        Matrix<N2, N2> matrix = getRotation().toMatrix();

        return MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            matrix.get(0, 0),
            matrix.get(0, 1),
            vector.get(0),
            matrix.get(1, 0),
            matrix.get(1, 1),
            vector.get(0),
            0.0,
            0.0,
            1.0
        );
    }

    public SideRelativePose2d nearestSideRelative(Collection<SideRelativePose2d> poses) {
        return Collections.min(
            poses,
            Comparator.comparing((SideRelativePose2d other) -> getTranslation().getDistance(other.getTranslation()))
                .thenComparing((SideRelativePose2d other) -> getRotation().minus(other.getRotation()).getRadians())
        );
    }

    @Override
    public String toString() {
        return String.format("SideRelativePose2d(%s, %s)", getTranslation(), getRotation());
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof SideRelativePose2d pose
            && getRawTranslation().equals(pose.getRawTranslation())
            && getRawRotation().equals(pose.getRawRotation());
    }

    @Override
    public SideRelativePose2d interpolate(Pose2d end, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return new SideRelativePose2d(end);
        } else {
            Twist2d twist = log(end);
            Twist2d scaledTwist = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
            return exp(scaledTwist);
        }
    }
}
