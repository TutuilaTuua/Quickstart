package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // Follower tuning (mass in kg)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.0);

    // Path end tolerances
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Mecanum drivetrain motor names and directions
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("left_front")
            .leftRearMotorName("left_back")
            .rightFrontMotorName("right_front")
            .rightRearMotorName("right_back")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // Dead wheels read from existing drive motor ports
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.001989436789)
            .strafeTicksToInches(0.001989436789)
            .turnTicksToInches(0.001989436789)
            .leftPodY(5.5)
            .rightPodY(-5.5)
            .strafePodX(-3.0)
            .leftEncoder_HardwareMapName("left_front")
            .rightEncoder_HardwareMapName("right_front")
            .strafeEncoder_HardwareMapName("left_back")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    // Builds the follower used by every OpMode
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
