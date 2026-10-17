package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

    // Two dead wheels read from existing drive motor ports, heading from the hub IMU
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(0.001989436789)
            .strafeTicksToInches(0.001989436789)
            .forwardPodY(0.0)
            .strafePodX(-3.0)
            .forwardEncoder_HardwareMapName("left_front")
            .strafeEncoder_HardwareMapName("left_back")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    // Builds the follower used by every OpMode
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
