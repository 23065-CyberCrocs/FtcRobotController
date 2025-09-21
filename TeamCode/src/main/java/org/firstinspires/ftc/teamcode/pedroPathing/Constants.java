package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
           /* .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true) add these before manual tuning to enable dual PID
            .useSecondaryDrivePIDF(true)*/;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("Mec fl")
            .rightRearMotorName("Mec fr")
            .leftRearMotorName("Mec rl")
            .leftFrontMotorName("Mec rr")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            // add .xVelocity(velocity) after completing Forward Velocity Tuner
            // add .yVelocity(velocity) after completing Lateral Velocity Tuner
            // add .forwardZeroPowerAcceleration(deceleration) after completing Forward Zero Power Acceleration Tuner (Adjust space constraint to Forward Velocity)
            // add .lateralZeroPowerAcceleration(deceleration) after completing Lateral Zero Power Acceleration Tuner (adjust space constraint to Lateral Velocity)

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(4096/(Math.PI*35)) // Can possibly be 4096 instead
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}