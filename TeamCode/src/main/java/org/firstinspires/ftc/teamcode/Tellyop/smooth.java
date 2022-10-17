package org.firstinspires.ftc.teamcode.Tellyop;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp

public class smooth extends LinearOpMode {
    public smooth(){
    }

    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotor m1 = (DcMotor)this.hardwareMap.dcMotor.get("back_left");
        DcMotor m2 = (DcMotor)this.hardwareMap.dcMotor.get("front_left");
        DcMotor m3 = (DcMotor)this.hardwareMap.dcMotor.get("front_right");
        DcMotor m4 = (DcMotor)this.hardwareMap.dcMotor.get("back_right");

        //sets the motor mapping. get should be the name form when we config the robot.
        m4.setDirection(Direction.REVERSE);
        m2.setDirection(Direction.REVERSE);
        m3.setDirection(Direction.REVERSE);
        //reverse motors until it works
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // this is the default but we should still put this
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // So we don't get an exception
        imu.initialize(parameters);


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = (-gamepad1.left_stick_y); //remember to revers it
            double x = (gamepad1.left_stick_x * 1.1); //gives us about equal speed when strafing
            double rx = (gamepad1.right_stick_x)/1.3;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double angle2 = -imu.getAngularOrientation().firstAngle;
            if (gamepad1.right_bumper){
                angle2 = (-imu.getAngularOrientation().firstAngle)-(-imu.getAngularOrientation().firstAngle);
            }
            double botHeading = angle2;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + (rotX * 1.5) + rx) / denominator)/3;
            double backLeftPower = ((rotY - (rotX * 1.1)  + rx) / denominator)/3;
            double frontRightPower = ((rotY - (rotX * 1.1) - rx) / denominator)/3;
            double backRightPower = ((rotY + rotX - rx) / denominator)/3;

            m2.setPower(frontLeftPower);
            m1.setPower(backLeftPower);
            m3.setPower(frontRightPower);
            m4.setPower(backRightPower)



        }
    }
}


