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
        DcMotor m1 = (DcMotor)this.hardwareMap.dcMotor.get("Back_left");
        DcMotor m2 = (DcMotor)this.hardwareMap.dcMotor.get("Front_left");
        DcMotor m3 = (DcMotor)this.hardwareMap.dcMotor.get("Front_right");
        DcMotor m4 = (DcMotor)this.hardwareMap.dcMotor.get("Back_right");
        DcMotor m5 = (DcMotor)this.hardwareMap.dcMotor.get("slide");
        Servo clawServo = (Servo)this.hardwareMap.get("claw");

        //sets the motor mapping. get should be the name form when we config the robot.
        m4.setDirection(Direction.REVERSE);
        m2.setDirection(Direction.REVERSE);
        m1.setDirection(Direction.REVERSE);
        //reverse motors until it works
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // this is the default but we should still put this
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // So we don't get an exception
        imu.initialize(parameters);


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = (-gamepad1.left_stick_y)/1.5; //remember to revers it
            double x = (gamepad1.left_stick_x * 1.1)/1.5; //gives us about equal speed when strafing
            double rx = (gamepad1.right_stick_x)/1.5; // slightly slower speed when rotating

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
            //sometioms a motor can get a vlaue of 2 when this happens it will dived the other motors by 2 so it beheaves correctly 
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + (rotX * 1.5) + rx) / denominator);
            double backLeftPower = ((rotY - (rotX * 1.1)  + rx) / denominator);
            double frontRightPower = ((rotY - (rotX * 1.1) - rx) / denominator);
            double backRightPower = ((rotY + rotX - rx) / denominator);
            double slidePower = gamepad1.left_trigger-gamepad1.right_trigger;
            if (gamepad1.x) clawServo.setPosition(0);
            if (gamepad1.y) clawServo.setPosition(.16);


            m2.setPower(frontLeftPower);
            m1.setPower(backLeftPower);
            m3.setPower(frontRightPower);
            m4.setPower(backRightPower);
            m5.setPower(slidePower);



        }
    }
}


