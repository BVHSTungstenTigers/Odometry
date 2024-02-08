package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;

public class Odometry {
    private DcMotor left;
    private DcMotor right;
    private DcMotor back;   
    private final XyhVector START_POS;
    private XyhVector currentPosition;
    public double lastTickLeft;
    public double lastTickRight;
    public double lastTickBack;
    private double angleOffset = 0;			// All measurements in inches
    private static double TAIL_LENGTH = 1.625; // Distance from center of side deadwheels and back deadwheel
    private static double WINGSPAN = 10.625; // Distance from center of side deadwheels and back deadwheel
    private static double WHEEL_RADIUS = 0.6889764; // Radius of deadwheel
    private static double CENTER_X_OFFSET = -2.75; // Distance from bot's center of rotation and midpoint between side deadwheels
    private static double TPR = 8192; // Encoder Ticks per Revolution
    private Hardware bot;
    private double imuOffset;

    public Odometry(Hardware bot, DcMotor left, DcMotor right, DcMotor back, double x, double y, double h) {
        this.left = left;
        this.right = right;
        this.back = back;
        this.bot = bot;
        imuOffset = (h/180*Math.PI - bot.getIMUAngle())/180*Math.PI;
        START_POS = new XyhVector(x, y, h/180*Math.PI);
        currentPosition = new XyhVector(START_POS);
        currentPosition.x = START_POS.x + CENTER_X_OFFSET*Math.cos(h/180*Math.PI);
        currentPosition.y = START_POS.y - CENTER_X_OFFSET*Math.sin(h/180*Math.PI);
        updateTick();
    }

    public double x() {
        return currentPosition.x - CENTER_X_OFFSET*Math.cos(currentPosition.h);
    }
    public double y() {
        return currentPosition.y - CENTER_X_OFFSET*Math.sin(currentPosition.h);
    }
    public double h() {
        return currentPosition.h*180/Math.PI;
    }

    public XyhVector getCurrentPosition() {
        return new XyhVector(x(),y(), h());
    }

    public double leftValue(){
        return left.getCurrentPosition();
    }
    public double rightValue(){
        return right.getCurrentPosition();
    }

    private void updateTick() {
        lastTickLeft = left.getCurrentPosition();
        lastTickRight = right.getCurrentPosition();
        lastTickBack = -back.getCurrentPosition();
    }

    public void setPosition(XyhVector pos) {
        imuOffset = pos.h/180*Math.PI - bot.getIMUAngle()/180*Math.PI;
        currentPosition = new XyhVector(pos);
        currentPosition.x = pos.x + CENTER_X_OFFSET*Math.cos(pos.h/180*Math.PI);
        currentPosition.y = pos.y + CENTER_X_OFFSET*Math.sin(pos.h/180*Math.PI);
        currentPosition.h = pos.h/180*Math.PI;
        updateTick();
    }

    public void setPosition(double x, double y, double h) {
        imuOffset = h/180*Math.PI - bot.getIMUAngle()/180*Math.PI;
        currentPosition = new XyhVector(x,y,h/180*Math.PI);
        currentPosition.x = x + CENTER_X_OFFSET*Math.cos(h/180*Math.PI);
        currentPosition.y = y + CENTER_X_OFFSET*Math.sin(h/180*Math.PI);
        updateTick();
    }

    public void update() {
        double tickLeft = left.getCurrentPosition();
        double tickRight = right.getCurrentPosition();
        double tickBack = -back.getCurrentPosition();
        double deltaTickLeft = tickLeft - lastTickLeft;
        double deltaTickRight = tickRight - lastTickRight;
        double deltaTickBack = tickBack - lastTickBack;


        double h = bot.getIMUAngle()/180*Math.PI + imuOffset;
        double deltaX = ((Math.PI * WHEEL_RADIUS) / TPR) * (deltaTickLeft + deltaTickRight);
        double deltaY = ((2 * Math.PI * WHEEL_RADIUS)/TPR) * ((TAIL_LENGTH / WINGSPAN) * (deltaTickRight - deltaTickLeft) + deltaTickBack);

        //double deltaTheta = ((2 * Math.PI * WHEEL_RADIUS) / (TPR * WINGSPAN)) * (deltaTickRight-deltaTickLeft);
        /*
        double deltaX = 2 * Math.PI * WHEEL_RADIUS / TPR * deltaTickRight;
        double deltaY = 0;
*/

        double x = currentPosition.x + deltaX * Math.cos(currentPosition.h) - deltaY * Math.sin(currentPosition.h);
        double y = currentPosition.y + deltaX * Math.sin(currentPosition.h) + deltaY * Math.cos(currentPosition.h);
        //double h = currentPosition.h + deltaTheta;

        currentPosition.x = x;
        currentPosition.y = y;
        currentPosition.h = h;

        lastTickLeft = tickLeft;
        lastTickRight = tickRight;
        lastTickBack = tickBack;
    }

    public void moveToPosition(XyhVector targetPosition, double driveSpeedModifier, boolean updateRotation, double GEDScale) {
        final double GOOD_ENOUGH_DISTANCE = 0.6 * GEDScale;
        final double DISTANCE_TO_SCALE = 12;

        double MIN_SPEED; //0.2 default
        {
            double voltage = bot.voltageSensor.getVoltage();
            if (voltage < 11.5) voltage = 11.5;
            else if (voltage > 14) voltage = 14;

            if (voltage <= 12) MIN_SPEED = 0.31;
            else MIN_SPEED = (0.275-0.2)/(12-14)*(voltage-14)+0.2;
        }
        final double MAX_ROTATION = 0.3;
        final double MIN_ROTATION = 0.0;
        final double SCALE_ANGLE = 35;
        final double GOOD_ENOUGH_ANGLE = 1;
        final double targetAngle = targetPosition.h;
        final double ENOUGH_TIME = 400 * 1000000;
        final int TIMEOUT_MAX = 220; // will stop after 250 loops of the while loop
        // the max should probably be based on distance
        // yes carter i know this is a sped ass solution

        int timeoutCount = 0;

        double speed;

        double dx = targetPosition.x - bot.odometry.x();
        double dy = targetPosition.y - bot.odometry.y();
        double distance = Math.hypot(dx, dy);
        double angle = Math.atan2(dx, -dy) - Math.PI / 4 - bot.odometry.h() / 180 * Math.PI;
        double deltaH = targetPosition.h - bot.odometry.h();
        double rotation = 0;
        double strafeAccounting;

        boolean isGood = false;
        long startGood = 0;

        while (!isStopRequested()) {
            if (gamepad1.a) {

                telemetry.addData("rotation", rotation);
                telemetry.addData("deltaH", deltaH);
                telemetry.addData("dx", dx);
                telemetry.addData("dy", dy);
                telemetry.addData("anglePre", Math.atan2(dx, -dy)*180/Math.PI-bot.odometry.h());
                telemetry.addData("x", bot.odometry.x());
                telemetry.addData("y", bot.odometry.y());
                telemetry.addData("h", bot.odometry.h());
                telemetry.addData("lastTickLeft", bot.odometry.lastTickLeft);
                telemetry.addData("lastTickRight", bot.odometry.lastTickRight);
                telemetry.addData("lastTickBack", bot.odometry.lastTickBack);
                telemetry.addData("leftDead", bot.leftDeadWheel.getCurrentPosition());
                telemetry.addData("rightDead", bot.rightDeadWheel.getCurrentPosition());
                telemetry.addData("backDead", bot.backDeadWheel.getCurrentPosition());
                telemetry.addData("strafeDistance", Math.atan2(dx, -dy)*180/Math.PI - bot.odometry.h());
                telemetry.addData("sensor a", bot.leftScoredPixelSensor.alpha());
                telemetry.addData("sensor r", bot.leftScoredPixelSensor.red());
                telemetry.addData("sensor g", bot.leftScoredPixelSensor.green());
                telemetry.addData("sensor b", bot.leftScoredPixelSensor.blue());
                bot.frontLeftMotor.setPower(0);
                bot.backLeftMotor.setPower(0);
                bot.frontRightMotor.setPower(0);
                bot.backRightMotor.setPower(0);

                if (bot.update(this)) break;
                continue;
            }
            timeoutCount += 1;

            if (timeoutCount > TIMEOUT_MAX) break;

            if (distance <= GOOD_ENOUGH_DISTANCE && Math.abs(deltaH) <= GOOD_ENOUGH_ANGLE) {
                if (!isGood) {
                    isGood = true;
                    startGood = System.nanoTime();
                } else if (System.nanoTime() - startGood >= ENOUGH_TIME) break;
            } else {
                if (isGood) isGood = false;
                if (startGood != 0) startGood = 0;
            }
            dx = targetPosition.x - bot.odometry.x();
            dy = targetPosition.y - bot.odometry.y();
            distance = Math.hypot(dx, dy);
            angle = Math.atan2(dx, -dy) - Math.PI / 4 - bot.odometry.h() / 180 * Math.PI;

            //telemetry.addData("strafe?", Math.abs(90 - Math.abs(Math.atan2(dx, -dy)*180/Math.PI - bot.odometry.h())));
            if (Math.abs(90 - Math.abs(Math.atan2(dx, -dy)*180/Math.PI - bot.odometry.h()+90)) <= 30) {
                strafeAccounting = 2;
                //telemetry.addLine("Accounting!");
            }
            else strafeAccounting = 1;

            deltaH = targetPosition.h - bot.odometry.h();
            if (deltaH < -180) deltaH += 360;
            else if (deltaH > 180) deltaH -= 360;

            if (distance < DISTANCE_TO_SCALE) {
                speed = (driveSpeedModifier - MIN_SPEED) * distance / DISTANCE_TO_SCALE + MIN_SPEED; //Squares so it deceases faster at the beginning but maintains small speed
            } else {
                speed = driveSpeedModifier;
            }

            speed *= strafeAccounting;

            if (speed > 1) speed = 1;

            if (updateRotation) {
                rotation = turnToNearest(bot.odometry.h(), targetPosition.h);
            }
            //telemetry.addData("rotation", rotation);

            bot.frontLeftMotor.setPower(Math.cos(angle) * speed + rotation);
            bot.backLeftMotor.setPower(Math.sin(angle) * speed + rotation);
            bot.frontRightMotor.setPower(Math.sin(angle) * speed - rotation);
            bot.backRightMotor.setPower(Math.cos(angle) * speed - rotation);

            telemetryX.setValue(bot.odometry.x());
            telemetryY.setValue(bot.odometry.y());
            telemetryH.setValue(bot.odometry.h());

            //telemetry.addData("Timeout count", timeoutCount);

            if (bot.update(this)) break;
        }

        bot.frontLeftMotor.setPower(0);
        bot.backLeftMotor.setPower(0);
        bot.frontRightMotor.setPower(0);
        bot.backRightMotor.setPower(0);

        sleep(100);

        return;

        /*while (!isStopRequested()) {
            telemetry.addData("x", bot.odometry.x());
            telemetry.addData("y", bot.odometry.y());
            telemetry.addData("h", bot.odometry.h());
            telemetry.addLine("Reached Location.");
            if (bot.update(this)) break;
            return;
        }*/
    }

    public double turnToNearest(double currentAngle, double targetAngle){ return turnToNearest(currentAngle, targetAngle, 1); }
    public double turnToNearest(double currentAngle, double targetAngle, double rSpeed) {
        double maxSpeed = 0.625;
        double minSpeed = 0.0925; //Goes linearly (rn) from max to min speed from 0 to distanceToScale as the x value and speed as the y value
        double rotationSpeed = maxSpeed * rSpeed;
        double distanceToScale = 45; //Distance to start slowing down
        double goodEnoughDistance = 0.5; //Stops turning within this angle of a cardinal direction
        double rotation;
        double distance = Math.abs(targetAngle - currentAngle);

        //Deal with -180/180 wrapping issues by adding/subtracting 360 thus that it goes the closest way to the angle
        if (distance > Math.abs(targetAngle - 360 - currentAngle)) {
            targetAngle -= 360;
            distance = Math.abs(targetAngle - currentAngle);
        } else if (distance > Math.abs(targetAngle + 360 - currentAngle)) {
            targetAngle += 360;
            distance = Math.abs(targetAngle - currentAngle);
        }

        //Sign function, if it's negative its negative direction
        double direction = Math.signum(targetAngle - currentAngle);

        if (distance <= goodEnoughDistance) {
            return 0;
        } else {
            if (distance < distanceToScale) { //Scale speed
                rotationSpeed = (maxSpeed - minSpeed) * distance / distanceToScale + minSpeed; //Squares so it deceases faster at the beginning but maintains small speed
            }
            //Set rotation value
            return -rotationSpeed * direction; //It's negative because some reason idk
        }
    }

    public void moveToPosition(double x, double y, double h, double d, boolean t) {
        moveToPosition(new XyhVector(x, y, h), d, t);
    }

    public void moveToPosition(XyhVector tp, double d, boolean t) {
        moveToPosition(tp, d, t, 1);
    }
    public void moveToPosition(double x, double y, double h, double d, boolean t, double g) {
        moveToPosition(new XyhVector(x, y, h), d, t, g);
    }

}
