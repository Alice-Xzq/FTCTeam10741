package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.*;

public class DriveWithEncoder {

    private ElapsedTime runtime = new ElapsedTime();

    /////////// All configuration parameters originally here are already moved to Parameters.java //////////////
    Hardware robot = new Hardware();
    DcMotor leftFront = robot.leftFront;
    DcMotor rightFront = robot.rightFront;
    DcMotor leftBack = robot.leftBack;
    DcMotor rightBack = robot.rightBack;
    public static double XY_CORRECTION = 1.2;
    public static int COUNTS_PER_MOTOR_REV = 1440;
    public static double DRIVE_GEAR_REDUCTION = 1.0;
    public static double WHEEL_DIAMETER_MM = 100.0;
    public static double WHEEL_DIAGONAL_DISTANCE = 464.2;
    public static double DEGREE_CORRECTION = 1.543;
    public static int COUNTS_PER_MM = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415));
    public static int           COUNTS_PER_DEGREE        = (int) (WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  *
            COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION );
    public static double MIN_DRIVE_SPEED = 0.2;
    public static double MAX_DRIVE_SPEED = 0.9;
    public static int COUNTS_THRESHOLD_FOR_SLOWDOWN = 50;
    public static double DIAGONAL_CORRECTION = 1.0;

    public void init() {

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        setPowerToAllDriveMotors(0);

    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double xMillimeters, double yMillimeters, double rAnglesInDegree,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int changeLF,changeRF,changeLB,changeRB;
        int maxDelta;

        // Determine new target position, and pass to motor controller
        changeLF =  (int)((-yMillimeters-xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  +rAnglesInDegree * COUNTS_PER_DEGREE);
        changeRF =  (int)((-yMillimeters+xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  -rAnglesInDegree * COUNTS_PER_DEGREE);
        changeLB =  (int)((-yMillimeters+xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  +rAnglesInDegree * COUNTS_PER_DEGREE);
        changeRB =  (int)((-yMillimeters-xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  -rAnglesInDegree * COUNTS_PER_DEGREE);
        maxDelta = max(max(abs(changeLB), abs(changeLF)), max(abs(changeRB), abs(changeRF)));

        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(Range.clip(abs(speed*changeLF/maxDelta),0,1));
        rightFront.setPower(Range.clip(abs(speed*changeRF/maxDelta),0,1));
        leftBack.setPower(Range.clip(abs(speed*changeLB/maxDelta),0,1));
        rightBack.setPower(Range.clip(abs(speed*changeRB/maxDelta),0,1));

        TelemetryWrapper.setLine(0,  "Running to (x,y,r)=("+xMillimeters+":"+yMillimeters +":"+rAnglesInDegree+")");
        TelemetryWrapper.setLine(1,  "Wheels to (lf,rf,lr,rr) ("+newLeftFrontTarget+":"+newRightFrontTarget +":"+newLeftBackTarget+":"+newRightBackTarget+")");
        while (
                (runtime.seconds()< timeoutS) ||
                        (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

            // Display it for the driver.
            TelemetryWrapper.setLine(2,  "Running at ("+leftFront.getCurrentPosition()+":"+rightFront.getCurrentPosition()
                    +":"+leftBack.getCurrentPosition()+":"+rightBack.getCurrentPosition()+")");
        }

        // Stop all motion;
        //stop();
        setPowerToAllDriveMotors(0);

        // Turn off RUN_TO_POSITION
        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void move(double powerx, double powery, double turn){
        double speedx = powerx;
        double speedy = powery;
        double offset = turn;
        leftFront.setPower(Range.clip(speedy-speedx+offset,-1,1));
        rightFront.setPower(Range.clip(speedy+speedx-offset,-1,1));
        leftBack.setPower(Range.clip(speedy+speedx+offset,-1,1));
        rightBack.setPower(Range.clip(speedy-speedx-offset,-1,1));
    }

    public void stop(){ setPowerToAllDriveMotors(0); }

    public void stopEnc() {

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);
    }

    /** The following are totally new methods for encoder-driving added by J.TU 17 April, 2019 */
    public void initEnc() {

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        setModeToAllDriveMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        rightFront.setTargetPosition(rightFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
        rightBack.setTargetPosition(rightBack.getCurrentPosition());
        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);
    }

    /**
     *      speed: the power used for the driving moters
     *      dist: distance to move forward or backward in millimeters
     *              - positive for forwards
     *              - negative for backwards
     *      timeoutMS: timeout setting for the move (in milliseconds)
     */
    public void moveForthBackEnc(double dist, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int changeLF, changeRF, changeLB, changeRB;
        //int maxDelta;

        int countsToMove = (int) ( dist * COUNTS_PER_MM );
        changeLF = countsToMove;
        changeRF = countsToMove;
        changeLB = countsToMove;
        changeRB = countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed

        //double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAllDriveMotors(power);


        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                ( runtime.milliseconds() < timeout ) ) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

    }

    // Dist: positive, go forwards  (robot)
    //       negative, go backwards (robot)
    public void moveForthBackEnc2(double dist, int timeoutMS ) {

        TelemetryWrapper.setLine(0,String.format("moveForthBackEnc2(dist,timeoutMs)=(%.1f,%d)",dist,timeoutMS));
        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        // short distance motion（COUNTS_THRESHOLD_FOR_SLOWDOWN*within 2 counts), first half: startupSpeed to slowSpeed，last half: slowSpeed to 0；otherwise，power from startupSpeed accelerating into fastSpeed，
        // then，moving constantly until counts <= COUNTS_THRESHOLD_FOR_SLOWDOWN，slowing down to 0。
        double startupSpeed = 0.05;
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);
        TelemetryWrapper.setLine(1,String.format("(sS,fS,cS)=(%.1f,%.1f,%.1f)",slowSpeed,fastSpeed,leftFront.getPower()));

        double power = slowSpeed; // start from speed 0

        TelemetryWrapper.setLine(2,String.format("(calPower,curPower)=(%.1f,%.1f)",power,leftFront.getPower()));
        int startCountUp, endCountUp, startCountDown, endCountDown;
        double startPowerUp, endPowerUp, startPowerDown, endPowerDown;


        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int changeLF, changeRF, changeLB, changeRB;
        //int maxDelta;

        int countsToMove = (int) ( dist * COUNTS_PER_MM );

        TelemetryWrapper.setLine(3,String.format("(dist,COUNTS_PER_MM,counsToMove) = (%.1f,%d,%d)",dist,COUNTS_PER_MM,countsToMove));
        // Calculate the starting points and ending points for speed-up and speed-down
        if (Math.abs(countsToMove)> (2*COUNTS_THRESHOLD_FOR_SLOWDOWN) ) {
            startCountUp = leftFront.getCurrentPosition();
            endCountUp = startCountUp - COUNTS_THRESHOLD_FOR_SLOWDOWN;
            startPowerUp = startupSpeed;
            endPowerUp = fastSpeed;

            startCountDown = startCountUp + countsToMove + COUNTS_THRESHOLD_FOR_SLOWDOWN;
            endCountDown =startCountDown - COUNTS_THRESHOLD_FOR_SLOWDOWN;
            startPowerDown = fastSpeed;
            endPowerDown = startupSpeed;
            TelemetryWrapper.setLine(4,String.format("Case1:(sCup,eCup,sPup,ePup)=(%d,%d,%.1f,%.1f),(sCd,eCd,sPd,ePd)=(%d,%d,%.1f,%.1f)",
                    startCountUp,endCountUp,startPowerUp,endPowerUp,
                    startCountDown,endCountDown,startPowerDown,endPowerDown));

        } else {

            startCountUp = leftFront.getCurrentPosition();
            endCountUp = startCountUp + (int) (countsToMove/2);
            startPowerUp = startupSpeed;
            endPowerUp = slowSpeed;

            startCountDown = endCountUp;
            endCountDown =startCountDown + (int) (countsToMove/2);
            startPowerDown = slowSpeed;
            endPowerDown = startupSpeed;
            TelemetryWrapper.setLine(4,String.format("Case2:(sCup,eCup,sPup,ePup)=(%d,%d,%.1f,%.1f),(sCd,eCd,sPd,ePd)=(%d,%d,%.1f,%.1f)",
                    startCountUp,endCountUp,startPowerUp,endPowerUp,
                    startCountDown,endCountDown,startPowerDown,endPowerDown));
        }

        changeLF = countsToMove;
        changeRF = countsToMove;
        changeLB = countsToMove;
        changeRB = countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        TelemetryWrapper.setLine(5,String.format("(c1,c2,c3,c4)=(%d,%d,%d,%d),(t1,t2,t3,t4)=(%d,%d,%d,%d)",
                leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftBack.getCurrentPosition(),rightBack.getCurrentPosition(),
                leftFront.getTargetPosition(),rightFront.getTargetPosition(),leftBack.getTargetPosition(),rightBack.getTargetPosition()));

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        //power = leftBack.getPower(); // start from current Speed

        power = slowSpeed;
        //double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAllDriveMotors(power);
        TelemetryWrapper.setLine(7,String.format("Before Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));

        int curCount;
        int loops=0;
        int loops0 = 0;
        int loops1 = 0;
        int loops2 = 0;
        int loops3 = 0;
        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                ( runtime.milliseconds() < timeout ) ) {
            // waiting to finish
            loops ++;
            curCount = leftBack.getCurrentPosition();
            if (whereIsTheNumber(curCount, startCountUp, endCountUp) == -1 ) {
                loops0 ++;
                power = startupSpeed;
            }
            if ( whereIsTheNumber(curCount, startCountUp, endCountUp) == 0) {
                loops1 ++;
                power = calPower(curCount, startCountUp, endCountUp, startPowerUp, endPowerUp);
            } else if ( whereIsTheNumber(curCount,startCountDown, endCountDown) == 0 ) {
                loops2 ++;
                power = calPower(curCount, startCountDown, endCountDown, startPowerDown, endPowerDown);
            } else {
                loops3 ++;
                power = leftBack.getPower();
            }

            TelemetryWrapper.setLine(2,String.format("(calPower,curPower)=(%.1f,%.1f)",power,leftFront.getPower()));
            TelemetryWrapper.setLine(5,String.format("(c1,c2,c3,c4)=(%d,%d,%d,%d),(t1,t2,t3,t4)=(%d,%d,%d,%d)",
                    leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftBack.getCurrentPosition(),rightBack.getCurrentPosition(),
                    leftFront.getTargetPosition(),rightFront.getTargetPosition(),leftBack.getTargetPosition(),rightBack.getTargetPosition()));
            TelemetryWrapper.setLine(6,String.format("(%b, %b, %b, %b) and (Loops,0,1,2,3):(%d,%d,%d,%d,%d)",
                    leftFront.isBusy(),rightFront.isBusy(),leftBack.isBusy(),rightBack.isBusy(),
                    loops, loops0,loops1,loops2,loops3 ));

            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);


            TelemetryWrapper.setLine(7,String.format("Within Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

        TelemetryWrapper.setLine(7,String.format("Exit Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));

    }


    /**
     *      speed: the power used for the driving motors
     *      dist: distance to move left or right in millimeters
     *              - positive for right
     *              - negative for left
     *      timeoutMS: timeout setting for the move (in milliseconds)
     */
    public void moveLeftRightEnc(double dist, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /** Calculate the targets */
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int changeLF, changeRF, changeLB, changeRB;
        int countsToMove = (int) (  dist * COUNTS_PER_MM * XY_CORRECTION );
        TelemetryWrapper.setLine(8,String.format("Counts to move: %d", countsToMove));
        changeLF =   countsToMove;
        changeRF =  - countsToMove;
        changeLB = - countsToMove;
        changeRB =  countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();
        setPowerToAllDriveMotors(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish

            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

    }

    //=======================

    /** dist: positive - move to right-forward
     *        negative - move to left-backward
     * @param dist
     * @param timeoutMS
     */
    public void move_RF_LB_Enc(double dist, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /** Calculate the targets */
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int changeLF, changeRF, changeLB, changeRB;
        int countsToMove = (int) (  dist * COUNTS_PER_MM * DIAGONAL_CORRECTION );
        TelemetryWrapper.setLine(8,String.format("Counts to move: %d", countsToMove));
        changeLF =   countsToMove;
        changeRF =  - countsToMove;
        changeLB = - countsToMove;
        changeRB =  countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();

        //setPowerToAllDriveMotors(power);
        leftFront.setPower(power);
        rightFront.setPower(power * 0.005);
        leftBack.setPower(power * 0.005);
        rightBack.setPower(power);

        while ((leftFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish

            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }

            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);


            //setPowerToAllDriveMotors(power);
            leftFront.setPower(power);
            rightFront.setPower(power * 0.05);
            leftBack.setPower(power * 0.05);
            rightBack.setPower(power);
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

    }



    //=======================




    /** Spinning clock-wise or counter-clock-wise
     *      speed: power used for motors
     *      angle_in_degree: the size of the angle the robot will turn
     *                          - positve for turning clock-wise
     *                          - negative for turning counter-clock-wise
     *      timeoutMS: timeout setting for the move (in millisecond)
     */
    public void spinEnc(double angle_in_degree, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();


        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int changeLF, changeRF, changeLB, changeRB;
        //int maxDelta;

        int countsToMove = (int) (angle_in_degree * COUNTS_PER_DEGREE);
        changeLF =  countsToMove;
        changeRF = -  countsToMove;
        changeLB =  countsToMove;
        changeRB = -  countsToMove;

        newLeftFrontTarget = leftFront.getCurrentPosition() + changeLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + changeRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + changeLB;
        newRightBackTarget = rightBack.getCurrentPosition() + changeRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();
        setPowerToAllDriveMotors(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);

        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);
    }

    private void setPowerToAllDriveMotors(double powerForAll) {
        leftFront.setPower(powerForAll);
        rightFront.setPower(powerForAll);
        leftBack.setPower(powerForAll);
        rightBack.setPower(powerForAll);
    }
    private void setModeToAllDriveMotors(DcMotor.RunMode runModeForAll) {
        leftFront.setMode(runModeForAll);
        rightFront.setMode(runModeForAll);
        leftBack.setMode(runModeForAll);
        rightBack.setMode(runModeForAll);
    }
    private void setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior zeroPowerBehaviorForAll) {
        leftFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        leftBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
    }

    public double calPower(int currCount, int startCount, int endCount, double startPower, double endPower) {
        double power = 0;
        if (startPower == endPower) {
            power = startPower;
            TelemetryWrapper.setLine(9,String.format("calPower: startPower == endPower, power=%.1f",power));
        } else if (startPower < endPower) {  // 加速阶段
            if (((startCount <= endCount) /*accelerating in positive direction*/ && (currCount >= endCount) /*当前位置已经超过目标位置*/) ||
                    ((startCount >= endCount) /*accelerating in negative direction*/ && (currCount <= endCount) /*当前位置已经超过目标位置*/)) {
                power = endPower;
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower accelerating, passing the ending position, power=%.1f",power));
            } else if (((startCount <= endCount) /*accelerating in positive direction*/ && (currCount <= startCount) /*当前位置尚不到起始位置*/)
                    || ((startCount >= endCount) /*accelerating in negative direction*/ && (currCount >= startCount) /*当前位置尚不到起始位置*/)) {
                power = startPower;
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower accelerating, not yet at ending position, power=%.1f",power));
            } else { /* 正、反转时加速，当前位置在起始位置和目标位置之间 */
                power =  (startPower + (Math.sin(((currCount - startCount) / (endCount - startCount)) * Math.PI / 2)) * (endPower - startPower));
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower accelerating, between the start and end, power=%.1f",power));
            }
        } else if (endPower < startPower) {  // slowing down
            if (((startCount <= endCount) /*slowing down in positive direction*/ && (currCount >= endCount) /*当前位置已经超过目标位置*/) ||
                    ((startCount >= endCount) /*反转时减速*/ && (currCount <= endCount) /*当前位置已经超过目标位置*/)) {
                power = endPower;
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower slowing down，passing the ending position, power=%.1f",power));

            } else if (((startCount <= endCount) /*slowing down in positive direction*/ && (currCount <= startCount) /*当前位置尚不到起始位置*/)
                    || ((startCount >= endCount) /*反转时减速*/ && (currCount >= startCount) /*当前位置尚不到起始位置*/)) {
                power = startPower;
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower slowing down，not yet at ending position, power=%.1f",power));
            } else { /* 正、反转时减速，当前位置在起始位置和目标位置之间 */
                power = (startPower + (Math.sin(((currCount - startCount) / (endCount - startCount)) * Math.PI / 2)) * (endPower - startPower));
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower slowing down，between the start and end, power=%.1f",power));
            }
        }
        return power;
    }

    public int whereIsTheNumber(int i, int startInt, int endInt) {
        int result;
        if (startInt <= endInt) { // 正转
            if ( i < startInt) {
                result = -1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in clockwise (startIn <= endInt, i < startInt, result = -1)");
            } else if ( i > endInt) {
                result = 1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in clockwise (startIn <= endInt, i > endInt, result = 1)");
            } else {
                result = 0;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in clockwise (startIn <= endInt, i in between, result = 0)");
            }
        } else { // startInt > endInt 反转
            if ( i > startInt) {
                result = -1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in anti-clockwise (startIn > endInt, i > startInt, result = -1)");
            } else if ( i < endInt) {
                result = 1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in anti-clockwise (startIn > endInt, i < endInt, result = 1)");
            } else {
                result = 0;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: turning in anti-clockwise (startIn > endInt, i in between, result = 0)");
            }
        }
        return result;
    }
}