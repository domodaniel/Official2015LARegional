/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team687.robot;


/**
 *
 * @author Wesley
 */
public class NerdyDrive {
    private static double gyroAngle = 0;
    private static double frontLt = 0, frontRt = 0, backLt = 0, backRt = 0;
    
    public static void setHeader(double h)  {
        gyroAngle = h;
    }
    
    public static void driveAlpha(double leftX, double leftY, double rightX)    {
        double gyroAngleRads = gyroAngle * Math.PI / 180;
        double desiredAngle = (Math.atan2(leftY, leftX) + 3*Math.PI/2) % (2*Math.PI);
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = normalize(unscaledJoy, true);
        double scalar = threshhold((sqr(leftY) + sqr(leftX)) / (sqr(maxJoy[0]) + sqr(maxJoy[1])));
        double rotate = rightX/2;
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = normalize(unnormalizedValues, false);
        
        frontLt = output[0];
        frontRt = output[1];
        backLt = output[2];
        backRt = output[3];
    }
    
    public static void driveRobotCentric(double leftX, double leftY, double rightX)    {
        double gyroAngleRads = 0;
        double desiredAngle = (Math.atan2(leftY, leftX) + 3*Math.PI/2) % (2*Math.PI);
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = normalize(unscaledJoy, true);
        double scalar = threshhold((sqr(leftY) + sqr(leftX)) / (sqr(maxJoy[0]) + sqr(maxJoy[1])));
        double rotate = rightX/2;
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = normalize(unnormalizedValues, false);
        
        frontLt = output[0];
        frontRt = output[1];
        backLt = output[2];
        backRt = output[3];
    }
    
    public static void driveBeta(double leftX, double leftY, double rightX, double rightY)  {
        double gyroAngleRads = gyroAngle * Math.PI / 180;
        double desiredAngle = (Math.atan2(leftY, leftX) + 3*Math.PI/2) % (2*Math.PI);
        double desiredRotateAngle = (Math.atan2(rightY, rightX) + 3*Math.PI/2) % (2*Math.PI);
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = normalize(unscaledJoy, true);
        double scalar = threshhold((sqr(leftY) + sqr(leftX)) / (sqr(maxJoy[0]) + sqr(maxJoy[1])));
        double heading = (gyroAngle+360)%360;
        NerdyDrivePID.setHeading(heading);
        double rotate = NerdyDrivePID.getPID(desiredRotateAngle*180/Math.PI);
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = normalize(unnormalizedValues, false);
        
        frontLt = output[0];
        frontRt = output[1];
        backLt = output[2];
        backRt = output[3];
    }
    
    public static double[] getDrive()   {
        double[] output = {frontLt, frontRt, backLt, backRt};
        return output;
    }
    
    public static double getFtLeft()    {
        return frontLt;
    }
    
    public static double getFtRight()   {
        return frontRt;
    }
    
    public static double getBkLeft()    {
        return backLt;
    }
    
    public static double getBkRight()   {
        return backRt;
    }
    
    private static double threshhold(double value){
        if(value > 0){
            return Math.min(value, 1);
        }else{
            return Math.max(value, -1);
        }
    }
    
    private static double[] normalize(double[] values, boolean scaleUp){
        double[] normalizedValues = new double[values.length];
        double max = Math.max(Math.abs(values[0]), Math.abs(values[1]));
        for(int i = 2; i < values.length; i++){
            max = Math.max(Math.abs(values[i]), max);
        }
        if(max < 1 && scaleUp == false) {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i];
            }
        }   else    {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i] / max;
            }
        }
        
        return normalizedValues;
    }
    
    private static double sqr(double input) {
        return input * input;
    }
}
