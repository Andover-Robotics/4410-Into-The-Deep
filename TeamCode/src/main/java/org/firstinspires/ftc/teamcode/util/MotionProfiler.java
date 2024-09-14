package org.firstinspires.ftc.teamcode.util;

public class  MotionProfiler {

    public MotionProfiler(double max_velocity, double max_acceleration){
        this.max_acceleration = max_acceleration;
        this.max_velocity = max_velocity;
    }
    private final double max_velocity, max_acceleration;
    private boolean isOver = true;
    private boolean isDone = false;
    private double temp_max_accel, temp_max_vel;
    private double start_pos, final_pos, distance, acceleration_dt, halfway_distance, acceleration_distance, new_max_velocity, deacceleration_dt, cruise_distance, cruise_dt, deacceleration_time, entire_dt;

    //MUST BE USED BEFORE RUNNING A NEW PATH
    public void init_new_profile(double start_pos, double final_pos){
        this.start_pos = start_pos;
        this.final_pos = final_pos;
        isOver = false;

        distance = final_pos-start_pos;

        if(distance < 0){
            temp_max_vel = -max_velocity;
            temp_max_accel = -max_acceleration;
        }else{
            temp_max_accel = max_acceleration;
            temp_max_vel = max_velocity;
        }

        // calculate the time it takes to accelerate to max velocity
        acceleration_dt = temp_max_vel / temp_max_accel;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * temp_max_accel * acceleration_dt * acceleration_dt;

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
            acceleration_dt = Math.sqrt(Math.abs(halfway_distance / (0.5 * temp_max_accel)));
        }
        acceleration_distance = 0.5 * temp_max_accel * acceleration_dt * acceleration_dt;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        new_max_velocity = temp_max_accel * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / new_max_velocity;
        deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
    }


    public double motion_profile_pos(double current_dt) {
//        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.

        if (current_dt > entire_dt) {
            isOver = true;
            isDone = true;
            return final_pos;
        }

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return start_pos + 0.5 * temp_max_accel * current_dt * current_dt;

            // if we're cruising
        else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * temp_max_accel * acceleration_dt * acceleration_dt;
            double cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return start_pos + acceleration_distance + new_max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * temp_max_accel * acceleration_dt * acceleration_dt;
            cruise_distance = new_max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return start_pos + acceleration_distance + cruise_distance + new_max_velocity * deacceleration_time - 0.5 * temp_max_accel * deacceleration_time * deacceleration_time;
        }
    }

    public double motion_profile_vel(double current_dt) {
//
        if (current_dt > entire_dt)
            return 0;

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return temp_max_accel*current_dt;

            // if we're cruising
        else if (current_dt < deacceleration_time) {
            return new_max_velocity;
        }

        // if we're decelerating
        else {
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return new_max_velocity-deacceleration_time;
        }
    }

    public double motion_profile_accel(double current_dt) {

        if (current_dt > entire_dt)
            return 0;

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return temp_max_accel;

            // if we're cruising
        else if (current_dt < deacceleration_time) {

            return 0;
        }

        // if we're decelerating
        else {
            // use the kinematic equations to calculate the instantaneous desired position
            return -temp_max_accel;
        }
    }

    public double getEntire_dt(){
        return entire_dt;
    }

    public boolean isOver(){
        return isOver;
    }

    public boolean isDone(){
        return isDone;
    }
}

