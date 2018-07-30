package com._604robotics.robotnik.utils;

import jaci.pathfinder.Trajectory;

public class PathFinderUtil {

    private PathFinderUtil() {}
    
    public static double getCurvature(Trajectory.Segment prev_seg, Trajectory.Segment seg) {
        // Get component form of the velocity vectors
        double prev_vx=prev_seg.velocity*Math.cos(prev_seg.heading);
        double prev_vy=prev_seg.velocity*Math.sin(prev_seg.heading);
        double curr_vx=seg.velocity*Math.cos(seg.heading);
        double curr_vy=seg.velocity*Math.sin(seg.heading);

        // Derivative of the tangent vector with respect to time
        double tang_prime_x=(curr_vx-prev_vx)/prev_seg.dt;
        double tang_prime_y=(curr_vy-prev_vy)/prev_seg.dt;

        // Normalized normal vector
        double normal_x=-prev_vy;
        double normal_y=prev_vx;

        double normal_mag=Math.hypot(normal_x,normal_y);
        normal_x/=normal_mag;
        normal_y/=normal_mag;

        // Dot product of t' dot t_norm
        // Minus sign so that positive points right
        double curvature=-1*(normal_x*tang_prime_x+normal_y*tang_prime_y); 
        return curvature;
    }
}
