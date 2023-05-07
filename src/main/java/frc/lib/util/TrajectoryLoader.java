// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.EnumMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


public class TrajectoryLoader {

    private static Map<Trajectories, PathPlannerTrajectory> trajectories = new EnumMap<>(Trajectories.class);

    public enum Trajectories {
        _1_Score_P1,
        _1_ScoreMovePickup_P1, 
        _2_ScoreMovePickupCharge_P1,
        _2_ScoreMoveCharge_P1,
        _3_ScoreMovePickup_P1,

        //local changes
        _1_ScoreMoveCharge_P1,
        _3_ScoreMoveCharge_P1,
        _SafeMode;

    }

    public static void loadTrajectories() {
        // loads PathPlanner paths and generates trajectories
        trajectories.put(Trajectories._1_ScoreMovePickup_P1,            PathPlanner.loadPath("SP 1- S,M", 4.0, 2.0));
        trajectories.put(Trajectories._2_ScoreMovePickupCharge_P1,           PathPlanner.loadPath("SP 2- M,P,C", 2.5, 2.0));
        trajectories.put(Trajectories._2_ScoreMoveCharge_P1, PathPlanner.loadPath("SP 2- M,C", 2.5, 2));
        trajectories.put(Trajectories._3_ScoreMovePickup_P1, PathPlanner.loadPath("SP 3- S,M", 4.0, 2.0));

        //local changes I put
        trajectories.put(Trajectories._1_ScoreMoveCharge_P1, PathPlanner.loadPath("SP 1- M,C", 3, 2));
        trajectories.put(Trajectories._3_ScoreMoveCharge_P1, PathPlanner.loadPath("SP 3- M,C", 3, 2));

        // adding paths together can be done via path1.concatenate(path2);

    }

    public static PathPlannerTrajectory getTrajectory(Trajectories t) {
        return trajectories.get(t);
    }
}
