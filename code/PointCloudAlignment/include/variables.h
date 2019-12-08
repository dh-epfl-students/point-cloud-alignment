#pragma once

/// This header file aims regroup in one place every variable that can be changed manually at one point
/// in the  plane segmentation, merging or alignement processes.

///================================ PREPROCESSING STEPS: contains normal and curvature computation. =========================================///

/// Size of the leaf used when resampling cloud
#define LEAF_SIZE 1
/// Upper bound on K computation.
#define MAX_K_ORIGINAL 50
/// Upper bound on K computation if the point cloud was resampled.
#define MAX_K_RESAMPLED 15
/// Lower bound on K computation.
#define MIN_K 7
/// Maximum curvature threshold to exclude points before plane segmentation.
#define MAX_CURVATURE 0.5

///================================ PLANE SEGMENTATION CLOUD ===============================================================================///

/**
 * @brief Max angle in radians between two normals to be
 * considered in the same plane. Examples: 0.349066f = 20°, 0.261799 = 15°
 */
#define MAX_NORMAL_ANGLE 0.174533f // = 10°
/// PFH evaluation threshold of 62nd bin to be classified as a plane.
#define PLANE_TRESHOLD 20
/// Number of iteration of phase 1. At the third iteration, the neighborhood set is classified as a plane or not.
#define PHASE1_ITERATIONS 3
/// Minimal number of points in a plane for it to be considered stable and not recompute the plane at every iteration.
#define MIN_STABLE_SIZE 500
/// Lower bound on the number of points belonging in a plane, for it to be valid.
#define MIN_PLANE_SIZE 10
/// Upper bound on the number of region grwoing iterations.
#define MAX_ITERATIONS 100

///================================ PLANE MERGING CLOUD ===================================================================================///

/// Number of nearest neighbouring centers to look for when merging planes.
#define KNN 20
/// Upper bound on the angle difference between two planes' normal vectors to be considered in the same alignemnet.
#define NORMAL_ERROR 0.0872665f  // 0.0872665f = 5°
/// Upper bound on the distance between two plane to be considered mergeables.
#define DISTANCE_ERROR 0.1f
/// Angle of the cone between two plane centers in which points will be searched for plane overlap.
#define OVERLAP_ANGLE 0.0872665f // 0.0872665f = 5°, 0.174533f = 10°

///================================ PLANE SEGMENTATION MESH ===============================================================================///

/// Number of neighbooring plane centers to look for when merging planes.
#define KNN_MESH 50
/// Upper bound on the angle difference between two planes' normal vectors to be considered in the same alignement.
#define MESH_NORMAL_ERROR 0.996f //cos(5°)
/// Upper bound on the distance between two vertices to be considered common vertices.
#define V_ERROR 0.5f
/// Lower bound on the plane surface to be considered valid.
#define MIN_SURFACE 10

///================================ REGISTRATION ==========================================================================================///

/// Factor for the Std deviation upper bound when excluding center pairs for realignment.
#define STD_DEV_MULT 2.0
/// Number of center's neighbour to take when computing PFH signature.
#define CENTER_KNN 10
/// Upper bound on the surface difference between a source and target plane to be associated.
#define SURFACE_INTERVAL 200
/// Max number of source planes to use for plane association. //NOT USED
#define MAX_SOURCE_PLANES 50
/// Lower bound on the distance progression after the first alignement for a point pair to be considered a good assiciation. // NOT USED
#define PROGRESSION_TRESHOLD 20
/// Upper bound on the distance allowed between to associated centers after the first alignment, to be used for the realignment. // NOT USED
#define MAX_ACCEPTED_DISTANCE 20
/// Number of bins in the APF histogram for each features
#define NB_BINS_APFH 11
/// Number of features in the APF histogram
#define NB_FEATURES_APFH 5

///================================ TESTING PROCESS =======================================================================================///

#define TARGET_COLOR ivec3(15, 255, 15)
#define SOURCE_COLOR ivec3(255, 15, 15)
#define ALIGNED_COLOR ivec3(15, 15, 255)
#define MAX_VIEWPORT_NB 4
