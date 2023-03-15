#pragma once

#include "open3d/pipelines/pose_estimation/DataStructure.h"
#include "open3d/pipelines/pose_estimation/Utils.h"

// view point position factor
const int VIEW_POINT_Z_EXTEND = 2;
// maximum nn points of kd-tree search for normal computing
const int NORMAL_CALC_NN =30;
// maximum iteration of sparse pose refine for clustered pose
const int SPARSE_REFINE_ICP_ITERATION =30;
const double NEIGHBOR_RADIUS_FACTOR =0.5;
const double VOTING_THRESHOLD_FACTOR =0.2;
const double VOTE_NUM_RATIO = 0.8;
const double VOTES_NUM_REDUCTION_FACTOR = 0.25;

namespace open3d {
namespace pipelines {
namespace pose_estimation {

class PPFEstimatorConfig {
public:
    PPFEstimatorConfig();
    ~PPFEstimatorConfig();

    // param to control key points selection
    // TODO: can be extended more key points selection methods
    enum class ReferencePointSelection {
        Random = 0,
    };

    // voting mode of PPF.
    // using_all_sampled_points: usually for most of case, in which the object
    // has surface metrics using_edge_points: for object which has flat shape
    // and has boundary metrics.
    // TODO: this mode still need to improved.
    enum class VotingMode {
        SampledPoints = 0,
        EdgePoints = 1,
    };

    // sparse pose refine methods
    enum class RefineMethod {
        NoRefine = 0,
        PointToPoint = 1,
        PointToPlane = 2,
    };

    struct TrainingParam {
        // flag to control whether invert model normal, usually set to true of
        // input point clouds are not in camera coordinate (RGBD camera), or set
        // to false as default.
        bool invert_model_normal;

        // whether use external pre-computed normals for training.
        bool use_external_normal;

        // relative variable of point cloud sample distance
        double rel_sample_dist;
        // relative variable of normal search radius, usually is set as half of
        // rel_sample_dist
        double calc_normal_relative;
        // relative variable of dense point cloud sample distance, only used in
        // edge mode
        double rel_dense_sample_dist;
    };

    struct ReferenceParam {
        ReferencePointSelection method;
        // the ratio of point clouds would be selected
        double ratio;
    };

    struct VotingParam {
        VotingMode method;
        // if in faster mode, the spread of ppf hash table will be reduced
        bool faster_mode;
        // ppf quantization resolution
        double angle_step;
        // minimum distance and angle threshold for point pair filtering
        double min_dist_thresh;
        double min_angle_thresh;
    };

    struct EdgeParam {
        // number of nearest points for kd-tree searching
        int pts_num;
        double angle_threshold;
    };

    struct RefineParam {
        RefineMethod method;
        // icp refine distance thresh, the actual D = rel_dist_sparse_thresh *
        // diameter * rel_sample_dist.
        double rel_dist_sparse_thresh;
    };

    TrainingParam training_param_;
    ReferenceParam ref_param_;
    VotingParam voting_param_;
    EdgeParam edge_param_;
    RefineParam refine_param_;

    // relative variable to control pose clustering distance threshold
    double rel_dist_thresh_;
    // threshold to decide a valid estimated pose
    double rel_angle_thresh_;
    // score threshold for outlier pose removal
    double score_thresh_;
    // control number of output pose
    int num_result_;
    int object_id_;
};

class PPFEstimator {
public:
    PPFEstimator();

    PPFEstimator(const PPFEstimatorConfig &config);

    ~PPFEstimator();

    /**
     * @brief Set Configuration
     *
     * @param config
     * @return true
     * @return false
     */
    bool SetConfig(const PPFEstimatorConfig &config);

    /**
     * @brief Train PPF estimator with given model point clouds
     *
     * @param pc
     * @return true
     * @return false
     */
    bool Train(const PointCloudPtr &pc);

    /**
     * @brief Estimate 6d pose from scene point clouds
     *
     * @param pc
     * @param results
     * @return true
     * @return false
     */
    bool Estimate(const PointCloudPtr &pc, std::vector<Pose6D> &results);

    /**
     * @brief Get Model Diameter
     *
     * @return double
     */
    double GetModelDiameter();

    /**
     * @brief Get all matched results
     *
     * @return std::vector<Pose6D>
     */
    std::vector<Pose6D> GetPose();

    /**
     * @brief Get sampled model point clouds
     *
     * @return open3d::geometry::PointCloud
     */
    open3d::geometry::PointCloud GetSampledModel();

    /**
     * @brief Get sampled scene point clouds
     *
     * @return open3d::geometry::PointCloud
     */
    open3d::geometry::PointCloud GetSampledScene();

    /**
     * @brief Get model edge points if using edge voting method
     *
     * @return open3d::geometry::PointCloud
     */
    open3d::geometry::PointCloud GetModelEdges();

    /**
     * @brief Get scene edge point if using edge voting method
     *
     * @return open3d::geometry::PointCloud
     */
    open3d::geometry::PointCloud GetSceneEdges();
    /**
     * @brief load ppf model from filename which endswith .p3m
     *
     * @param filename
     */
    bool LoadModel(const std::string &filename);
    /**
     * @brief save ppf model from filename which endswith .p3m
     *
     * @param filename
     */
    bool SaveModel(const std::string &filename);
private:

    class Impl;
    std::unique_ptr<Impl> impl_ptr_;
    bool CheckConfig(const PPFEstimatorConfig &config);
    bool ReadFromFile(FILE * file);
    bool WriteToFile(FILE * file);
};

}  // namespace pose_estimation
}  // namespace pipelines
}  // namespace open3d