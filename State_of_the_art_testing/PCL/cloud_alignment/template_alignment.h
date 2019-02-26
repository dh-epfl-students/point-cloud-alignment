#pragma once

#include <string>

#include <Eigen/Core>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ia_kfpcs.h>

#include "feature_cloud.h"

using namespace std;

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (100),
      approx_overlap(1.0),
      voxel_size(0.5f),
      abort_score(0.0),
      nr_threads(4)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);

      // Initialize the parameters in the kFPCS algorithm
      kfpcs_ia_.setApproxOverlap(this->approx_overlap);
      kfpcs_ia_.setDelta(this->voxel_size, false);
      kfpcs_ia_.setScoreThreshold(this->abort_score);
      kfpcs_ia_.setNumberOfThreads(this->nr_threads);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());

      kfpcs_ia_.setInputTarget(target_cloud.getPointCloud());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud () using sample consensus initial alignment
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align the given template cloud to the target specified by setTargetCloud() using KFPCS initial alignment
    void alignKFPCS(FeatureCloud &template_cloud, TemplateAlignment::Result &result) {
        kfpcs_ia_.setInputCloud(template_cloud.getPointCloud());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        kfpcs_ia_.align(registration_output);

        cout << "Size of registered cloud: " << registration_output.size() << endl;

        result.fitness_score = (float)kfpcs_ia_.getFitnessScore();
        result.final_transformation = kfpcs_ia_.getFinalTransformation();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (string method, std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());

      for (size_t i = 0; i < templates_.size (); ++i)
      {
          if(method.compare("KFPCS")) {
            alignKFPCS(templates_[i], results[i]);
          } else {
            align (templates_[i], results[i]);
          }
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result, string method = "KFPCS")
    {
      cout << "Finding best alignment using " << method << endl;

      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (method, results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;

    // The Four Points Congruent Sets Initial Alignment (KFPCS) registration routine and its parameters
    pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs_ia_;
    int nr_threads;
    float voxel_size;
    float approx_overlap;
    float abort_score;
};
