#include "pfh_evaluation.h"

bool PFHEvaluation::isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices)
{
    Eigen::VectorXf pfh_histogram;
    pcl::PFHEstimation<PointNormalK, PointNormalK, pcl::PFHSignature125> pfh;
    pfh.computePointPFHSignature(*points, *points, indices, 5, pfh_histogram);

    cout << "PFHSignature125 dimensions: " << pfh_histogram.rows() << ", " << pfh_histogram.cols() << endl;
    for(int i = 0; i < pfh_histogram.rows(); ++i)
    {
        for(int j = 0; j < pfh_histogram.cols(); ++i)
        {
            cout << pfh_histogram[i,j] << " ";
        }

        cout << endl;
    }

    return true;
}
