#include "pfh_evaluation.h"

bool PFHEvaluation::isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices)
{
    Eigen::VectorXf pfh_histogram(125);
    pcl::PFHEstimation<PointNormalK, PointNormalK, pcl::PFHSignature125> pfh;
    pfh.computePointPFHSignature(*points, *points, indices, 5, pfh_histogram);

    // Test the histogram for plane

    cout << "PFHSignature125 dimensions: " << pfh_histogram.rows() << ", " << pfh_histogram.cols() << endl;
    for(int i = 0; i < pfh_histogram.rows(); ++i)
    {
        cout << i << ": " << pfh_histogram[i] << " ";
    }
    cout << endl;

    //TODO: enhance this part
    return pfh_histogram[62] > PLANE_TRESHOLD;
}
