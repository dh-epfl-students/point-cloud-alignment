#include<string>
#include<fstream>
#include<sstream>

#include<boost/filesystem.hpp>

#include<omp.h>

#include "common.h"
#include "plane_segmentation.h"

using namespace std;

int main()
{
    // Read file containing the list of PC to process
    string filename("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/PCtoPreprocess.txt");
    vector<string> fileList;

    ifstream file;
    file.open(filename);

    string line;
    while(getline(file, line))
    {
        fileList.push_back(line.c_str());
    }

    file.close();

    #pragma omp parallel for
    for(size_t i = 0; i < fileList.size(); ++i)
    {
        string currFile = fileList[i];

        if(!currFile.empty())
        {
            PlaneSegmentation segmenter;
            segmenter.init(currFile);
            segmenter.resampleCloud();
            segmenter.preprocessCloud();

            // Preprocessed file is stored in the same folder as the input file
            boost::filesystem::path p(currFile);
            string pc_file = p.filename().string();
            auto path = p.remove_filename();
            cout << "Saving PC in " << pc_file << endl;

            pcl::io::savePLYFile(pc_file, *segmenter.getPointCloud());
        }
    }

    return 0;
}
