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

    // Write the preprocessed clouds filenames in this file
    ofstream outputFile("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/PCTestingSet.txt");

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

            string name = p.stem().string();

            //New name will be original + "_preproc"
            stringstream ss;
            ss << name << "_preproc.ply";
            string pc = ss.str();

            auto path = p.remove_filename();
            path.append(pc);

            cout << "Saving PC in " << path.string() << endl;

            pcl::io::savePLYFile(path.string(), *segmenter.getPointCloud());
            outputFile << path.string() << endl;
        }
    }

    outputFile.close();

    cout << "All files were successfully preprocessed. Exiting..." << endl;

    return 0;
}
