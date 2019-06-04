#include<string>

#include "common.h"
#include "test_parser.h"

using namespace std;

int main()
{
    // Read file containing the list of PC to process
    string inputfile("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/testset_creation_list.txt");

    // Write the preprocessed clouds filenames in this file
    string outputFile("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/DEBUG_testing_set_list.txt");

    TestParser parser;
    parser.prepareTestingSet(inputfile, outputFile);

    cout << "All files were successfully preprocessed. Exiting..." << endl;

    return 0;
}
