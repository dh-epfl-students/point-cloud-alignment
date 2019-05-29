#pragma once

#include<string>
#include<fstream>
#include<sstream>

#include<boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "common.h"
#include "test_set.h"

struct ParsedTestSet
{
    int nbM, nbC;
    bool t_is_cloud;
    string target_file;
    string source_cloud;
    string source_mesh;

    ParsedTestSet() {}
    ParsedTestSet(int nbMeshes, int nbClouds): nbM(nbMeshes), nbC(nbClouds) {}

    void setTarget(string file, string type)
    {
        target_file = file;
        t_is_cloud = type == "c";
    }

    void setSource(string file, string type)
    {
        if(type == "c")
        {
            source_cloud = file;
        }
        else if(type == "m")
        {
            source_mesh = file;
        }
    }

    bool isInitialised()
    {
        return !target_file.empty() &&
                ((nbC > 0 && !source_cloud.empty()) || (nbM > 0 && !source_mesh.empty()));
    }
};

class TestParser
{
public:
    bool prepareTestingSet(string inputfile, string outputfile);

private:
    vector<ParsedTestSet> parseFile(string inputfile);

    vector<TestingSet> createTestSet(vector<ParsedTestSet> &l_test);

    void applyRandomTransforms(vector<TestingSet> &l_test);

    void preprocessClouds(vector<TestingSet> &l_test);

    void saveObjectsPLY(vector<TestingSet> &l_test);

    void writeTestSet(vector<TestingSet> &l_test, string file);
};
