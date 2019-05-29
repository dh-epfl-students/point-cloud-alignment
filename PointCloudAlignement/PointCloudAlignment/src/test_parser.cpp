#include "test_parser.h"

bool TestParser::prepareTestingSet(string inputfile, string outputfile)
{
    // Read and parse input file
    vector<ParsedTestSet> l_test = parseFile(inputfile);

    // Create the vector of TestingSet
    vector<TestingSet> l_readySet = createTestSet(l_test);

    // Load and apply random rotations and translations, then preprocess clouds
    applyRandomTransforms(l_readySet);

    preprocessClouds(l_readySet);

    // Save clouds and meshes
    saveObjectsPLY(l_readySet);

    // Write testing txt file
    writeTestSet(l_readySet, outputfile);

    return true;
}

vector<ParsedTestSet> TestParser::parseFile(string inputfile)
{
    vector<ParsedTestSet> l_output;

    ifstream file(inputfile);

    ParsedTestSet set;
    bool nextIsTarget = false;
    string line;

    while(getline(file, line))
    {
        // Tokenize the line
        vector<string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);

        if(tokens[0].find("group") != string::npos)
        {
            // Store previous testing set if it is initialised
            if(set.isInitialised()) l_output.push_back(set);

            // Next tokens of line are numbers of transformations for cloud and mesh
            set = ParsedTestSet(stoi(tokens[2]), stoi(tokens[1]));

            nextIsTarget = true;
        }
        else if(tokens.empty())
        {
            if(set.isInitialised()) l_output.push_back(set);
        }
        else
        {
            // Parsing line of type: "filepath c/m"
            if(nextIsTarget)
            {
                set.setTarget(tokens[0], tokens[1]);
                nextIsTarget = false;
            }
            else
            {
                set.setSource(tokens[0], tokens[1]);
            }
        }
    }

    file.close();

    return l_output;
}

vector<TestingSet> TestParser::createTestSet(vector<ParsedTestSet> &l_test)
{
    vector<TestingSet> output_list;

    for(auto item: l_test)
    {
        TestingSet set(item.target_file, item.t_is_cloud);

        // Add sources clouds
        for(int i = 0; i < item.nbC; ++i)
        {
            set.addSource(item.source_cloud, true);
        }

        // Add sources meshes
        for(int i = 0; i < item.nbM; ++i)
        {
            set.addSource(item.source_mesh, false);
        }
    }

    return output_list;
}

void TestParser::applyRandomTransforms(vector<TestingSet> &l_test)
{
    for(size_t i = 0; i < l_test.size(); ++i)
    {
        l_test[i].applyRandomTransforms();
    }
}

void TestParser::preprocessClouds(vector<TestingSet> &l_test)
{
    for(size_t i = 0; i < l_test.size(); ++i)
    {
        l_test[i].preprocessClouds();
    }
}

void TestParser::saveObjectsPLY(vector<TestingSet> &l_test)
{
    for(size_t i = 0; i < l_test.size(); ++i)
    {
        l_test[i].saveObjectsPLY();
    }
}

void TestParser::writeTestSet(vector<TestingSet> &l_test, string file)
{
    ofstream output(file);

    for(size_t i = 0; i < l_test.size(); ++i)
    {
        l_test[i].writeTestSet(output);
    }

    output << endl;
    output.close();
}
