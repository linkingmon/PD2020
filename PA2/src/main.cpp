#include <iostream>
#include <fstream>
#include <vector>
#include "floorplanner.h"
using namespace std;

int main(int argc, char **argv)
{
    fstream input_block, input_nets;
    fstream output;
    double alpha;

    if (argc == 5)
    {
        alpha = stod(argv[1]);
        if (alpha > 1 || alpha < 0)
        {
            cerr << "The alpha value must be between 0 and 1";
            exit(1);
        }
        input_block.open(argv[2], ios::in);
        input_nets.open(argv[3], ios::in);
        output.open(argv[4], ios::out);
        if (!input_block)
        {
            cerr << "Cannot open the input file .block\"" << argv[2]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!input_nets)
        {
            cerr << "Cannot open the input file .nets\"" << argv[3]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!output)
        {
            cerr << "Cannot open the output file \"" << argv[4]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
    }
    else
    {
        cerr << "Usage: ./fp <alpha value> <input.block> <input.nets> <output file>" << endl;
        exit(1);
    }

    Floorplanner *floorplanner = new Floorplanner(input_block, input_nets, alpha);
    floorplanner->floorplan();
    floorplanner->writeResult(output);

    return 0;
}
