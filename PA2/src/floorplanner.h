#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include <fstream>
#include <vector>
#include <map>
#include <utility>
#include <iostream>
#include <sstream>
#include <deque>
#include "block.h"
#include "net.h"
#include "contour.h"
using namespace std;

class Floorplanner
{
public:
    // constructor and destructor
    Floorplanner(fstream &inFile_block, fstream &inFile_nets, double alpha) : _blocknum(0), _pinnum(0), _alpha(alpha), _contour(new Contour), my_best_cost(1e5)
    {
        parseInput(inFile_block, inFile_nets);
        _best_node_array.reserve(_blocknum);
        srand(0);
        start_time();
        _step_cnt = 0;
    }
    ~Floorplanner()
    {
        clear();
    }

    // basic access methods
    // int getPartSize(int part) const { return _partSize[part]; }

    // modify method
    void parseInput(fstream &inFile_block, fstream &inFile_net);
    void parseInputBlock(fstream &inFile);
    void parseInputNet(fstream &inFile);
    void floorplan();

    // member functions about reporting
    void printSummary() const;
    void reportBlock() const;
    void reportPin() const;
    void reportNet() const;
    void writeResult(fstream &outFile);

private:
    double _alpha;                   // weight of area cost to wire cost
    double _beta;                    // weight of cost function to aspect cost
    double _area_norm_factor;        // avg of area of random floorplan
    double _wire_length_norm_factor; // avg of wire length of random floorplan
    int _feasible_chain_size = 100;  // max current recording of feasible solution
    int _feasible_count;             // the number of feasible counts
    int _norm_num = 1000;            // times of random floorplanning for estimating area and wire norm factor
    int _blocknum;                   // number of blocks
    int _pinnum;                     // number of pins
    int _netnum;                     // number of nets
    int N;                           // steps per iteration
    double _outline_aspect;          // aspect ratio of the outline (H/W)
    pair<int, int> _outline;         // the width and height of the outline
    vector<Block *> _blockArray;     // block array of the circuit
    vector<Pin *> _pinArray;         // pin array of the circuit
    vector<Net *> _netArray;         // net array of the circuit (each net contains a vector of pin and block)
    map<string, int> _blockName2Id;  // mapping from block name to id
    map<string, int> _pinName2Id;    // mapping from pin name to id
    int reject_num;                  // number of rejection in a iteration
    int uphill_num;                  // number of uphill moves in a iteration
    int move_num;                    // number of moves in a iteration
    clock_t _time;                   //execution time
    int _iteration_cnt;              // number of iterations
    int _step_cnt;                   // number of steps

    // B*tree related
    Node *root_node;               // the root node of the B*tree
    Contour *_contour;             // the contour data structure
    int perturb_type;              // type of operation
    vector<Node> _best_node_array; // save the solution of the best solution
    // for parital recovery of perterb & deperturb
    int delete_type;       // type of deletion
    bool take_type;        // the takken child side for two child type deletion
    Node *record_node;     //
    Node *subtree_node;    // root of teh subtree
    Node *space_node;      // the NULL space of a node
    bool node1_is_right;   //
    bool is_right;         //
    bool take_type_insert; // insert type
    int _perturb_val1;     // picked node index 1
    int _perturb_val2;     // picked node index 2
    // record best cost
    double my_best_cost;
    double my_best_wire;
    int my_best_area;
    int my_best_H;
    int my_best_W;
    Node *my_best_root_node;

    //SA related
    // vector<pair<int, double>> _solution_chain;
    deque<bool> _feasible_chain;

    // debugging related parameter
    int error_cnt; // tree deep

    // time related
    void start_time() { _time = clock(); }
    float get_time() { return float(clock() - _time) / CLOCKS_PER_SEC; }

    // private functions
    // global initialize
    void calculate_norm_factor();
    void random_build_tree(); /*is removed*/
    void set_initial_solution();
    void calculate_outline_aspect() { _outline_aspect = double(_outline.second) / double(_outline.first); };
    void reset_node();

    // SA related
    double SA_iteration(double &);
    double random_floorplan(int times); // for calculating the scale of the cost
    void keep_best_cost(double &, int &, double &, int &, int &);
    void keep_best_nodes();
    void recover_best_nodes();
    void set_feasible(bool);

    // calculate cost functions
    double calculate_total_cost();
    double calculate_total_cost(int &, double &, int &, int &);
    void calculate_area_cost(int &, int &, int &);
    void calculate_wire_length_cost(double &);

    // per step operation
    // B*tree related function
    void update_x(Node *);
    void update_y(Node *);
    void update_node();
    void perturb();
    void deperturb();
    Node *find_space(Node *root, bool &is_right);

    // print functions
    void print_tree();            // print current B*-tree
    void print_node(Node *, int); // recursive calling function called by print_tree()

    // Clean up floorplanner
    void clear();
};

#endif // FLOORPLANNER_H
