#ifndef PARTITIONER_H
#define PARTITIONER_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
using namespace std;

class Partitioner
{
public:
    // constructor and destructor
    Partitioner(fstream &inFile) : _cutSize(0), _netNum(0), _cellNum(0), _maxPinNum(0), _bFactor(0)
    {
        parseInput(inFile);
        _partSize[0] = 0;
        _partSize[1] = 0;
    }
    ~Partitioner()
    {
        clear();
    }

    // basic access methods
    int getCutSize() const { return _cutSize; }
    int getNetNum() const { return _netNum; }
    int getCellNum() const { return _cellNum; }
    double getBFactor() const { return _bFactor; }
    int getPartSize(int part) const { return _partSize[part]; }

    // modify method
    void parseInput(fstream &inFile);
    void partition();

    // member functions about reporting
    void printSummary() const;
    void reportNet() const;
    void reportCell() const;
    void writeResult(fstream &outFile);

private:
    int _terminal;                 // number of terminals
    int _cutSize;                  // cut size
    int _partSize[2];              // size (cell number) of partition A(0) and B(1)
    int _netNum;                   // number of nets
    int _cellNum;                  // number of cells
    int _maxPinNum;                // Pmax for building bucket list
    double _bFactor;               // the balance factor to be met
    Cell *_maxGainCell;            // pointer to max gain cell
    vector<Net *> _netArray;       // net array of the circuit
    vector<Cell *> _cellArray;     // cell array of the circuit
    map<int, Node *> _bList[2];    // bucket list of partition A(0) and B(1)
    map<string, int> _netName2Id;  // mapping from net name to id
    map<string, int> _cellName2Id; // mapping from cell name to id
    vector<Cell *> move_cells;     // store the moved cells
    int max_partial_gain;          // record max partial gain
    int cur_partial_gain;          // revord current partial gain
    int max_partial_gain_index;    // record the index of max partial gain

    double _min_group_size; // minimum group size
    double _max_group_size; // maximum group size

    // private functions
    // global initialize
    void set_balance_constraint(); // set the balanced constraint
    bool check_balance(int);       // check if the size is in teh balanced range
    void set_initial_solution();   // set the initial solution (first half of the cellArray in one side, and the others in the other side)
    void initial_max_pin();        // calculate max pin number

    // per iteration functions
    void initial_gain(); // initialize all gains
    void build_bucket(); // build the bucket

    // per step functions
    void update_bucket(Cell *, bool); // update gain and move cells
    void move_cell(Cell *);           // update the place of the cell in the bucket list
    void remove_from_bucket(Cell *);  // remove the cell from the bucket list
    void insert_in_bucket(Cell *);    // insert the cell into the bicket list
    void set_max_gain_cell();         // calculate the max gain cell that makes the partition balanced

    // print functions
    void print_balance_constraint() const;
    void print_bucket();
    void print_node(Node *) const;

    // Clean up partitioner
    void clear();
};

#endif // PARTITIONER_H
