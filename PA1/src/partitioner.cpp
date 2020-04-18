#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include "cell.h"
#include "net.h"
#include "partitioner.h"
using namespace std;

void Partitioner::parseInput(fstream &inFile)
{
    string str;
    // Set balance factor
    inFile >> str;
    _bFactor = stod(str);

    // Set up whole circuit
    while (inFile >> str)
    {
        if (str == "NET")
        {
            string netName, cellName, tmpCellName = "";
            inFile >> netName;
            int netId = _netNum;
            _netArray.push_back(new Net(netName));
            _netName2Id[netName] = netId;
            while (inFile >> cellName)
            {
                if (cellName == ";")
                {
                    tmpCellName = "";
                    break;
                }
                else
                {
                    // a newly seen cell
                    if (_cellName2Id.count(cellName) == 0)
                    {
                        int cellId = _cellNum;
                        _cellArray.push_back(new Cell(cellName, 0, cellId));
                        _cellName2Id[cellName] = cellId;
                        _cellArray[cellId]->addNet(netId);
                        _cellArray[cellId]->incPinNum();
                        _netArray[netId]->addCell(cellId);
                        ++_cellNum;
                        tmpCellName = cellName;
                    }
                    // an existed cell
                    else
                    {
                        if (cellName != tmpCellName)
                        {
                            assert(_cellName2Id.count(cellName) == 1);
                            int cellId = _cellName2Id[cellName];
                            _cellArray[cellId]->addNet(netId);
                            _cellArray[cellId]->incPinNum();
                            _netArray[netId]->addCell(cellId);
                            tmpCellName = cellName;
                        }
                    }
                }
            }
            ++_netNum;
        }
    }
    return;
}

void Partitioner::partition()
{
    set_balance_constraint(); // set balance constraint parameter
    set_initial_solution();   // set initial solution, the first half in cellarray in one side, and the other half in the other
    initial_max_pin();        // calculate pmax for build_bucket
    int cnt = 0;
    while (true)
    {
        cout << "Current cutsize: " << _cutSize << " in iteration " << cnt << endl;
        cnt += 1;
        initial_gain(); // initialize gain (per iteration)
        build_bucket(); // builb bucket (per iteration)
        // initialize partial gain
        move_cells.clear();
        cur_partial_gain = 0;
        max_partial_gain = -2147483648;
        max_partial_gain_index = 0;

        // check max gain cell & terminate step
        while (true)
        {
            set_max_gain_cell(); // calculate max gain cell
            if (_maxGainCell == NULL)
                break;
            cur_partial_gain += _maxGainCell->getGain();
            move_cells.push_back(_maxGainCell);
            if (max_partial_gain <= cur_partial_gain)
            {
                max_partial_gain = cur_partial_gain;
                max_partial_gain_index = move_cells.size();
            }
            // move cells
            move_cell(_maxGainCell);
        }

        // find maximum partial sum and move the cells that required
        if (max_partial_gain == 0)
            break; // no more gain increment
        for (int j = max_partial_gain_index; j < move_cells.size(); ++j)
        { // unmove the cells after max_partial_gain_index
            // update partSize
            int from_side = move_cells[j]->getPart();
            _partSize[from_side] -= 1;
            _partSize[!from_side] += 1;
            // update net
            vector<int> cur_netlist = move_cells[j]->getNetList();
            for (int k = 0; k < cur_netlist.size(); ++k)
            {
                Net *&cur_net = _netArray[cur_netlist[k]];
                cur_net->incPartCount(!from_side);
                cur_net->decPartCount(from_side);
            }
            move_cells[j]->move();
        }
        _cutSize -= max_partial_gain;

        // unlock all
        for (int j = 0; j < _cellArray.size(); ++j)
            _cellArray[j]->unlock();
    }
}

void Partitioner::printSummary() const
{
    cout << endl;
    cout << "==================== Summary ====================" << endl;
    cout << " Cutsize: " << _cutSize << endl;
    cout << " Total cell number: " << _cellNum << endl;
    cout << " Total net number:  " << _netNum << endl;
    cout << " Cell Number of partition A: " << _partSize[0] << endl;
    cout << " Cell Number of partition B: " << _partSize[1] << endl;
    cout << " Numebr of terminals: " << _terminal << endl;
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Partitioner::reportNet() const
{
    cout << "Number of nets: " << _netNum << endl;
    for (size_t i = 0, end_i = _netArray.size(); i < end_i; ++i)
    {
        cout << setw(8) << _netArray[i]->getName() << ": ";
        vector<int> cellList = _netArray[i]->getCellList();
        for (size_t j = 0, end_j = cellList.size(); j < end_j; ++j)
        {
            cout << setw(8) << _cellArray[cellList[j]]->getName() << " ";
        }
        cout << endl;
    }
    return;
}

void Partitioner::reportCell() const
{
    cout << "Number of cells: " << _cellNum << endl;
    for (size_t i = 0, end_i = _cellArray.size(); i < end_i; ++i)
    {
        cout << setw(8) << _cellArray[i]->getName() << ": ";
        vector<int> netList = _cellArray[i]->getNetList();
        for (size_t j = 0, end_j = netList.size(); j < end_j; ++j)
        {
            cout << setw(8) << _netArray[netList[j]]->getName() << " ";
        }
        cout << endl;
    }
    return;
}

void Partitioner::writeResult(fstream &outFile)
{
    stringstream buff;
    buff << _cutSize;
    outFile << "Cutsize = " << buff.str() << '\n';
    buff.str("");
    buff << _partSize[0];
    outFile << "G1 " << buff.str() << '\n';
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    {
        if (_cellArray[i]->getPart() == 0)
        {
            outFile << _cellArray[i]->getName() << " ";
        }
    }
    outFile << ";\n";
    buff.str("");
    buff << _partSize[1];
    outFile << "G2 " << buff.str() << '\n';
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    {
        if (_cellArray[i]->getPart() == 1)
        {
            outFile << _cellArray[i]->getName() << " ";
        }
    }
    outFile << ";\n";
    return;
}

void Partitioner::print_balance_constraint() const
{
    cout << endl;
    cout << "================== Constraint ===================" << endl;
    cout << "Bfactor:            " << _bFactor << endl;
    cout << "Minimum group size: " << _min_group_size << endl;
    cout << "Maximum group size: " << _max_group_size << endl;
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Partitioner::set_balance_constraint()
{
    _min_group_size = double(1 - _bFactor) * double(_cellNum) / 2;
    _max_group_size = double(1 + _bFactor) * double(_cellNum) / 2;
    return;
}

bool Partitioner::check_balance(int group_size)
{
    return (group_size >= _min_group_size && group_size <= _max_group_size);
}

void Partitioner::set_initial_solution()
{
    // move lower half read cell to the other side
    for (int i = 0; i < int(_cellNum / 2); ++i)
        _cellArray[i]->move();

    // update PartCount in each Net (Linear time)
    for (int i = 0; i < _cellArray.size(); ++i)
    {
        vector<int> net_idx = _cellArray[i]->getNetList();
        bool cell_side = _cellArray[i]->getPart();
        _partSize[cell_side] += 1;
        for (int j = 0; j < net_idx.size(); ++j)
            _netArray[net_idx[j]]->incPartCount(cell_side);
    }

    // calculate initial cut size (Linear time)
    for (int i = 0; i < _netArray.size(); ++i)
        if ((_netArray[i]->getPartCount(0) != 0) // the net has cells in both side (it is cut!)
            && (_netArray[i]->getPartCount(1) != 0))
            _cutSize += 1;

    return;
}

void Partitioner::initial_gain()
{
    // zero all gain
    for (int i = 0; i < _cellArray.size(); ++i)
        _cellArray[i]->setGain(0);

    // run intial gain (Linear time)
    for (int i = 0; i < _cellArray.size(); ++i)
    {
        Cell *&cur_cell = _cellArray[i];
        vector<int> net_idx = cur_cell->getNetList();
        bool from_side = cur_cell->getPart();
        for (int j = 0; j < net_idx.size(); ++j)
        {
            Net *&cur_net = _netArray[net_idx[j]];
            if (cur_net->getPartCount(from_side) == 1)
                cur_cell->incGain();
            if (cur_net->getPartCount(!from_side) == 0)
                cur_cell->decGain();
        }
    }
    return;
}

void Partitioner::clear()
{
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    {
        delete _cellArray[i];
    }
    for (size_t i = 0, end = _netArray.size(); i < end; ++i)
    {
        delete _netArray[i];
    }
    return;
}

void Partitioner::initial_max_pin()
{
    for (int i = 0; i < _cellArray.size(); ++i)
    {
        int pin_num = _cellArray[i]->getPinNum();
        _terminal += pin_num;
        if (pin_num > _maxPinNum)
            _maxPinNum = pin_num;
    }
}

void Partitioner::build_bucket()
{
    int max_gain = 0;
    _bList[0].clear();
    _bList[1].clear();
    for (int i = 0; i < _cellArray.size(); ++i)
    {
        Cell *&cur_cell = _cellArray[i];
        insert_in_bucket(cur_cell);
    }
}

void Partitioner::print_bucket()
{
    cout << endl;
    cout << "==================== Bucket =====================" << endl;
    for (int side = 0; side < 2; ++side)
    {
        cout << "=================== Bucket " << side << " ====================" << endl;
        for (int i = -_maxPinNum; i <= _maxPinNum; ++i)
        {
            map<int, Node *> &cur_bucket = _bList[side];
            cout << "Bucket" << setw(4) << i << ": ";
            if (cur_bucket.count(i) > 0)
                print_node(cur_bucket[i]);
            cout << endl;
        }
    }
    cout << endl;
    return;
}

void Partitioner::print_node(Node *head_node) const
{
    cout << _cellArray[head_node->getId()]->getName() << " ";
    if (head_node->getNext() != 0)
        print_node(head_node->getNext());
}

void Partitioner::update_bucket(Cell *cur_cell, bool increment)
{
    remove_from_bucket(cur_cell);
    if (increment)
        cur_cell->incGain();
    else
        cur_cell->decGain();
    insert_in_bucket(cur_cell);
}

void Partitioner::move_cell(Cell *max_gain_cell)
{
    // Update max gain cell & remove from bucket
    remove_from_bucket(max_gain_cell);
    max_gain_cell->lock();
    bool from_side = max_gain_cell->getPart();
    vector<int> cur_netlist = max_gain_cell->getNetList();
    _partSize[from_side] -= 1;
    _partSize[!from_side] += 1;
    // Move and Update gain
    // seems to be quadratic time, but the operation in the inner loop will only be run for constant times
    // also update gain if gain is increment or decrement
    for (int j = 0; j < cur_netlist.size(); ++j)
    {
        Net *&cur_net = _netArray[cur_netlist[j]];
        vector<int> cell_idx_list = cur_net->getCellList();

        // Before Move
        int to_side_size = cur_net->getPartCount(!from_side);
        if (to_side_size == 0)
        {
            for (int k = 0; k < cell_idx_list.size(); ++k)
            {
                Cell *&cur_cell = _cellArray[cell_idx_list[k]];
                if (!(cur_cell->getLock()))
                    update_bucket(cur_cell, true);
            }
        } // increment all free cells on cur_net
        else if (to_side_size == 1)
        {
            for (int k = 0; k < cell_idx_list.size(); ++k)
            {
                Cell *&cur_cell = _cellArray[cell_idx_list[k]];
                if (cur_cell->getPart() == !from_side && !(cur_cell->getLock()))
                    update_bucket(cur_cell, false);
            }
        } // decrement the only cell on the To side if it is free (how to find the only cell)

        // Move
        cur_net->decPartCount(from_side);
        cur_net->incPartCount(!from_side);

        // After Move
        int from_side_size = cur_net->getPartCount(from_side);
        if (from_side_size == 0)
        {
            for (int k = 0; k < cell_idx_list.size(); ++k)
            {
                Cell *&cur_cell = _cellArray[cell_idx_list[k]];
                if (!(cur_cell->getLock()))
                    update_bucket(cur_cell, false);
            }
        } // decrement all free cells on cur_net
        else if (from_side_size == 1)
        {
            for (int k = 0; k < cell_idx_list.size(); ++k)
            {
                Cell *&cur_cell = _cellArray[cell_idx_list[k]];
                if (cur_cell->getPart() == from_side && !(cur_cell->getLock()))
                    update_bucket(cur_cell, true);
            }
        } // increment the only cell on the From side if it is free
    }
    max_gain_cell->move();
}

void Partitioner::remove_from_bucket(Cell *cur_cell)
{
    // get bucket
    bool cur_cell_side = cur_cell->getPart();
    map<int, Node *> &cur_bucket = _bList[cur_cell_side];
    // remove
    int cur_cell_gain = cur_cell->getGain();
    Node *cur_node = cur_cell->getNode();
    Node *prev_node = cur_node->getPrev();
    Node *next_node = cur_node->getNext();
    if (prev_node == NULL)
    {
        if (next_node == NULL)
            cur_bucket.erase(cur_cell_gain);
        else
        {
            cur_bucket[cur_cell_gain] = next_node;
            next_node->setPrev(NULL);
        }
    }
    else
    {
        if (next_node == NULL)
            prev_node->setNext(NULL);
        else
        {
            prev_node->setNext(next_node);
            next_node->setPrev(prev_node);
        }
    }
}

void Partitioner::insert_in_bucket(Cell *cur_cell)
{
    // get bucket
    bool cur_cell_side = cur_cell->getPart();
    map<int, Node *> &cur_bucket = _bList[cur_cell_side];
    // insert
    int cur_cell_gain = cur_cell->getGain();
    Node *cur_node = cur_cell->getNode();
    if (!(cur_bucket.count(cur_cell_gain)))
    {
        cur_node->setNext(NULL);
        cur_node->setPrev(NULL);
        cur_bucket[cur_cell_gain] = cur_node;
    }
    else
    {
        Node *&original_head = cur_bucket[cur_cell_gain];
        original_head->setPrev(cur_node);
        cur_node->setNext(original_head);
        cur_node->setPrev(NULL);
        cur_bucket[cur_cell_gain] = cur_node;
    }
}

void Partitioner::set_max_gain_cell()
{
    bool max_gain_in_side0 = true;
    bool max_gain_in_side1 = true;
    // check balance condition
    if (check_balance(_partSize[0]) && !check_balance(_partSize[0] - 1)) // if we choose max gain cell at side 0, it must be unbalances
        max_gain_in_side0 = false;
    else if (check_balance(_partSize[0]) && !check_balance(_partSize[0] + 1)) // if we choose max gain cell at side 1, it must be unbalances
        max_gain_in_side1 = false;
    // check if bucket is not empty
    map<int, Node *>::reverse_iterator iter0 = (_bList[0].rbegin());
    map<int, Node *>::reverse_iterator iter1 = (_bList[1].rbegin());
    if (iter0 == _bList[0].rend())
        max_gain_in_side0 = false;
    if (iter1 == _bList[1].rend())
        max_gain_in_side1 = false;
    // determine max gain cell
    if (max_gain_in_side0 && !max_gain_in_side1)
        _maxGainCell = _cellArray[iter0->second->getId()];
    else if (!max_gain_in_side0 && max_gain_in_side1)
        _maxGainCell = _cellArray[iter1->second->getId()];
    else if (!max_gain_in_side0 && !max_gain_in_side1)
        _maxGainCell = NULL;
    else
    {
        int max_gain_1 = iter0->first;
        int max_gain_2 = iter1->first;
        if (max_gain_1 >= max_gain_2)
            _maxGainCell = _cellArray[iter0->second->getId()];
        else
            _maxGainCell = _cellArray[iter1->second->getId()];
    }
}