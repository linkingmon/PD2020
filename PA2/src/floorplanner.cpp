#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include "block.h"
#include "net.h"
#include "floorplanner.h"
// #define DEBUG_MODE
// #define PRINT_MODE
using namespace std;
enum
{
    ROTATE,
    INSERT_DEL,
    SWAP
};

float rand_01()
{
    return double(rand() % 100000) * double(1e-5);
}

void Floorplanner::parseInput(fstream &inFile_block, fstream &inFile_nets)
{
    parseInputBlock(inFile_block);
    parseInputNet(inFile_nets);
}

void Floorplanner::parseInputBlock(fstream &inFile)
{
    string str_title, str_value1, str_value2;
    // read outline and numbers of block and pin
    inFile >> str_title >> str_value1 >> str_value2;
    _outline = make_pair(stoi(str_value1), stoi(str_value2));
    inFile >> str_title >> str_value1;
    inFile >> str_title >> str_value1;
    // Set up blocks
    while (inFile >> str_title)
    {
        inFile >> str_value1;
        if (str_value1 == "terminal")
        {
            inFile >> str_value1 >> str_value2;
            _pinArray.push_back(new Pin(str_title, stoi(str_value1), stoi(str_value2)));
            _pinName2Id[str_title] = _pinnum;
            ++_pinnum;
        }
        else
        {
            inFile >> str_value2;
            _blockArray.push_back(new Block(str_title, stoi(str_value1), stoi(str_value2), _blocknum));
            _blockName2Id[str_title] = _blocknum;
            ++_blocknum;
        }
    }
    return;
}

void Floorplanner::parseInputNet(fstream &inFile)
{
    string str_title, str_value;
    inFile >> str_title >> str_value;
    while (inFile >> str_title >> str_value)
    {
        int netId = _netnum;
        int netSize = stoi(str_value);
        _netArray.push_back(new Net());
        for (size_t i = 0; i < netSize; ++i)
        {
            inFile >> str_value;
            // it is a block
            if (_blockName2Id.count(str_value))
            {
                int blockId = _blockName2Id[str_value];
                _netArray[netId]->addBlock(reinterpret_cast<Pin *>(_blockArray[blockId]));
            }
            // it is a pin
            else
            {
                int pinId = _pinName2Id[str_value];
                _netArray[netId]->addBlock(_pinArray[pinId]);
            }
        }
        ++_netnum;
    }
    return;
}

double Floorplanner::random_floorplan(int steps)
{
    cout << endl;
    cout << "============= Random Floorplanning ==============" << endl;
    double total_cost = 0; // recording the average uphill move cost
    double best_cost, prev_cost, cur_cost;
    int t = 0; // times of uphill moves
    update_node();
    best_cost = prev_cost = calculate_total_cost();
    // keep best
    do
    {
        for (int i = 0; i < steps; ++i)
        {
            perturb();
            update_node();
            cur_cost = calculate_total_cost();
            if (cur_cost - prev_cost > 0)
            {
                ++t;
                total_cost += (cur_cost - prev_cost);
                prev_cost = cur_cost;
            }
            if (best_cost > cur_cost)
            {
                best_cost = cur_cost;
            }
        }
    } while (total_cost == 0);
    // recover bset
    // update node
    total_cost /= t;
    cout << "Avg cost: " << total_cost << endl;
    cout << "=================================================" << endl;
    cout << endl;
    return total_cost;
}

void Floorplanner::floorplan() // apply fast-SA scheme
{
    // calculate outline aspect
    calculate_outline_aspect();
    // report parsing results
#ifdef PRINT_MODE
    reportPin();
    reportBlock();
#endif
    // set initial floorplan
    set_initial_solution();
    // calculate the normalize factor of area and wirelength (also the aspect ratio scaling factor)
    calculate_norm_factor();
    // random floorplanning for calculate avg cost

    int k = 10;
    double P = 0.95;
    N = _blocknum * k;
    double avg_cost = random_floorplan(N);
    double T1 = avg_cost / log(1 / P); // set initial temperature
    double terminate_temperature = 1e-4;
    int c = 100;
    int num_stage_1 = 1; // 1 num for 7 iteration
    int num_stage_2 = 7;
    // stage I : aims to put into the outline (minimize area)
    cout << "========== SA Floorplanning - Stage I ===========" << endl;
#ifdef PRINT_MODE
    cout << "Temp " << fixed << setprecision(5) << T1 << " Iteration " << 1 << " has cost: " << my_best_cost << endl;
#endif
    avg_cost = SA_iteration(T1);
    cout << "=================================================" << endl;
    cout << endl;
    // stage II : aims to put into the outline (minimize area)
    cout << "======= Fast-SA Floorplanning - Stage II ========" << endl;
    recover_best_nodes();
    for (int i = num_stage_1; i < num_stage_2; ++i)
    {
        double cur_temperature = T1 * avg_cost / i / c;
#ifdef PRINT_MODE
        cout << "Temp " << fixed << setprecision(5) << cur_temperature << " Iteration " << i << " has cost: " << my_best_cost << endl;
#endif
        avg_cost = SA_iteration(cur_temperature);
    }
    cout << "=================================================" << endl;
    cout << endl;
    // stage III : classical SA
    cout << "======= Fast-SA Floorplanning - Stage III =======" << endl;
    recover_best_nodes();
    for (int i = num_stage_2;; ++i)
    {
        double cur_temperature = T1 * avg_cost / i;
#ifdef PRINT_MODE
        cout << "Temp " << fixed << setprecision(5) << cur_temperature << " Iteration " << i << " has cost: " << my_best_cost << endl;
#endif
        avg_cost = SA_iteration(cur_temperature);
        if (cur_temperature < terminate_temperature && (double(reject_num) / double(move_num) > 0.95))
        {
            _iteration_cnt = i;
            break;
        }
    }
    cout << "=================================================" << endl;
    cout << endl;
}

double Floorplanner::SA_iteration(double &cur_temperature)
{
    // local parameters
    double prev_cost, cur_cost;
    prev_cost = calculate_total_cost();

    reject_num = 0;
    uphill_num = 0;
    move_num = 0;
    // initialize the feasible chain with one false
    _feasible_chain.push_back(false);
    double avg_cost = 0;
    do
    {
        ++_step_cnt;
        ++move_num;
        perturb();
        update_node();
        double wire;
        int area, max_x, max_y;
        _beta = 1 - _feasible_count / _feasible_chain.size();
        cur_cost = calculate_total_cost(area, wire, max_x, max_y);
        // cout << "cur_cost " << cur_cost << endl;
        // cout << "beta " << _beta << endl;
        double d_cost = cur_cost - prev_cost;
        avg_cost += d_cost;
        float p = 1 / (exp(d_cost / cur_temperature)); // probapility for uphill move
        p = p > 1 ? 1 : p;
        if (d_cost > 0 && rand_01() > p) // reject
        {
            ++reject_num;
            deperturb();
            set_feasible(false);
        }
        else
        {
            if (d_cost > 0)
                ++uphill_num;
            prev_cost = cur_cost;
            if (cur_cost < my_best_cost)
            {
                keep_best_cost(cur_cost, area, wire, max_x, max_y);
                keep_best_nodes();
            }
            set_feasible(true);
        }
    } while (move_num < 2 * N && uphill_num < N);
    return avg_cost / double(move_num);
}

void Floorplanner::set_feasible(bool feasible)
{

    if (_feasible_chain.size() > _feasible_chain_size)
    {
        if (_feasible_chain.front() == true)
            --_feasible_count;
        _feasible_chain.pop_front();
    }
    _feasible_chain.push_back(feasible);
    if (feasible)
        ++_feasible_count;
    return;
}

void Floorplanner::calculate_norm_factor()
{
    cout << endl;
    cout << "============ Calculate norm factor ==============" << endl;
    _area_norm_factor = 0;
    _wire_length_norm_factor = 0;
    for (size_t i = 0; i < _norm_num; ++i)
    {
        perturb();
        update_node();
        int max_x, max_y, area;
        double wire;
        calculate_area_cost(max_x, max_y, area);
        calculate_wire_length_cost(wire);
        _area_norm_factor += area;
        _wire_length_norm_factor += wire;
    }
    _area_norm_factor /= double(_norm_num);
    _wire_length_norm_factor /= double(_norm_num);
    cout << "Average area: " << setw(10) << _area_norm_factor << '\n';
    cout << "Average wire length: " << setw(8) << _wire_length_norm_factor << '\n';
    cout << "=================================================" << endl;
}

void Floorplanner::set_initial_solution()
{
    cout << endl;
    cout << "============ Setting initial solution ===========" << endl;
    for (int i = 0; i < _blocknum; ++i)
    {
        _blockArray[i]->getNode()->setParent((((i + 1) / 2 - 1) >= 0) ? _blockArray[(i + 1) / 2 - 1]->getNode() : NULL);
        _blockArray[i]->getNode()->setLeft((2 * i + 1 < _blocknum ? _blockArray[2 * i + 1]->getNode() : NULL));
        _blockArray[i]->getNode()->setRight((2 * i + 2 < _blocknum ? _blockArray[2 * i + 2]->getNode() : NULL));
    }
    root_node = _blockArray[0]->getNode();
#ifdef PRINT_MODE
    print_tree();
#endif
    cout << "=================================================" << endl;
}

void Floorplanner::random_build_tree()
{
    reset_node();
    vector<int> unchosen_index, chosen_index;
    unchosen_index.reserve(_blocknum);
    chosen_index.reserve(_blocknum);
    for (size_t i = 0; i < _blocknum; ++i)
        unchosen_index.push_back(i);
    for (size_t i = 0; i < _blocknum; ++i)
    {
        // randomly choose one from unchosen set
        int rand_num = rand() % unchosen_index.size();
        int cur_index = unchosen_index[rand_num];
        unchosen_index.erase(unchosen_index.begin() + rand_num);
        // insert on any node, if none on root
        if (chosen_index.size() == 0)
        {
            root_node = _blockArray[cur_index]->getNode();
        }
        else
        {
            int parent_index = chosen_index[rand() % chosen_index.size()];
            Node *parent_node = _blockArray[parent_index]->getNode();
            Node *cur_node = _blockArray[cur_index]->getNode();
            if (rand() % 2)
            {
                if (parent_node->getLeft() == NULL)
                {
                    parent_node->setLeft(cur_node);
                    cur_node->setParent(parent_node);
                }
                else
                {
                    if (rand() % 2)
                    {
                        cur_node->setLeft(root_node);
                        root_node->setParent(cur_node);
                    }
                    else
                    {
                        cur_node->setRight(root_node);
                        root_node->setParent(cur_node);
                    }
                    root_node = cur_node;
                }
            }
            else
            {
                if (parent_node->getRight() == NULL)
                {
                    parent_node->setRight(cur_node);
                    cur_node->setParent(parent_node);
                }
                else
                {
                    if (rand() % 2)
                    {
                        cur_node->setLeft(root_node);
                        root_node->setParent(cur_node);
                    }
                    else
                    {
                        cur_node->setRight(root_node);
                        root_node->setParent(cur_node);
                    }
                    root_node = cur_node;
                }
            }
        }
        // push into chosen set
        chosen_index.push_back(cur_index);
    }
    return;
}

void Floorplanner::perturb()
{
    perturb_type = rand() % 3;
    // perturb_type = rand() % 15;
    // if (perturb_type < 2)
    //     perturb_type = INSERT_DEL;
    // else if (perturb_type < 12)
    //     perturb_type = SWAP;
    // else
    //     perturb_type = ROTATE;
    _perturb_val1 = rand() % _blocknum; // rotate index, swap node 1, delete node
    _perturb_val2 = rand() % _blocknum; // swap node 2, insert node
    if (_perturb_val1 == _perturb_val2)
        _perturb_val2 = (_perturb_val1 + 1) % _blocknum;

    // print_tree();
    // cout << "Perturb " << perturb_type << " " << _perturb_val1 << " " << _perturb_val2 << endl;

    switch (perturb_type)
    {
    case ROTATE: // type 1: rotate a block
        _blockArray[_perturb_val1]->rotate();
        break;
    case INSERT_DEL: // type2 : insert and delete nodes
    {
        Node *node1 = _blockArray[_perturb_val1]->getNode();
        Node *node2 = _blockArray[_perturb_val2]->getNode();
        Node *parent_node = node1->getParent();
        Node *left_child = node1->getLeft();
        node1_is_right = (parent_node != NULL && parent_node->getRight() == node1);
        if (left_child == NULL)
        {
            Node *right_child = node1->getRight();
            if (right_child == NULL)
            { // it is leaf
                delete_type = 0;
                record_node = parent_node;
                node1_is_right ? parent_node->setRight(NULL) : parent_node->setLeft(NULL);
            }
            else
            { // it has only right child
                delete_type = 1;
                record_node = right_child;
                if (parent_node == NULL)
                {
                    root_node = right_child;
                    right_child->setParent(NULL);
                }
                else
                {
                    node1_is_right ? parent_node->setRight(right_child) : parent_node->setLeft(right_child);
                    right_child->setParent(parent_node);
                }
            }
        }
        else
        {
            Node *right_child = node1->getRight();
            if (right_child != NULL)
            { // has two child
                delete_type = 2;
                take_type = rand() % 2; // 0 for left, 1 for right
                if (take_type)          // take right
                {
                    record_node = right_child;
                    subtree_node = right_child->getLeft();
                    right_child->setParent(parent_node);
                    if (parent_node != NULL)
                        (node1_is_right) ? parent_node->setRight(right_child) : parent_node->setLeft(right_child);
                    else
                        root_node = right_child;
                    right_child->setLeft(left_child);
                    left_child->setParent(right_child);
                    if (subtree_node != NULL)
                    {
                        space_node = find_space(left_child, is_right);
                        subtree_node->setParent(space_node);
                        is_right ? space_node->setRight(subtree_node) : space_node->setLeft(subtree_node);
                    }
                }
                else // take left
                {
                    record_node = left_child;
                    subtree_node = left_child->getRight();
                    left_child->setParent(parent_node);
                    if (parent_node != NULL)
                        (node1_is_right) ? parent_node->setRight(left_child) : parent_node->setLeft(left_child);
                    else
                        root_node = left_child;
                    left_child->setRight(right_child);
                    right_child->setParent(left_child);
                    if (subtree_node != NULL)
                    {
                        space_node = find_space(right_child, is_right);
                        subtree_node->setParent(space_node);
                        is_right ? space_node->setRight(subtree_node) : space_node->setLeft(subtree_node);
                    }
                }
            }
            else
            { // it has only right child
                delete_type = 3;
                record_node = left_child;
                Node *parent_node = node1->getParent();
                if (parent_node == NULL)
                {
                    root_node = left_child;
                    left_child->setParent(NULL);
                }
                else
                {
                    node1_is_right ? parent_node->setRight(left_child) : parent_node->setLeft(left_child);
                    left_child->setParent(parent_node);
                }
            }
        }
        {                                  // insert
            take_type_insert = rand() % 2; // 0 for left, 1 for right
            if (take_type_insert)
            {
                Node *right_child = node2->getRight();
                node2->setRight(node1);
                node1->setParent(node2);
                if (right_child != NULL)
                {
                    node1->setRight(right_child);
                    right_child->setParent(node1);
                }
                else
                    node1->setRight(NULL);
                node1->setLeft(NULL);
            }
            else
            {
                Node *left_child = node2->getLeft();
                node2->setLeft(node1);
                node1->setParent(node2);
                if (left_child != NULL)
                {
                    node1->setLeft(left_child);
                    left_child->setParent(node1);
                }
                else
                    node1->setLeft(NULL);
                node1->setRight(NULL);
            }
        }
        break;
    }
    case SWAP: // type 3: swap two nodes
        // check if the swap node is the same node
        // get the node
        {
            Node *node1 = _blockArray[_perturb_val1]->getNode();
            Node *node2 = _blockArray[_perturb_val2]->getNode();
            // re-assign if one of them is root
            if (node1 == root_node)
                root_node = node2;
            else if (node2 == root_node)
                root_node = node1;
            // check if one swap node is the parent of the other
            char parent_child_relation = 0;
            if (node2->getParent() == node1)
            {
                if (node1->getLeft() == node2)
                    parent_child_relation = 1;
                else
                    parent_child_relation = 2;
            }
            else if (node1->getParent() == node2)
            {
                if (node2->getLeft() == node1)
                    parent_child_relation = 3;
                else
                    parent_child_relation = 4;
            }
            // swap parent
            Node *swap1 = node1->getParent();
            Node *swap2 = node2->getParent();
            node1->setParent(swap2);
            if (swap2 != NULL)
                (swap2->getRight() == node2) ? swap2->setRight(node1) : swap2->setLeft(node1);
            node2->setParent(swap1);
            if (swap1 != NULL)
                (swap1->getRight() == node1) ? swap1->setRight(node2) : swap1->setLeft(node2);
            // swap left
            swap1 = node1->getLeft();
            swap2 = node2->getLeft();
            node1->setLeft(swap2);
            if (swap2 != NULL)
                swap2->setParent(node1);
            node2->setLeft(swap1);
            if (swap1 != NULL)
                swap1->setParent(node2);
            // swap right
            swap1 = node1->getRight();
            swap2 = node2->getRight();
            node1->setRight(swap2);
            if (swap2 != NULL)
                swap2->setParent(node1);
            node2->setRight(swap1);
            if (swap1 != NULL)
                swap1->setParent(node2);
            // re-assign parent_child condition
            switch (parent_child_relation)
            {
            case 1:
                node1->setParent(node2);
                node2->setLeft(node1);
                break;
            case 2:
                node1->setParent(node2);
                node2->setRight(node1);
                break;
            case 3:
                node2->setParent(node1);
                node1->setLeft(node2);
                break;
            case 4:
                node2->setParent(node1);
                node1->setRight(node2);
                break;
            }
            break;
        }
    }
    // print_tree();
}

Node *Floorplanner::find_space(Node *space_node, bool &is_right)
{
    while (true)
    {
        Node *left_child = space_node->getLeft();
        Node *right_child = space_node->getRight();
        if (left_child == NULL)
        {
            is_right = false;
            return space_node;
        }
        if (right_child == NULL)
        {
            is_right = true;
            return space_node;
        }
        space_node = (rand() % 2) ? left_child : right_child;
    }
}

double Floorplanner::calculate_total_cost()
{
    int max_x, max_y, area;
    double wire;
    calculate_area_cost(max_x, max_y, area);
    calculate_wire_length_cost(wire);
    // calculate intersect area
    int maxa_x = (max_x > _outline.first) ? max_x : _outline.first;
    int maxa_y = (max_y > _outline.second) ? max_y : _outline.second;
    double overflow_area_ratio = ((max_x > _outline.first) || (max_y > _outline.second)) ? double(maxa_x * maxa_y) / double(_outline.first * _outline.second) : 0;
    // calculate cost
    double cost = _alpha * double(max_x * max_y) / _area_norm_factor + (1 - _alpha) * wire / _wire_length_norm_factor;
    double cost_ratio = 0.5 + (1 - _beta);
    cost_ratio = cost_ratio > 0.9 ? 0.9 : cost_ratio;
    cost = cost * (1 - cost_ratio) + overflow_area_ratio * cost_ratio;
    return cost;
}

double Floorplanner::calculate_total_cost(int &area, double &wire, int &max_x, int &max_y)
{
    calculate_area_cost(max_x, max_y, area);
    calculate_wire_length_cost(wire);
    // calculate intersect area
    int maxa_x = (max_x > _outline.first) ? max_x : _outline.first;
    int maxa_y = (max_y > _outline.second) ? max_y : _outline.second;
    double overflow_area_ratio = ((max_x > _outline.first) || (max_y > _outline.second)) ? double(maxa_x * maxa_y) / double(_outline.first * _outline.second) : 0;
    // calculate cost
    double cost = _alpha * double(max_x * max_y) / _area_norm_factor + (1 - _alpha) * wire / _wire_length_norm_factor;
    double cost_ratio = 0.5 + (1 - _beta);
    cost_ratio = cost_ratio > 0.9 ? 0.9 : cost_ratio;
    cost = cost * (1 - cost_ratio) + overflow_area_ratio * cost_ratio;
    return cost;
}

void Floorplanner::update_node()
{
    // initilize root
    _blockArray[root_node->getId()]->setx(0);
    _blockArray[root_node->getId()]->sety(0);
    update_x(root_node);
    // initilize contour
    _contour->clear();
    _contour->initial_contour();
    update_y(root_node);
    // print_tree();
}

void Floorplanner::update_x(Node *cur_node)
{
    // traversing order: self > left = right
    Node *right_node = cur_node->getRight();
    Node *left_node = cur_node->getLeft();
    Block *cur_block = _blockArray[cur_node->getId()];
    int x_value = cur_block->getx();
    int x_size = cur_block->get_sizex();
    if (right_node != NULL)
    {
        _blockArray[right_node->getId()]->setx(x_value);
        update_x(right_node);
    }
    if (left_node != NULL)
    {
        _blockArray[left_node->getId()]->setx(x_value + x_size);
        update_x(left_node);
    }
}

void Floorplanner::update_y(Node *cur_node)
{
    // traversing order: parent > left > right
    _contour->insert(_blockArray[cur_node->getId()]);
    Node *right_node = cur_node->getRight();
    Node *left_node = cur_node->getLeft();
    if (left_node != NULL)
        update_y(left_node);
    if (right_node != NULL)
        update_y(right_node);
}

void Floorplanner::calculate_area_cost(int &max_x, int &max_y, int &area)
{
    max_x = 0;
    for (size_t i = 0; i < _blocknum; ++i)
    {
        int max_corner_x = _blockArray[i]->getx() + _blockArray[i]->get_sizex();
        if (max_corner_x > max_x)
            max_x = max_corner_x;
    }
    max_y = _contour->getmaxy();
    area = max_x * max_y;
    return;
}

void Floorplanner::calculate_wire_length_cost(double &wire_cost)
{
    wire_cost = 0;
    for (size_t i = 0; i < _netnum; ++i)
    {
        wire_cost += _netArray[i]->calcHPWL();
    }
}

void Floorplanner::reset_node()
{
    for (size_t i = 0; i < _blocknum; ++i)
    {
        _blockArray[i]->getNode()->setLeft(NULL);
        _blockArray[i]->getNode()->setRight(NULL);
        _blockArray[i]->getNode()->setParent(NULL);
    }
}

void Floorplanner::print_tree()
{
    cout << endl;
    error_cnt = 0;
    cout << "====================== Tree =====================" << endl;
    print_node(root_node, 0);
    cout << "=================================================" << endl;
    cout << endl;
    if (error_cnt != _blocknum) // for debugging (the program will not end if loops occurs in the tree)
        exit(-1);
}

void Floorplanner::print_node(Node *cur_node, int level)
{
    if (level > _blocknum)
        assert(0);
    for (int i = 0; i < level; ++i)
        cout << "         ";
    if (cur_node == NULL)
    {
        cout << "x" << endl;
        return;
    }
    else
    {
        cout << cur_node->getId() << "(" << setw(5) << _blockArray[cur_node->getId()]->getx() << "," << setw(5) << _blockArray[cur_node->getId()]->gety() << ")" << (_blockArray[cur_node->getId()]->getRotate() ? "R" : "") << endl;
        ++error_cnt;
    }
    print_node(cur_node->getLeft(), level + 1);
    print_node(cur_node->getRight(), level + 1);
}

void Floorplanner::printSummary() const
{
    cout << endl;
    cout << "==================== Summary ====================" << endl;
    cerr << " Alpha value:   " << _alpha << endl;
    cerr << " Outline Size:   " << _outline.first << " " << _outline.second << endl;
    cout << " Total block number: " << _blocknum << endl;
    cout << " Total pin number: " << _pinnum << endl;
    cout << " Total net number:  " << _netnum << endl;
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Floorplanner::reportNet() const
{
    cout << endl;
    cout << "================= Report Nets ===================" << endl;
    cout << "Number of nets: " << _netnum << endl;
    // for (size_t i = 0, end_i = _netArray.size(); i < end_i; ++i)
    //     _netArray[i]->print_net();
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Floorplanner::reportBlock() const
{
    cout << endl;
    cout << "================ Report Blocks ==================" << endl;
    cout << "Number of nets: " << _blocknum << endl;
    for (size_t i = 0, end_i = _blockArray.size(); i < end_i; ++i)
        _blockArray[i]->print();
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Floorplanner::reportPin() const
{
    cout << endl;
    cout << "================= Report Pins ===================" << endl;
    cout << "Number of nets: " << _pinnum << endl;
    for (size_t i = 0, end_i = _pinArray.size(); i < end_i; ++i)
        _pinArray[i]->print();
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Floorplanner::writeResult(fstream &outFile)
{
    // <final cost>
    // // Cost = αA + (1-α)W
    // <total wirelength>
    // // W = sum of HPWL
    // <chip_area>
    // // area = (chip_width) * (chip_height)
    // <chip_width> <chip_height>
    // //resulting chip width and height
    // <program_runtime>
    // //report the runtime in seconds
    // <macro_name> <x1> <y1> <x2> <y2>
    // <macro_name> <x1> <y1> <x2> <y2>
    // // (x1, y1): lower-left corner, (x2, y2): upper-right corner
    // … More macros
    cout << "================= Write Results =================" << endl;
// #ifdef PRINT_MODE
    int total_area = 0;
    for (size_t i = 0; i < _blocknum; ++i)
    {
        Block *cur_block = _blockArray[i];
        total_area += cur_block->getW() * cur_block->getH();
    }
    cout << "Total area: " << total_area << endl;
    cout << "Dead space: " << 1 - double(total_area) / double(my_best_area) << endl;
// #endif
    int num_of_pin_in_net = 0;
    for (int i = 0; i < _netnum; ++i)
        num_of_pin_in_net += _netArray[i]->getsize();
    cout << "Number of pins in net: " << num_of_pin_in_net << '\n';
    cout << "Number of steps: " << _step_cnt << '\n';
    cout << "Number of blocks: " << _blocknum << '\n';
    cout << "Number of pins: " << _pinnum << '\n';
    cout << "Number of nets: " << _netnum << '\n';
    cout << "Number of iterations: " << _iteration_cnt << '\n';
    cout << "Cost: " << float(my_best_area) * _alpha + my_best_wire * (1 - _alpha) << '\n';
    cout << "Wire: " << fixed << setprecision(3) << my_best_wire << '\n';
    cout << "Area: " << my_best_area << '\n';
    outFile << fixed << setprecision(3) << float(my_best_area) * _alpha + my_best_wire * (1 - _alpha) << '\n';
    outFile << fixed << setprecision(3) << my_best_wire << '\n';
    outFile << my_best_area << '\n';
    outFile << my_best_W << " " << my_best_H << '\n';
    outFile << fixed << setprecision(3) << get_time() << '\n';
    cout << "Runtime: " << fixed << setprecision(3) << get_time() << '\n';
    recover_best_nodes();
    update_node();
    for (size_t i = 0; i < _blocknum; ++i)
    {
        Block *cur_block = _blockArray[i];
        outFile << left << setw(8) << cur_block->getName() << setw(8) << cur_block->getx() << setw(8) << cur_block->gety()
                << setw(8) << cur_block->getx() + cur_block->get_sizex() << setw(8) << cur_block->gety() + cur_block->get_sizey() << endl;
    }
    cout << "=================================================" << endl;
}

void Floorplanner::keep_best_cost(double &cost, int &area, double &wire, int &W, int &H)
{
    // record best cost
    my_best_cost = cost;
    my_best_wire = wire;
    my_best_area = area;
    my_best_W = W;
    my_best_H = H;
    return;
}

void Floorplanner::keep_best_nodes()
{
    _best_node_array.clear(); // clear prvious best solution
    my_best_root_node = root_node;
    // cout << "Keep best node" << endl;
    for (size_t i = 0; i < _blocknum; ++i)
    {
        Node temp = *(_blockArray[i]->getNode()); // copy the dynamic object to static
        _best_node_array.push_back(temp);
        // cout << "(" << _blockArray[i]->getNode()->getId() << "," << _blockArray[i]->getx() << "," << _blockArray[i]->gety() << ") " << endl;
        // cout << "(" << _blockArray[i]->getNode()->getId() << "," << _blockArray[i]->getNode()->getLeft() << "," << _blockArray[i]->getNode()->getRight() << ") " << endl;
        // _blockArray[temp.getId()]->print();
    }
    // cout << endl;
}

void Floorplanner::recover_best_nodes()
{
    // cout << "Recover best node" << endl;
    root_node = my_best_root_node;
    for (size_t i = 0; i < _blocknum; ++i)
    {
        Node &temp = _best_node_array[i];
        Node *replace_node = _blockArray[i]->getNode();
        replace_node->setParent(temp.getParent());
        replace_node->setRight(temp.getRight());
        replace_node->setLeft(temp.getLeft());
        replace_node->setRotate(temp.getRotate());
#ifdef DEBUG_MODE
        // cout << "(" << replace_node->getId() << "," << temp.getId() << ") " << endl;
        // cout << "(" << _blockArray[replace_node->getId()]->getNode()->getId() << "," << _blockArray[replace_node->getId()]->getx() << "," << _blockArray[replace_node->getId()]->gety() << ") " << endl;
        // cout << "(" << _blockArray[replace_node->getId()]->getNode()->getId() << "," << _blockArray[replace_node->getId()]->getNode()->getLeft() << "," << _blockArray[replace_node->getId()]->getNode()->getRight() << ") " << endl;
        // _blockArray[temp.getId()]->print();
        assert(replace_node->getId() == temp.getId());
#endif
    }
    // cout << endl;
}

void Floorplanner::clear()
{
    delete _contour;
    for (size_t i = 0; i < _blocknum; ++i)
        delete _blockArray[i];
    for (size_t i = 0; i < _pinnum; ++i)
        delete _pinArray[i];
    for (size_t i = 0; i < _netnum; ++i)
        delete _netArray[i];
}

void Floorplanner::deperturb()
{
    switch (perturb_type)
    {
    case ROTATE: // type 1: rotate a block
        _blockArray[_perturb_val1]->rotate();
        break;
    case INSERT_DEL: // type2 : insert and delete nodes
    {
        Node *node1 = _blockArray[_perturb_val1]->getNode();
        Node *node2 = _blockArray[_perturb_val2]->getNode();
        { // 0 for left, 1 for right
            if (take_type_insert)
            {
                Node *child_node = node1->getRight();
                node2->setRight(child_node);
                if (child_node != NULL)
                    child_node->setParent(node2);
            }
            else
            {
                Node *child_node = node1->getLeft();
                node2->setLeft(child_node);
                if (child_node != NULL)
                    child_node->setParent(node2);
            }
        }
        {
            switch (delete_type)
            {
            case 0:
            {
                (node1_is_right) ? record_node->setRight(node1) : record_node->setLeft(node1);
                node1->setParent(record_node);
                node1->setLeft(NULL);
                node1->setRight(NULL);
                break;
            }
            case 1:
            {
                Node *parent_node = record_node->getParent();
                node1->setParent(parent_node);
                node1->setLeft(NULL);
                node1->setRight(record_node);
                record_node->setParent(node1);
                if (parent_node == NULL)
                    root_node = node1;
                else
                    (node1_is_right) ? parent_node->setRight(node1) : parent_node->setLeft(node1);
                break;
            }
            case 2:
            {
                Node *parent_node = record_node->getParent();
                if (take_type) // take right
                {
                    if (subtree_node != NULL)
                    {
                        (is_right) ? space_node->setRight(NULL) : space_node->setLeft(NULL);
                        subtree_node->setParent(record_node);
                    }
                    node1->setParent(parent_node);
                    if (parent_node == NULL)
                        root_node = node1;
                    else
                        (node1_is_right) ? parent_node->setRight(node1) : parent_node->setLeft(node1);
                    Node *left_of_record_node = record_node->getLeft();
                    node1->setLeft(left_of_record_node);
                    left_of_record_node->setParent(node1);
                    node1->setRight(record_node);
                    record_node->setParent(node1);
                    record_node->setLeft(subtree_node);
                }
                else
                {
                    if (subtree_node != NULL)
                    {
                        (is_right) ? space_node->setRight(NULL) : space_node->setLeft(NULL);
                        subtree_node->setParent(record_node);
                    }
                    node1->setParent(parent_node);
                    if (parent_node == NULL)
                        root_node = node1;
                    else
                        (node1_is_right) ? parent_node->setRight(node1) : parent_node->setLeft(node1);
                    Node *right_of_the_record_node = record_node->getRight();
                    node1->setRight(right_of_the_record_node);
                    right_of_the_record_node->setParent(node1);
                    node1->setLeft(record_node);
                    record_node->setParent(node1);
                    record_node->setRight(subtree_node);
                }
                break;
            }
            case 3:
            {
                Node *parent_node = record_node->getParent();
                node1->setParent(parent_node);
                node1->setRight(NULL);
                node1->setLeft(record_node);
                record_node->setParent(node1);
                if (parent_node == NULL)
                {
                    root_node = node1;
                }
                else
                {
                    (node1_is_right) ? parent_node->setRight(node1) : parent_node->setLeft(node1);
                }
                break;
            }
            }
        }
        break;
    }
    case SWAP: // type 3: swap two nodes
        // check if the swap node is the same node
        // get the node
        {
            Node *node1 = _blockArray[_perturb_val1]->getNode();
            Node *node2 = _blockArray[_perturb_val2]->getNode();
            // re-assign if one of them is root
            if (node1 == root_node)
                root_node = node2;
            else if (node2 == root_node)
                root_node = node1;
            // check if one swap node is the parent of the other
            char parent_child_relation = 0;
            if (node2->getParent() == node1)
            {
                if (node1->getLeft() == node2)
                    parent_child_relation = 1;
                else
                    parent_child_relation = 2;
            }
            else if (node1->getParent() == node2)
            {
                if (node2->getLeft() == node1)
                    parent_child_relation = 3;
                else
                    parent_child_relation = 4;
            }
            // swap parent
            Node *swap1 = node1->getParent();
            Node *swap2 = node2->getParent();
            node1->setParent(swap2);
            if (swap2 != NULL)
                (swap2->getRight() == node2) ? swap2->setRight(node1) : swap2->setLeft(node1);
            node2->setParent(swap1);
            if (swap1 != NULL)
                (swap1->getRight() == node1) ? swap1->setRight(node2) : swap1->setLeft(node2);
            // swap left
            swap1 = node1->getLeft();
            swap2 = node2->getLeft();
            node1->setLeft(swap2);
            if (swap2 != NULL)
                swap2->setParent(node1);
            node2->setLeft(swap1);
            if (swap1 != NULL)
                swap1->setParent(node2);
            // swap right
            swap1 = node1->getRight();
            swap2 = node2->getRight();
            node1->setRight(swap2);
            if (swap2 != NULL)
                swap2->setParent(node1);
            node2->setRight(swap1);
            if (swap1 != NULL)
                swap1->setParent(node2);
            // re-assign parent_child condition
            switch (parent_child_relation)
            {
            case 1:
                node1->setParent(node2);
                node2->setLeft(node1);
                break;
            case 2:
                node1->setParent(node2);
                node2->setRight(node1);
                break;
            case 3:
                node2->setParent(node1);
                node1->setLeft(node2);
                break;
            case 4:
                node2->setParent(node1);
                node1->setRight(node2);
                break;
            }
            break;
        }
    }
    // print_tree();
}