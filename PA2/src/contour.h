#ifndef CONTOUR_H
#define CONTOUR_H

#include <vector>
#include "block.h"
#include <cassert>
using namespace std;

class Contour_node
{
    friend class Contour;

public:
    // Constructor and destructor
    // Contour_node(int x, int y) : _prev(NULL), _next(NULL), _center(make_pair(x, y)) {}
    Contour_node(int x, int y) : _next(NULL), _center(make_pair(x, y)) {}
    ~Contour_node() {}

    // Basic access methods
    // Contour_node *getPrev() const { return _prev; }
    Contour_node *getNext() const { return _next; }
    int getx() const { return _center.first; }
    int gety() const { return _center.second; }

    // Set functions
    // void setPrev(Contour_node *prev) { _prev = prev; }
    void setNext(Contour_node *next) { _next = next; }
    void sety(int y) { _center.second = y; }

private:
    // Contour_node *_prev;
    Contour_node *_next;
    pair<int, int> _center;
};
class Contour
{
public:
    // Constructor and destructor
    Contour() : _head(NULL)
    {
    }
    ~Contour()
    {
        clear();
    }

    // Basic access methods
    void print_contour()
    {
        if (_head == NULL)
            cout << "The contour is empty !" << endl;
        Contour_node *cur_node = _head;
        while (true)
        {
            cout << "(" << setw(5) << cur_node->getx() << "," << setw(5) << cur_node->gety() << ")"
                 << "--->";
            Contour_node *next_node = cur_node->getNext();
            if (next_node != NULL)
                cur_node = next_node;
            else
            {
                cout << "\b\b\b\b    " << endl;
                return;
            }
        }
    }

    // insert functions
    void insert(Block *cur_block)
    {
        // let x of cur block is a~b
        // in link list find (c, d) s.t. c >= a, if c = a : there must be (c, xxx) remove that until the next check
        //                                       else if c > a : it must not happen
        // in link list find (e, f) s.t. e>=b, if e = b : there must be (e, xxx) save it but remove (e, f)
        //                                     else if e > b: add one more (b, f)
        // when remove the intermediate node, maintain the max y
        // add (a, maxy + height) and (b, maxy + height) into the list
        int x_begin = cur_block->getx();
        int x_end = x_begin + cur_block->get_sizex();
        Contour_node *new_node_begin = new Contour_node(x_begin, 0);
        Contour_node *new_node_end = new Contour_node(x_end, 0);
        new_node_begin->setNext(new_node_end);
        // new_node_end->setPrev(new_node_begin);
        if (_head == NULL)
            return;
        Contour_node *cur_node = _head;
        bool find = false;
        int max_y = 0;
        while (true)
        {
            Contour_node *next_node = cur_node->getNext();
            if (!find)
            {
                if (cur_node->getx() == x_begin)
                {
                    cur_node->setNext(new_node_begin);
                    // new_node_begin->setPrev(cur_node);
                    find = true;
                }
                // else if (cur_node->getx() > x_begin)
                //     assert(0);
            }
            else
            {
                if (cur_node->getx() > x_end)
                {
                    Contour_node *intermediate_node = new Contour_node(x_end, cur_node->gety());
                    new_node_end->setNext(intermediate_node);
                    // intermediate_node->setPrev(new_node_end);
                    intermediate_node->setNext(cur_node);
                    // cur_node->setPrev(intermediate_node);
                    break;
                }
                else if (cur_node->getx() == x_end)
                {
                    new_node_end->setNext(next_node);
                    // next_node->setPrev(new_node_end);
                    delete cur_node;
                    break;
                }
                // updata max y value
                if (cur_node->gety() > max_y)
                    max_y = cur_node->gety();
                delete cur_node;
            }
            cur_node = next_node;
        }
        int block_height = cur_block->get_sizey();
        cur_block->sety(max_y);
        new_node_begin->sety(max_y + block_height);
        new_node_end->sety(max_y + block_height);
    }
    void initial_contour()
    {
        _head = new Contour_node(0, 0);
        Contour_node *end = new Contour_node(2147483647, 0);
        _head->setNext(end);
        // end->setPrev(_head);
    }
    void clear()
    {
        if (_head == NULL)
            return;
        Contour_node *cur_node = _head;
        while (true)
        {
            Contour_node *next_node = cur_node->getNext();
            delete cur_node;
            if (next_node != NULL)
                cur_node = next_node;
            else
                return;
        }
    }
    int getmaxy()
    {
        int max_y = 0;
        if (_head == NULL)
            return max_y;
        Contour_node *cur_node = _head;
        while (true)
        {
            Contour_node *next_node = cur_node->getNext();
            if (cur_node->gety() > max_y)
                max_y = cur_node->gety();
            if (next_node != NULL)
                cur_node = next_node;
            else
                return max_y;
        }
    }

private:
    Contour_node *_head; // the head of the double link list (actually single link is enough)
};

#endif // CONTOUR_H
