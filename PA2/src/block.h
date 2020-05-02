#ifndef BLOCK_H
#define BLOCK_H

#include <vector>
#include <iomanip>
using namespace std;

class Pin
{
public:
    // Constructor and destructor
    Pin(string &name, int center_x, int center_y) : _center_x(center_x), _center_y(center_y), _name(name) {}
    ~Pin() {}

    // Basic access methods
    string getName() const { return _name; }
    virtual double get_center_x() const { return _center_x; }
    virtual double get_center_y() const { return _center_y; }

    // Set functions
    void setName(const string name) { _name = name; }

    // print functions
    virtual void print()
    {
        cout << "Name: " << setw(6) << _name << ", Center: (" << setw(5) << _center_x << "," << setw(5) << _center_y
             << ")" << endl;
    }

protected:
    int _center_x; // x-coord of the pin
    int _center_y; // y-coord of the pin
    string _name;  // name of the block
};

class Node
{
    friend class Block;

public:
    // Constructor and destructor
    Node(const int &id) : _rotate(false), _id(id), _left(NULL), _right(NULL) {}
    ~Node() {}

    // Basic access methods
    int getId() const { return _id; }
    Node *getLeft() const { return _left; }
    Node *getRight() const { return _right; }
    Node *getParent() const { return _parent; }

    // Set functions
    void setId(const int &id) { _id = id; }
    void setLeft(Node *left) { _left = left; }
    void setRight(Node *right) { _right = right; }
    void setParent(Node *parent) { _parent = parent; }
    void setRotate(bool rot) { _rotate = rot; }
    void rotate() { _rotate = !_rotate; }
    bool getRotate() const { return _rotate; }

private:
    int _id;      // id of the node (indicating the block)
    Node *_left;  // pointer to the left node
    Node *_right; // pointer to the right node
    Node *_parent;
    bool _rotate; // whether the block is rotated
};

class Block : public Pin
{
public:
    // Constructor and destructor
    Block(string &name, int W, int H, int id) : _W(W), _H(H), Pin(name, 0, 0)
    {
        _node = new Node(id);
    }
    ~Block() {}

    // Basic access methods
    Node *getNode() const { return _node; }
    bool getRotate() const { return _node->getRotate(); }
    // string getName() const { return _name; }
    // vector<int> getNetList() const { return _netList; }
    int getW() const { return _W; }
    int getH() const { return _H; }
    int getx() const { return _X; }
    int gety() const { return _Y; }
    int get_sizex() const { return (_node->getRotate() ? _H : _W); }
    int get_sizey() const { return (_node->getRotate() ? _W : _H); }
    double get_center_x() const { return double(get_sizex()) / 2 + double(_X); }
    double get_center_y() const { return double(get_sizey()) / 2 + double(_Y); }

    // Set functions
    void setx(int X)
    {
        _X = X;
    }
    void sety(int Y)
    {
        _Y = Y;
    }
    void setNode(Node *node) { _node = node; }

    // Modify methods
    void rotate() { _node->rotate(); }
    // void addNet(const int netId) { _netList.push_back(netId); }
    void print()
    {
        cout << "Name: " << setw(6) << _name << ", Corner: (" << setw(5) << _X << "," << setw(5) << _Y
             << ")"
             << " , Size: (" << setw(5) << _W << "," << setw(5) << _H << "), BlockId: " << setw(3) << _node->getId();
        cout << endl;
    }

private:
    int _H;
    int _W;
    int _X;
    int _Y;
    Node *_node; // node used to link the cells together
};

#endif // BLOCK_H
