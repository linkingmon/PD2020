#ifndef NET_H
#define NET_H

#include <vector>
using namespace std;

class Net
{
public:
    // constructor and destructor
    Net() {}
    ~Net() {}

    // basic access methods
    vector<Pin *> getBlockList() const { return _blockList; }
    size_t getsize() { return _blockList.size(); }

    // modify methods
    void addBlock(Pin *pin) { _blockList.push_back(pin); }

    // other member functions
    double calcHPWL()
    {
        double min_x = 1e10;
        double min_y = 1e10;
        double max_x = -1e10;
        double max_y = -1e10;
        for (size_t i = 0, end = _blockList.size(); i < end; ++i)
        {
            double center_x = _blockList[i]->get_center_x();
            double center_y = _blockList[i]->get_center_y();
            if (center_x > max_x)
                max_x = center_x;
            if (center_y > max_y)
                max_y = center_y;
            if (center_x < min_x)
                min_x = center_x;
            if (center_y < min_y)
                min_y = center_y;
        }
        return (max_x - min_x) + (max_y - min_y);
    }

private:
    vector<Pin *> _blockList; // List of blocks the net is connected to
};

#endif // NET_H
