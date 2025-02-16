#ifndef NET_H
#define NET_H

#include <vector>
using namespace std;

class Net
{
public:
    // constructor and destructor
    Net(string& name) :
        _name(name) {
        _partCount[0] = 0; _partCount[1] = 0;
    }
    ~Net()  { }

    // basic access methods
    string getName()           const { return _name; }
    int getPartCount(int part) const { return _partCount[part]; }
    vector<int> getCellList()  const { return _cellList; }

    // set functions
    void setName(const string name) { _name = name; }
    void setPartCount(int part, const int count) { _partCount[part] = count; }

    // modify methods
    void incPartCount(int part)     { ++_partCount[part]; }
    void decPartCount(int part)     { --_partCount[part]; }
    void addCell(const int cellId)  { _cellList.push_back(cellId); }

    void print_net(){
        cout << endl;
        cout << "=================== Print Net ===================" << endl;
        cout << "Net name:         " << getName() << endl;
        cout << "Number of cells:  " << _cellList.size() << endl;
        cout << "Part counts:      " << _partCount[0] << " " << _partCount[1] << endl;
        cout << "Cell list" << endl;
        for(int i = 0 ;  i < _cellList.size() ; ++i) cout << _cellList[i] << " ";
        cout << endl;
        cout << "=================================================" << endl;
        cout << endl;
    }

private:
    int             _partCount[2];  // Cell number in partition A(0) and B(1)
    string          _name;          // Name of the net
    vector<int>     _cellList;      // List of cells the net is connected to
};

#endif  // NET_H
