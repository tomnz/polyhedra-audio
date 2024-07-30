#ifndef IAUDIO_H
#define IAUDIO_H

#include <vector>

using std::vector;

class IAudio {
public:
    IAudio() {}

    virtual void ingest(FLOAT_T *data)
    virtual vector<float> bandVals() = 0;
};

#endif
