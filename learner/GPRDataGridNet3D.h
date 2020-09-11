#ifndef GPRDATAGRID3D_H_
#define GPRDATAGRID3D_H_
#include "DataGridNet3D.h"
#include "learner/GPR.h"
#include "framework/Config.h"

// The GPRDataGrid extends the DataGrid with GPR training capabilities.

class GPRDataGridNet3D : public DataGridNet3D
{

public:

    GPRDataGridNet3D();
    ~GPRDataGridNet3D(){}

    void train();
    void trainAll();
};

#endif /* GPRDATAGRID3D_H_ */
