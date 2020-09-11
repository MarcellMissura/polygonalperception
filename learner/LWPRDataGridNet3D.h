#ifndef LWPRDATAGRID3D_H_
#define LWPRDATAGRID3D_H_
#include "DataGridNet3D.h"
#include <lwpr.hh>

// The LWPRDataGrid extends the DataGrid with LWPR training capabilities.

class LWPRDataGridNet3D : public DataGridNet3D
{
    LWPR_Object* lwpr;

public:

    LWPRDataGridNet3D();
    ~LWPRDataGridNet3D(){}

    void train();
    void drawReceptiveFields();
};

#endif /* LWPRDATAGRID3D_H_ */
