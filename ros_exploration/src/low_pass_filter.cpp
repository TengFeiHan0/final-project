#include "low_pass_filter.h"

low_pass_filter::low_pass_filter(QObject *parent) : QObject(parent)
{
    filter_coeff = 0;
    data = data_last = 0;
    DT = 0;
    F_pass = 0;
    deviation = 0;
}

low_pass_filter::~low_pass_filter()
{

}

bool low_pass_filter::filter(float data_in)
{
    if(DT!=0)
    {
        if(filter_coeff == 0 && F_pass!=0)
            filter_coeff = DT/(DT+1/(2*3.1415*F_pass));

        if(filter_coeff!=0)
        {
            data = data_last + filter_coeff*(data_in-data_last);
            deviation = data_in - data_last;
            data_last = data;
            return 1;
        }
        else
        {
          data = 0;
          return 0;
        }
    }

}

