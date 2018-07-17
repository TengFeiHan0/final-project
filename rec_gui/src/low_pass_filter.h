#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <QObject>
#include <QDebug>

class low_pass_filter : public QObject
{
    Q_OBJECT
public:
    explicit low_pass_filter(QObject *parent = 0);
    ~low_pass_filter();
    float data,data_last;
    float DT;
    float F_pass;

    bool filter(float data_in);

private:
    float filter_coeff;

signals:

public slots:
};

#endif // LOW_PASS_FILTER_H
