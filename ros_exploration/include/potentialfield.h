#ifndef POTENTIALFIELD_H
#define POTENTIALFIELD_H

#include <QObject>
#include <QDebug>
#include "opencv2/opencv.hpp"

using namespace std;
typedef cv::Point3f  POINT;

#define POINT_0 POINT(0,0,0)

#define RAND_NUM (((float)rand()/RAND_MAX))

class PotentialField : public QObject
{
    struct state
    {
        float Obs_radius;
        vector<POINT> Obs_pos;
        float Obs_coeff;

        POINT Start_pos;
        float Start_radius;
        float Start_coeff;

        POINT Goal_pos;
        float Goal_radius;
        float Goal_coeff;

        float Border_radius;
        float Border_coeff;

        POINT Robot_pos;
        float step;

        float Max_Force;

        pair<POINT,POINT> admissible_area;

    };

    Q_OBJECT
public:
    explicit PotentialField(QObject *parent = 0);
    ~PotentialField();

    state space;

    bool RunPotentialPlanning(float THR, bool create_border_points);
    POINT CalcPlanningForce(void);
    void AddBorderPoints(float step);
    void AddBorderRandomPoints(int Num);
    float vec_len(POINT vec);

    vector<POINT> border_pos;
private:
     pair<POINT,POINT> last_admissible_area;


signals:

public slots:
};

#endif // POTENTIALFIELD_H
