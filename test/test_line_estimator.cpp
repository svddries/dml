#include <iostream>


#include "dml/line_estimator.h"

int main(int argc, char **argv)
{

    dml::LineEstimator estimator;
    estimator.setWindowSize(10);

    double x = 10;
    double y = 123;
    for(unsigned int i = 0; i < 100; ++i)
    {
        estimator.addPoint(x, y);

        std::cout << i << ": " << x << ", " << y << ": ";

        double a, b;
        if (estimator.calculateLineEstimate(&a, &b))
            std::cout << "a = " << a << ", b = " << b;

        std::cout << std::endl;

        x += i;
        y += i * 3;
    }

    return 0;
}
