#include <armadillo>
#include <geolib/datatypes.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

int NUM_POINTS = 100;

// ----------------------------------------------------------------------------------------------------

double random(double min, double max)
{
    return ((double)rand() / RAND_MAX) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform randomTransform()
{
    geo::Matrix3 b;
    b.setRPY(random(0, 3.1415), random(0, 3.1415), random(0, 3.1415));
    return geo::Transform(b, geo::Vector3(random(-10, 10), random(-10, 10), random(-10, 10)));
}

// ====================================================================================================
//                                            GENERATE DATA
// ====================================================================================================

void generateData(std::vector<geo::Vector3>& X, std::vector<geo::Vector3>& P, int N) {

    X.resize(N);
    P.resize(N);

    // initialize random seed
    srand (time(NULL));

    for(int i = 0; i < N; ++i) {
        X[i] = geo::Vector3(random(-10, 10), random(-10, 10), random(-10, 10));
    }

    geo::Transform t = randomTransform();

    for(int i = 0; i < N; ++i) {
        P[i] = t * X[i];
    }
}

// ====================================================================================================
//                                                ICP
// ====================================================================================================

void icp(const std::vector<geo::Vector3>& X, const std::vector<geo::Vector3>& P, geo::Transform& t)
{
    int n = X.size();

    // - - - - - - - - - - - - - - CALCULATE CENTERS OF MASS - - - - - - - - - - - - - -

    geo::Vector3 uX(0, 0, 0);
    geo::Vector3 uP(0, 0, 0);

    for(int i = 0; i < n; ++i) {
        uX += X[i];
        uP += P[i];
    }

    uX /= n;
    uP /= n;

    // - - - - - - - - - - - - - - CENTRALIZE DATASETS - - - - - - - - - - - - - -

    // Subtract the corresponding center of mass from every point in the two point sets

    std::vector<geo::Vector3> Xp(n);
    std::vector<geo::Vector3> Pp(n);

    for(int i = 0; i < n; ++i) {
        Xp[i] = X[i] - uX;
        Pp[i] = P[i] - uP;
    }


    // - - - - - - - - - - - - SVD TO DETERMINE ROTATION MATRIX  - - - - - - - - - - - -

    arma::mat W(3, 3);
    W.zeros();

    for(int i = 0; i < n; ++i) {
        for(int k = 0; k < 3; ++k) {
            for(int l = 0; l < 3; ++l) {
                W(k, l) += Xp[i].m[k] * Pp[i].m[l];
            }
        }
    }

    arma::mat U(3, 3);
    arma::mat V(3, 3);
    arma::vec s(3);

    arma::svd(U, s, V, W);
    arma::mat R_arma = U * V.t();


    // - - - - - - - - - - - - CREATE TRANSFORMATION  - - - - - - - - - - - -

    // convert rotation matrix to correct type
    geo::Matrix3 R;
    for(int k = 0; k < 3; ++k) {
        for(int l = 0; l < 3; ++l) {
            R(k, l) = R_arma(k, l);
        }
    }

    // calculate translation
    geo::Vector3 trans = uX - R * uP;

    // create transformation from translation and rotation
    t = geo::Transform(R, trans);
}

// ====================================================================================================
//                                           ERROR CALCULATION
// ====================================================================================================

double calculateError(const std::vector<geo::Vector3>& X, const std::vector<geo::Vector3>& P, const geo::Transform& t)
{
    double error = 0;
    for(unsigned int i = 0; i < X.size(); ++i) {
        geo::Vector3 p = t * P[i];
        error += (X[i] - p).length2();
    }

    return error / X.size();
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // init data structures
    std::vector<geo::Vector3> X;
    std::vector<geo::Vector3> P;

    // generate data
    generateData(X, P, NUM_POINTS);

    // icp
    geo::Transform t;
    icp(X, P, t);

    // calculate error
    double error = calculateError(X, P, t);

    // print error
    std::cout << "MSE = " << error << std::endl;

    return 0;
}
