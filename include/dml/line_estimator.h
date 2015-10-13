#ifndef DML_LINE_ESTIMATOR_H_
#define DML_LINE_ESTIMATOR_H_

#include <iostream>
#include <vector>

namespace dml
{

template<typename T>
class RingBuffer
{

public:

    RingBuffer() {}

    void setSize(unsigned int max_size)
    {
        if (max_size > buffer_.size())
            filled_ = false;

        buffer_.resize(max_size);
        i_latest_ = 0;
        i_oldest_ = 0;
    }

    void add(const T& v)
    {
        buffer_[i_oldest_] = v;
        i_latest_ = i_oldest_;

        ++i_oldest_;
        if (i_oldest_ >= buffer_.size())
        {
            i_oldest_ = 0;
            filled_ = true;
        }
    }

    const T& oldest() const
    {
        return buffer_[i_oldest_];
    }

    const T& latest() const
    {
        return buffer_[i_latest_];
    }

    bool filled() const
    {
        return filled_;
    }

private:

    bool filled_;

    std::vector<T> buffer_;

    unsigned int i_latest_;

    unsigned int i_oldest_;

};

class LineEstimator
{

public:

    LineEstimator() : x_sum_(0), y_sum_(0), xy_sum_(0), x2_sum_(0) {}

    ~LineEstimator() {}

    void setWindowSize(unsigned int w)
    {
        x_.setSize(w);
        y_.setSize(w);
        window_size_ = w;
    }

    bool filled() const
    {
        return x_.filled();
    }

    void latest(double* x, double* y)
    {
        *x = x_.latest();
        *y = y_.latest();
    }

    void addPoint(double x, double y)
    {
        if (x_.filled())
        {
            x_sum_ -= x_.oldest();
            y_sum_ -= y_.oldest();
            xy_sum_ -= (x_.oldest() * y_.oldest());
            x2_sum_ -= (x_.oldest() * x_.oldest());
        }

        x_.add(x);
        y_.add(y);

        x_sum_ += x;
        y_sum_ += y;
        xy_sum_ += x * y;
        x2_sum_ += x * x;
    }

    // Estimates line: y = constant + slope * x
    // Returns false if the number of points added is less than the window size
    bool calculateLineEstimate(double* constant, double* slope) const
    {
        if (!x_.filled())
            return false;

        double x_mean = x_sum_ / window_size_;
        double y_mean = y_sum_ / window_size_;

        *slope = (window_size_ * xy_sum_ - x_sum_ * y_sum_) / (window_size_ * x2_sum_ - x_sum_ * x_sum_);

        *constant = y_mean - *slope * x_mean;

        return true;
    }

private:

    double x_sum_;
    double y_sum_;
    double xy_sum_;
    double x2_sum_;

    RingBuffer<double> x_;
    RingBuffer<double> y_;

    unsigned int window_size_;

};

} // end namespace dml

#endif
