#pragma once

// Discrete version of a continuous transfer function of order 1 or 2:
//
//     H(s) = (num[0] s^n + ... + num[n]) / (den[0] s^n + ... + den[n])
//
// Coefficients run from the highest power of s down to the constant term, so a first-order
// lag p / (s + p) is num = {0, p}, den = {1, p} and a washout s / (s + w) is num = {1, 0},
// den = {1, w}. The filter is discretised once, at a fixed step, with the bilinear (Tustin)
// substitution s = (2 / T) (1 - z^-1) / (1 + z^-1).
class TustinFilter
{
public:
    static constexpr int kMaxOrder = 2;

    void configure(const double* num, const double* den, int order, double stepS)
    {
        order_ = (order < 1) ? 1 : (order > kMaxOrder ? kMaxOrder : order);
        const double k = 2.0 / stepS;

        double zNum[kMaxOrder + 1] = {};
        double zDen[kMaxOrder + 1] = {};
        for (int term = 0; term <= order_; ++term)
        {
            // num[term] multiplies s^(n - term), which becomes
            // k^(n - term) (1 - z^-1)^(n - term) (1 + z^-1)^term over a common (1 + z^-1)^n.
            const int sPower = order_ - term;
            double poly[kMaxOrder + 1] = {1.0};
            int degree = 0;
            for (int i = 0; i < order_; ++i)
            {
                const double sign = (i < sPower) ? -1.0 : 1.0;
                for (int j = degree + 1; j > 0; --j)
                    poly[j] += sign * poly[j - 1];
                ++degree;
            }
            double scale = 1.0;
            for (int i = 0; i < sPower; ++i)
                scale *= k;
            for (int j = 0; j <= order_; ++j)
            {
                zNum[j] += num[term] * scale * poly[j];
                zDen[j] += den[term] * scale * poly[j];
            }
        }

        double numSum = 0.0;
        double denSum = 0.0;
        for (int j = 0; j <= order_; ++j)
        {
            b_[j] = zNum[j] / zDen[0];
            a_[j] = zDen[j] / zDen[0];
            numSum += b_[j];
            denSum += a_[j];
        }
        steadyGain_ = (denSum != 0.0) ? numSum / denSum : 0.0;
        settle(0.0);
    }

    // Returns the filtered value. With reset set, the filter starts settled on this input
    // and returns the steady-state output for it.
    double step(double input, bool reset)
    {
        if (reset)
            return settle(input);

        double output = b_[0] * input;
        for (int i = 1; i <= order_; ++i)
            output += b_[i] * pastInput_[i - 1] - a_[i] * pastOutput_[i - 1];

        for (int i = order_ - 1; i > 0; --i)
        {
            pastInput_[i] = pastInput_[i - 1];
            pastOutput_[i] = pastOutput_[i - 1];
        }
        pastInput_[0] = input;
        pastOutput_[0] = output;
        return output;
    }

private:
    double settle(double input)
    {
        const double output = steadyGain_ * input;
        for (int i = 0; i < kMaxOrder; ++i)
        {
            pastInput_[i] = input;
            pastOutput_[i] = output;
        }
        return output;
    }

    int order_ = 1;
    double b_[kMaxOrder + 1] = {};
    double a_[kMaxOrder + 1] = {};
    double pastInput_[kMaxOrder] = {};  // newest first
    double pastOutput_[kMaxOrder] = {};
    double steadyGain_ = 0.0;
};
