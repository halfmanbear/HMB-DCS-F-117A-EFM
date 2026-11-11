#pragma once

// Discrete version of a continuous transfer function of order 1 or 2:
//
//     H(s) = (num[0] s^n + ... + num[n]) / (den[0] s^n + ... + den[n])
//
// Coefficients run from the highest power of s down to the constant term, so a first-order
// lag p / (s + p) is num = {0, p}, den = {1, p} and a washout s / (s + w) is num = {1, 0},
// den = {1, w}. The filter is discretised once, at a fixed step, with the bilinear (Tustin)
// substitution s = (2 / T) (1 - z^-1) / (1 + z^-1).
class Tustin_filter
{
public:
    static constexpr int max_order = 2;

    void configure(const double* num, const double* den, int order, double step_s)
    {
        order_ = (order < 1) ? 1 : (order > max_order ? max_order : order);
        const double k = 2.0 / step_s;

        double z_num[max_order + 1] = {};
        double z_den[max_order + 1] = {};
        for (int term = 0; term <= order_; ++term)
        {
            // num[term] multiplies s^(n - term), which becomes
            // k^(n - term) (1 - z^-1)^(n - term) (1 + z^-1)^term over a common (1 + z^-1)^n.
            const int s_power = order_ - term;
            double poly[max_order + 1] = {1.0};
            int degree = 0;
            for (int i = 0; i < order_; ++i)
            {
                const double sign = (i < s_power) ? -1.0 : 1.0;
                for (int j = degree + 1; j > 0; --j)
                    poly[j] += sign * poly[j - 1];
                ++degree;
            }
            double scale = 1.0;
            for (int i = 0; i < s_power; ++i)
                scale *= k;
            for (int j = 0; j <= order_; ++j)
            {
                z_num[j] += num[term] * scale * poly[j];
                z_den[j] += den[term] * scale * poly[j];
            }
        }

        double num_sum = 0.0;
        double den_sum = 0.0;
        for (int j = 0; j <= order_; ++j)
        {
            b_[j] = z_num[j] / z_den[0];
            a_[j] = z_den[j] / z_den[0];
            num_sum += b_[j];
            den_sum += a_[j];
        }
        steady_gain_ = (den_sum != 0.0) ? num_sum / den_sum : 0.0;
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
            output += b_[i] * past_input_[i - 1] - a_[i] * past_output_[i - 1];

        for (int i = order_ - 1; i > 0; --i)
        {
            past_input_[i] = past_input_[i - 1];
            past_output_[i] = past_output_[i - 1];
        }
        past_input_[0] = input;
        past_output_[0] = output;
        return output;
    }

private:
    double settle(double input)
    {
        const double output = steady_gain_ * input;
        for (int i = 0; i < max_order; ++i)
        {
            past_input_[i] = input;
            past_output_[i] = output;
        }
        return output;
    }

    int order_ = 1;
    double b_[max_order + 1] = {};
    double a_[max_order + 1] = {};
    double past_input_[max_order] = {};  // newest first
    double past_output_[max_order] = {};
    double steady_gain_ = 0.0;
};
