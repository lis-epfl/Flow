#include "emd.h"

void EMD_init(EMD* emd, float lpf_alpha, float hpf_alpha)
{
    emd->lpf_alpha_ = lpf_alpha;
    emd->hpf_alpha_ = hpf_alpha;
    emd->hpf_a_in_ = 0.0f;
    emd->hpf_a_out_ = 0.0f;
    emd->hpf_b_in_ = 0.0f;
    emd->hpf_b_out_ = 0.0f;
    emd->lpf_a_ = 0.0f;
    emd->lpf_b_ = 0.0f;
}


float EMD_update(EMD* emd, float a, float b)
{
    // high pass on input A
    emd->hpf_a_out_ = emd->hpf_a_out_ * emd->hpf_alpha_ +
                     (a - emd->hpf_a_in_) * emd->hpf_alpha_;
    emd->hpf_a_in_  = a;

    // high pass on input B
    emd->hpf_b_out_ = emd->hpf_b_out_ * emd->hpf_alpha_ +
                     (b - emd->hpf_b_in_) * emd->hpf_alpha_;
    emd->hpf_b_in_  = b;

    // low pass on A
    emd->lpf_a_ = emd->lpf_alpha_ * emd->hpf_a_out_ +
                 (1 - emd->lpf_alpha_) * emd->lpf_a_;

    // low pass on B
    emd->lpf_b_ = emd->lpf_alpha_ * emd->hpf_b_out_ +
                 (1 - emd->lpf_alpha_) * emd->lpf_b_;


    float R = (emd->lpf_b_ * emd->hpf_a_out_) - (emd->lpf_a_ * emd->hpf_b_out_);

    return R;
}
