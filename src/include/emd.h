typedef struct
{
    float lpf_alpha_;
    float hpf_alpha_;
    float hpf_a_in_;
    float hpf_a_out_;
    float hpf_b_in_;
    float hpf_b_out_;
    float lpf_a_;
    float lpf_b_;
} EMD;

void EMD_init(EMD* emd, float lpf_alpha, float hpf_alpha);
float EMD_update(EMD* EMD, float a, float b);
