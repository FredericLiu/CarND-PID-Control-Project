#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
	p.push_back(Kp_init);
	p.push_back(Ki_init);
	p.push_back(Kd_init);
	
	dp.push_back(1.0);
	dp.push_back(1.0);
	dp.push_back(0.005);

	
	sum_cte_ = 0;
	steer_value_ = 0;
	prev_cte_ = 0;
	min_cycle = 5;
	cycle_count = 0;
	p_index = 0;
	twiddle_step = 0; 
	is_twiddle_initialized = false;
}

void PID::UpdateError(double cte) {
	double dP_sum = dp[0] + dp[1] + dp[2];
    if(cycle_count >= min_cycle){
        std::cout << "[UpdateError] [dP-sum=" << dP_sum << ", P=(" << p[0] << "," << p[1] << "," << p[2] << ")]" << "best_err: "<<best_err<<std::endl;
		 
		if (!is_twiddle_initialized){
			best_err = TotalError();
			is_twiddle_initialized = true;
		}else if(dP_sum >= 0.001){		
            Twiddle();
        }
		cycle_count = 0;
		ctes.clear();
    }
	
	++cycle_count;
	
	steer_value_ = -p[0]*cte -p[1]*(cte-prev_cte_) -p[2]*sum_cte_;
	std::cout<<"cte: "<<cte<<"  prev_cte: "<<prev_cte_<<"  sum_cte_"<<sum_cte_<<std::endl;
	sum_cte_ += cte;
	prev_cte_ = cte;
	ctes.push_back(cte);
}

double PID::TotalError() {
	double sum_error = 0.0;
    for(auto cte: ctes){
        sum_error += cte * cte;
    }
    return sum_error/ ctes.size();
}

void PID::Twiddle(){
	double err;
	switch(twiddle_step){
		case 0:
		{
			p[p_index] += dp[p_index];
			twiddle_step = 1;
			
			return;
		}
		
		case 1:
			err = TotalError();
			if (err < best_err){
				best_err = err;
				dp[p_index] *=1.1;
				twiddle_step = 0;
				p_index = (p_index+1)%dp.size();
				
				return;
			} else{
				p[p_index] -= 2*dp[p_index];
				twiddle_step = 2;
				return;
			}
		
		case 2:
			err = TotalError();
			if (err < best_err){
				best_err = err;
				dp[p_index] *= 1.1;
				twiddle_step = 0;
				p_index = (p_index+1)%dp.size();

				return;
			} else {
				p[p_index] += dp[p_index];
				dp[p_index] *= 0.9;
				twiddle_step = 0;
				p_index = (p_index+1)%dp.size();
				return;
			}		
	}

}

