
#include <algorithm>
#include <cmath>
#include "gaussian.h"
#include "config.h"

using namespace std;
//using namespace config;


template <typename T>
std::shared_ptr<T> create_auto_buf(size_t len, bool init_zero = false) {
	std::shared_ptr<T> ret(new T[len], [](T *ptr){delete []ptr;});
	if (init_zero)
		memset(ret.get(), 0, sizeof(T) * len);
	return ret;
}

GaussCache::GaussCache(float sigma) 
{
	// TODO decide window size ?
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	//cout << kw << " " << sigma << endl;
	if (kw % 2 == 0) kw ++;
	kernel_buf = create_auto_buf<float>(kw);
	const int center = kw / 2;
	kernel = kernel_buf.get() + center;

	kernel[0] = 1;

	float exp_coeff = -1.0 / (sigma * sigma * 2),
				 wsum = 1;
	for (int i = 1; i <= center; i ++)
		wsum += (kernel[i] = exp(i * i * exp_coeff)) * 2;

	float fac = 1.0 / wsum;
	kernel[0] = fac;
	for (int i = 1; i <= center; i ++)
		kernel[-i] = (kernel[i] *= fac);
}


