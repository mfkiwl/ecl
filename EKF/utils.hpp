#include <matrix/math.hpp>

#pragma once

// converts Tait-Bryan 312 sequence of rotations from frame 1 to frame 2
// to the corresponding rotation matrix that rotates from frame 2 to frame 1
// rot312(0) - First rotation is a RH rotation about the Z axis (rad)
// rot312(1) - Second rotation is a RH rotation about the X axis (rad)
// rot312(2) - Third rotation is a RH rotation about the Y axis (rad)
// See http://www.atacolorado.com/eulersequences.doc
matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312);

// Use Kahan summation algorithm to get the sum of "sum_previous" and "input".
// This function relies on the caller to be responsible for keeping a copy of
// "accumulator" and passing this value at the next iteration.
// Ref: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
float kahanSummation(float sum_previous, float input, float &accumulator);

// calculate the inverse rotation matrix from a quaternion rotation
// this produces the inverse rotation to that produced by the math library quaternion to Dcmf operator
matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat);

namespace ecl{
	inline float powf(float x, int exp)
	{
		float ret;
		if (exp > 0) {
			ret = x;
			for (int count = 1; count < exp; count++) {
				ret *= x;
			}
			return ret;
		} else if (exp < 0) {
			return 1.0f / ecl::powf(x, -exp);
		}
		return 1.0f;
	}
}

// Vector that only store nonzero elements,
// which indices are specified as parameter pack
template<typename Type, size_t... Idxs>
class SparseVector {
private:
	static constexpr size_t N = sizeof...(Idxs);
	static constexpr size_t _indices[N] {Idxs...};
	matrix::Vector<Type, N> _data {};

	static constexpr size_t findInverseIndex(size_t index, size_t range = N) {
		assert(range != 0);
		const size_t last_elem = range - 1;
		return (_indices[last_elem] == index) ? last_elem : findInverseIndex(index, range - 1);
	}

public:
	constexpr size_t size() const { return N; }
	constexpr size_t index(size_t i) const { return SparseVector::_indices[i]; }

	SparseVector() {}

	SparseVector(const matrix::Vector<Type, N>& data) {
		_data = data;
	}

	inline Type operator()(size_t i) const {
		return _data(findInverseIndex(i));
	}

	inline Type& operator()(size_t i) {
		return _data(findInverseIndex(i));
	}

	void setZero() {
		_data.setZero();
	}
};

template<typename Type, size_t... Idxs>
constexpr size_t SparseVector<Type, Idxs...>::_indices[SparseVector<Type, Idxs...>::N];

template<size_t ... Idxs>
using SparseVectorf = SparseVector<float, Idxs...>;
