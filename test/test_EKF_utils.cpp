/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file test_EKF_utils.cpp
 *
 * @brief Unit tests for the miscellaneous EKF utilities
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "EKF/ekf.h"
#include "EKF/utils.hpp"

TEST(eclPowfTest, compareToStandardImplementation)
{
	std::vector<int> exponents = {-3,-2,-1,-0,0,1,2,3};
	std::vector<float> bases = {-INFINITY, -11.1f,-0.5f, -0.f, 0.f, 0.5f, 11.1f, INFINITY};

	for (auto const exponent : exponents) {
		for (auto const basis : bases) {
			EXPECT_EQ(ecl::powf(basis, exponent),
				  std::pow(basis, static_cast<float>(exponent)));
		}
	}
}

TEST(SparseVectorTest, initialization) {
	SparseVectorf<4, 6> a;
	EXPECT_EQ(a.size(), 2);
	EXPECT_EQ(a.index(0), 4);
	EXPECT_EQ(a.index(1), 6);
	EXPECT_DEATH(a(0) = 1.f, ".*");
	EXPECT_DEATH(a(7) = 1.f, ".*");
	a(4) = 1.f;
	a(6) = 2.f;
}

TEST(SparseVectorTest, initializationWithVector) {
	Vector3f vec(1.f, 2.f, 3.f);
	SparseVectorf<4, 6, 22> a(vec);
	EXPECT_EQ(a.size(), 3);
	EXPECT_EQ(a.index(0), 4);
	EXPECT_EQ(a.index(1), 6);
	EXPECT_EQ(a.index(2), 22);
	EXPECT_FLOAT_EQ(a(4), vec(0));
	EXPECT_FLOAT_EQ(a(6), vec(1));
	EXPECT_FLOAT_EQ(a(22), vec(2));
	a.setZero();
	EXPECT_FLOAT_EQ(a(4), 0.f);
	EXPECT_FLOAT_EQ(a(6), 0.f);
	EXPECT_FLOAT_EQ(a(22), 0.f);
}
