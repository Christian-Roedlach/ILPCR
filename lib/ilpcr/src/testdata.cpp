/*
 * ILPCR - Indoor Localization by Point Cloud Registration
 * Copyright (C) 2024 Christian Roedlach
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 * File:            testdata.cpp
 * Description:     functions for generating test data
 */

#include "testdata.h"

#include <pcl/common/impl/angles.hpp>

namespace ilpcr
{
    Eigen::Affine3f makeTransformationMatrix(uint32_t number) {
        const float diameter = 1.0;
        const float angle_per_iteration = pcl::deg2rad (10.);
        static float z_change_per_iteration = 0.02;
        const float z_max = 1.4;
        const float z_min = -1.0;
        const float z_start = -0.3;
        const float angle_rotate_z_per_iteration = 30.;
        static float z_value = z_start;

        z_value += z_change_per_iteration;

        if (z_value > z_max || z_value < z_min)
        {
            z_change_per_iteration = -z_change_per_iteration;
            z_value += 2 * z_change_per_iteration;
        }

        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        Eigen::Vector3f translation(
                diameter * cos(angle_per_iteration * number),
                diameter * sin(angle_per_iteration * number),
                z_value);
        t = Eigen::Translation<float, 3>(translation);
        double rotate_z = pcl::deg2rad (angle_rotate_z_per_iteration * number);
        t.rotate(Eigen::AngleAxisf (rotate_z, Eigen::Vector3f::UnitZ()));
        //t.translate(translation);

        return t;
    }
}