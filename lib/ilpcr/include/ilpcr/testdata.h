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
 * File:            testdata.h
 * Description:     functions for generating test data
 */

#ifndef ILPCR_TESTDATA_H
#define ILPCR_TESTDATA_H

#include "types.h"
#include "viewer.h"
#include <Eigen/Eigen>

namespace ilpcr
{
    Eigen::Affine3f makeTransformationMatrix(uint32_t number);
}

#endif // ILPCR_TESTDATA_H