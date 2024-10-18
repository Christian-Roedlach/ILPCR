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
 * File:            ilpcrCupoch.h
 * Description:     functions utilizing the Cupoch library (FGR algorithm) 
 */

#ifndef ILPCR_CUPOCH_H
#define ILPCR_CUPOCH_H

#if CUPOCH_ENABLED

#include "types.h"

#define USE_OMP_FOR_CUPOCH_CONVERSION 1

namespace ilpcr {
    using EigenVector33f = Eigen::Matrix<float, 33, 1>;
    using vector33f_host = thrust::host_vector<EigenVector33f>;
    using vector33f_host_ptr = std::shared_ptr<vector33f_host>;

    int loadDestinationCloud_cupoch(fpfh_data_t *fpfh_data);
    int matchPointClouds_cupoch(capture_data_t *capture_data, fpfh_data_t *fpfh_data);

} // namespace ilprc

#endif // CUPOCH_ENABLED

#endif // ILPCR_CUPOCH_H