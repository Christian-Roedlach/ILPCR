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
 * File:            presets.h
 * Description:     ILPCR presets system
 */

#ifndef ILPCR_RS2_FILTER_PRESETS_H
#define ILPCR_RS2_FILTER_PRESETS_H

#include <string>

namespace ilpcr 
{
    typedef enum {
        rs2_filter_preset_undefined,
        rs2_filter_preset_first,

        rs2_filter_preset_1 = rs2_filter_preset_first,
        rs2_filter_preset_2,
        rs2_filter_preset_1a,
        rs2_filter_preset_2a,
        rs2_filter_preset_3,
        rs2_filter_preset_3a,
        rs2_filter_preset_4,
        rs2_filter_preset_4a,
        rs2_filter_preset_5,
        rs2_filter_preset_5a,
        rs2_filter_preset_6,
        rs2_filter_preset_6a,

        rs2_filter_preset_count
    } rs2_filter_preset_names_t;

    typedef struct {
        rs2_filter_preset_names_t preset;
        std::string preset_name;
        std::string rs2_config_file; /* prefixed with camera type - e.g. "D455_" */
        float limit_min;
        float limit_max;
        bool  dec_fil_enable;       /* decimation filter */
        float dec_fil_magnitude; 
        bool  spa_fil_enable;       /* spatial filter*/
        float spa_fil_magnitude; 
        float spa_fil_alpha;
        float spa_fil_delta;
        float spa_fil_holes_fill;
        bool  tem_fil_enable;       /* temporal filter */
        float tem_fil_alpha;
        float tem_fil_delta;
        int   tem_fil_pers_ctl;     /* persistence control */
    } rs2_filter_settings_t;

    extern const rs2_filter_settings_t rs2_filter_presets[rs2_filter_preset_count];

    typedef enum {
        pcr_preset_undefined = 0,
        pcr_preset_first,

        pcr_preset_narf_1 = pcr_preset_first,
        pcr_preset_fpfh_1,
        pcr_preset_fpfh_2,
        pcr_preset_fpfh_3,
        pcr_preset_fpfh_4,
        pcr_preset_fpfh_5cu, 
        pcr_preset_fpfh_5_fpfh_fgr_gpu,    

        pcr_names_count,

        pcr_names_custom
    } pcr_names_t;

    typedef enum {
        ilpcr_proc_type_undefined,

        ilpcr_proc_type_cpu,
        ilpcr_proc_type_fgr_gpu,
        ilpcr_proc_type_fpfh_fgr_gpu,

        ilpcr_proc_type_count
    } proc_t;
    
    typedef struct {
        pcr_names_t preset;
        std::string preset_name;
        int     max_iterations;
        int     nr_of_samples;
        int     corr_rand;
        float   sim_threshold;
        double  max_corr_distance;
        float   inlier_fraction;
        double  raster_size;
        double  feature_search_radius;
        double  nest_search_radius;
        proc_t  processing_unit;
        bool    use_normal_surface_search;
    } pcr_settings_t;

    /*
    const std::string pcr_settings_config_file_strings[] = 
    {
        "preset_name",
        "max_iterations",
        "nr_of_samples",
        "corr_rand",
        "sim_threshold",
        "max_corr_distance",
        "inlier_fraction",
        "raster_size",
        "feature_search_radius",
        "nest_search_radius"
    };
    */

    extern const pcr_settings_t pcr_presets[pcr_names_count];
}


#endif // ILPCR_RS2_FILTER_PRESETS_H