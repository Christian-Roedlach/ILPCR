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
 * File:            debug.h
 * Description:     DEBUG defines
 */

#ifndef ILPCR_DEBUG_H
#define ILPCR_DEBUG_H

/* Macro */
#ifndef ILPCR_DEBUG
    #error "ILPCR_DEBUG has to be defined BEFORE including debug.h! (either 1 or 0)"
#endif // ILPCR_DEBUG

#if (ILPCR_DEBUG)
    #define ILPCR_DEBUG_EXECUTION(x) x;
#else // ILPCR_DEBUG
    #define ILPCR_DEBUG_EXECUTION(x)
#endif // ILPCR_DEBUG

#endif // ILPCR_DEBUG_H