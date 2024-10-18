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
 * File:            log.h
 * Description:     functions for logging purposes
 */

#ifndef ILPCR_LOG_H
#define ILPCR_LOG_H

#include <string>
#include <iostream>

namespace ilpcr
{
    int log_init(std::string filename, std::string header, std::ofstream* logfile);
    int log_write(std::string text, bool write_endl, std::ofstream* logfile);
    int log_write_process_info(std::ofstream* logfile, int argc, char** argv);
    void log_close(std::ofstream* logfile);
    void log_reopen(std::string filename, std::ofstream* logfile);

    void set_log_enabled(bool yes_or_no);
    bool get_log_enabled(void);
}

#endif // ILPCR_LOG_H