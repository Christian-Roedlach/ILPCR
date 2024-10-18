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
 * File:            log.cpp
 * Description:     functions for logging purposes
 */

#include "log.h"
#include <fstream>
#include <unistd.h>
#include <chrono>

namespace ilpcr
{
    using namespace std;

    static bool log_enabled = false;

    int log_init(std::string filename, std::string header, std::ofstream* logfile)
    {
        int retval = EXIT_FAILURE;
        if(get_log_enabled())
        {
            if (nullptr != logfile)
            {
                ifstream logfile_read;
                bool create_new_logfile = false;

                // check if logfile already exists
                logfile_read.open (filename);
                if (!logfile_read.is_open()) {
                    create_new_logfile = true;
                }
                logfile_read.close();

                // Open logfile
                logfile->open (filename, ios::out | ios::app);

                if (logfile->is_open())
                {
                    retval = EXIT_SUCCESS;
                    if (create_new_logfile)
                    {
                        *logfile << header << endl;
                    } 
                } else {
                    retval = EXIT_FAILURE;
                    cerr << "ERROR opening logfile: " << filename << endl;
                }
            } else {
                retval = EXIT_FAILURE;
                cerr << "ERROR nullptr received!" << endl;
            }
        } else {
            retval = EXIT_SUCCESS;
        }
             
        return retval;
    }

    int log_write(std::string text, bool write_endl, std::ofstream* logfile)
    {   
        int retval = EXIT_FAILURE;

        if(get_log_enabled())
        {
            if(logfile != nullptr && logfile->is_open())
            {
                retval = EXIT_SUCCESS;
                if (write_endl)
                    *logfile << text << endl;
                else
                    *logfile << text << ",";
            } else {
                retval = EXIT_FAILURE;
                cerr << "ERROR writing to logfile!" << endl;
            }
        }
        
        return retval;
    }

    // writes following entries: hostname,executable,timestamp,cmd_args
    int log_write_process_info(std::ofstream* logfile, int argc, char** argv)
    {
        int retval = EXIT_FAILURE;

        if(get_log_enabled())
        {
            if (logfile != nullptr && logfile->is_open())
            {
                char timeStamp[100];
                char hostname[100];
                timespec ts;

                timespec_get(&ts, TIME_UTC);
                std::strftime(timeStamp, sizeof timeStamp, "%D %T", std::gmtime(&ts.tv_sec));

                if (0 != gethostname(hostname, sizeof(hostname)))
                    cerr << "Reading hostname FAILED!\n";
                else
                    *logfile << hostname << ",";  /* hostname */

                if (argc > 0)
                    *logfile << argv[0] << ",";  /* executable */
                
                *logfile << timeStamp << '.' << ts.tv_nsec << " UTC,";  /* timestamp */

                 /* cmd line args */
                for(int i = 1; i < argc; ++i)
                    *logfile << argv[i] << ' ';
                *logfile << ",";
                
                retval = EXIT_SUCCESS;
            } else {
                retval = EXIT_FAILURE;
                cerr << "ERROR writing to logfile!" << endl;
            }
        }
        
        return retval;
    }

    void log_close(std::ofstream* logfile)
    {
        if(get_log_enabled() && logfile != nullptr)
        {
            logfile->close();
        }
    }

    void log_reopen(std::string filename, std::ofstream* logfile)
    {
        if(get_log_enabled() && logfile != nullptr)
        {
            logfile->open (filename, ios::out | ios::app);
        }
    }

    void set_log_enabled(bool yes_or_no)
    {
        log_enabled = yes_or_no;
    }

    bool get_log_enabled(void)
    {
        return log_enabled;
    }
}