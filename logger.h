#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "system.h"

#ifdef ISTRO_LOGGER_LOG4CXX

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

#define LOG_DEFINE(logger_variable, logger_name) LoggerPtr logger_variable(Logger::getLogger(logger_name))

// Load XML configuration file using DOMConfigurator
#define LOG_CONFIG_LOAD(logger_config) DOMConfigurator::configure(logger_config)
#define LOG_DESTROY() {}

#define LOG_THREAD_NAME(logger_thread_name) MDC::put("threadName", logger_thread_name)

#define LOG_TRACE(logger_variable, logger_msg) LOG4CXX_TRACE(logger_variable, " " << logger_msg)
#define LOG_DEBUG(logger_variable, logger_msg) LOG4CXX_DEBUG(logger_variable, " " << logger_msg)
#define LOG_INFO(logger_variable, logger_msg)  LOG4CXX_INFO(logger_variable, " " << logger_msg)
#define LOG_WARN(logger_variable, logger_msg)  LOG4CXX_WARN(logger_variable, " " << logger_msg)
#define LOG_ERROR(logger_variable, logger_msg) LOG4CXX_ERROR(logger_variable, " " << logger_msg)
#define LOG_FATAL(logger_variable, logger_msg) LOG4CXX_FATAL(logger_variable, " " << logger_msg)

#define LOGM_TRACE(logger_variable, logger_method, logger_msg) LOG4CXX_TRACE(logger_variable, ":" << logger_method << "(): " << logger_msg)
#define LOGM_DEBUG(logger_variable, logger_method, logger_msg) LOG4CXX_DEBUG(logger_variable, ":" << logger_method << "(): " << logger_msg)
#define LOGM_INFO(logger_variable, logger_method, logger_msg)  LOG4CXX_INFO(logger_variable, ":" << logger_method << "(): " << logger_msg)
#define LOGM_WARN(logger_variable, logger_method, logger_msg)  LOG4CXX_WARN(logger_variable, ":" << logger_method << "(): " << logger_msg)
#define LOGM_ERROR(logger_variable, logger_method, logger_msg) LOG4CXX_ERROR(logger_variable, ":" << logger_method << "(): " << logger_msg)
#define LOGM_FATAL(logger_variable, logger_method, logger_msg) LOG4CXX_FATAL(logger_variable, ":" << logger_method << "(): " << logger_msg)

#else

#include <iostream>

void lmi(void);
void lmd(void);
void lml(void);
void lmu(void);

#define LOG_DEFINE(logger_variable, logger_name) static const char logger_variable[] = logger_name;

#define LOG_CONFIG_LOAD(logger_config) { lmi(); }
#define LOG_DESTROY() { lmd(); }

#define LOG_THREAD_NAME(logger_thread_name) {}

#define LOG_TRACE(logger_variable, logger_msg)  { lml(); std::cout << "TRACE " << logger_variable << ": " << logger_msg << std::endl; lmu(); }
#define LOG_DEBUG(logger_variable, logger_msg)  { lml(); std::cout << "DEBUG " << logger_variable << ": " << logger_msg << std::endl; lmu(); }
#define LOG_INFO(logger_variable, logger_msg)   { lml(); std::cout << "INFO  " << logger_variable << ": " << logger_msg << std::endl; lmu(); }
#define LOG_WARN(logger_variable, logger_msg)   { lml(); std::cout << "WARN  " << logger_variable << ": " << logger_msg << std::endl; lmu(); }
#define LOG_ERROR(logger_variable, logger_msg)  { lml(); std::cout << "ERROR " << logger_variable << ": " << logger_msg << std::endl; lmu(); }
#define LOG_FATAL(logger_variable, logger_msg)  { lml(); std::cout << "FATAL " << logger_variable << ": " << logger_msg << std::endl; lmu(); }

#define LOGM_TRACE(logger_variable, logger_method, logger_msg)  { lml(); std::cout << "TRACE " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }
#define LOGM_DEBUG(logger_variable, logger_method, logger_msg)  { lml(); std::cout << "DEBUG " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }
#define LOGM_INFO(logger_variable, logger_method, logger_msg)   { lml(); std::cout << "INFO  " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }
#define LOGM_WARN(logger_variable, logger_method, logger_msg)   { lml(); std::cout << "WARN  " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }
#define LOGM_ERROR(logger_variable, logger_method, logger_msg)  { lml(); std::cout << "ERROR " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }
#define LOGM_FATAL(logger_variable, logger_method, logger_msg)  { lml(); std::cout << "FATAL " << logger_variable << "::" << logger_method << "(): " << logger_msg << std::endl; lmu(); }

#endif

#include <iomanip>

// zero fill padding for integers
#define iozf(x, p)  std::setfill('0') << std::setw(p) << ((int)x) << std::setw(0)
// fixed float
#define ioff(x, p)  std::fixed << std::setprecision(p) << ((double)x) << std::resetiosflags(std::ios::fixed)

#endif
