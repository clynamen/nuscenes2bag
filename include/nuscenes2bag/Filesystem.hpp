#pragma once

#if CMAKE_CXX_STANDARD >= 17
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif