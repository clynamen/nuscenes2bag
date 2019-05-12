#pragma once

#include <string>

#define SHOW_FIRST_MEMBER(memberName) "{" #memberName << o.memberName
#define SHOW_MEMBER(memberName) ", "#memberName << o.memberName
#define SHOW_LAST_MEMBER(memberName) ", "#memberName"}" << o.memberName

template <typename T> std::string to_debug_string(const T& t);
