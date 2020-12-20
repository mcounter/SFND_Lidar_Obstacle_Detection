#ifndef TOOLS_H_
#define TOOLS_H_

#include <string>
#include <algorithm>

std::string strUpr(const std::string& s)
{
    std::string us = s;
    std::transform(us.begin(), us.end(), us.begin(), ::toupper);

    return us;
}

#endif /* TOOLS_H_ */