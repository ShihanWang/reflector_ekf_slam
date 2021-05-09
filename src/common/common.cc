#include "common/common.h"
#include <fstream>
#include <sstream> 

std::vector<std::string> SplitString(const std::string &input,
                                     const char delimiter)
{
    std::istringstream stream(input);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(stream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

bool IsFileExist(const std::string &file)
{
    std::ifstream f(file.c_str());
    return f.good();
}