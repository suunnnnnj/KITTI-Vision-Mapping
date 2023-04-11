#include "utils.hpp"

std::vector<std::string> listdir(const std::string &directoryPath, const std::string &extension)
{
    std::vector<std::string> list;

    for(auto path : boost::filesystem::directory_iterator(directoryPath))
    {
        if(extension != "")
        {
            if(path.path().extension() == extension)
                list.push_back(path.path().string());
        }
        else
        {
            list.push_back(path.path().string());
        }
    }   

    sort(list.begin(), list.end());

    return list;
}

std::vector<std::string> tokenize(std::string context, const std::string &delim)
{
    std::vector<std::string> tokens;

    size_t pos = 0;

    while ((pos = context.find(delim)) != std::string::npos)
    {
        tokens.push_back(context.substr(0, pos));
        context.erase(0, pos + delim.length());
    }

    return tokens;
}