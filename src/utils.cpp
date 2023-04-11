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
 
    boost::char_separator<char> sep(delim.c_str());
    boost::tokenizer<boost::char_separator<char>> tokenizer(context, sep);
 
    for (const std::string &s: tokenizer) 
        tokens.push_back(s);

    return tokens;
}