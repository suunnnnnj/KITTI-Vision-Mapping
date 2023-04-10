#include "utils.hpp"

std::vector<std::string> listidr(const std::string &directoryPath, const std::string &extension)
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

// template <typename N>
// std::string zeroPadding(const N &number, const int numberOfZeros)
// {
//     std::string count_str = std::to_string(number);
//     std::string padded_str = std::string(numberOfZeros - count_str.length(), '0') + count_str;

//     return padded_str;
// }