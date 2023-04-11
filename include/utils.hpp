#ifndef UTILS_HPP
#define UTILS_HPP

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <string>
#include <vector>

std::vector<std::string> listdir(const std::string &directoryPath, const std::string &extension = "");
std::vector<std::string> tokenize(std::string context, const std::string &delim = " ");

template <typename N>
std::string zeroPadding(const N &number, const int numberOfZeros)
{
    std::string count_str = std::to_string(number);
    std::string padded_str = std::string(numberOfZeros - count_str.length(), '0') + count_str;

    return padded_str;
}

#endif