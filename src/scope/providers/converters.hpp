#ifndef SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_
#define SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_

#include <string>

#include "providers/converters.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Converts a string to a vector of images
 * 
 * @param str The string to convert
 * 
 * @return The vector of location records that the string represents
 * 
 * @pre str must contain a list of valid file paths to image files.
 * Each file path needs to be seperated by commas or spaces.
 */
inline Images strtoimages(const std::string &str) {
    char delimiter = str.find(" ") != std::string::npos ? ' ' : ',';

    size_t start = 0;
    size_t end = str.find(delimiter);

    Images images;

    while (end != std::string::npos) {
        images.push_back(found::strtoimage(str.substr(start, end - start)));
        start = end + 1;
        end = str.find(delimiter, start);
    }

    return images;
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_
