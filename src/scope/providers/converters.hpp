#ifndef SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_
#define SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_

#include <string>

#include "providers/converters.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Loads images from a list of file paths.
 *
 * @param str Comma- or space-separated list of image file paths.
 *
 * @return Images loaded from the listed paths.
 *
 * @pre Each entry in str must be a valid path to an image file.
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
