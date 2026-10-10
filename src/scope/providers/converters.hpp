#ifndef SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_
#define SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_

#include <string>

#include "providers/converters.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Loads images from a list of file paths.
 *
 * @param str Comma- or space-separated list of image file paths. Empty
 *            entries (a trailing or doubled delimiter) are skipped.
 *
 * @return Images loaded from the listed paths, in order.
 *
 * @throws std::runtime_error if any listed path cannot be loaded as an image.
 */
inline Images strtoimages(const std::string &str) {
    char delimiter = str.find(" ") != std::string::npos ? ' ' : ',';

    size_t start = 0;

    Images images;

    while (start < str.size()) {
        size_t end = str.find(delimiter, start);
        if (end == std::string::npos) {
            end = str.size();
        }
        if (end > start) {
            images.push_back(found::strtoimage(str.substr(start, end - start)));
        }
        start = end + 1;
    }

    return images;
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_CONVERTERS_HPP_
