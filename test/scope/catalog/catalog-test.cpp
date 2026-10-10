#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "common/decimal.hpp"

#include "scope/catalog/catalog.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

// LOST's SphericalToSpatial convention: ra=0, dec=0 -> (1, 0, 0).
TEST(LoadBscTest, ConvertsKnownDirections) {
    Catalog catalog = LoadBsc(kFixtureCatalog);

    // ra=0, dec=0 -> (1, 0, 0)
    EXPECT_NEAR(catalog[0].spatial.x(), DECIMAL(1.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[0].spatial.y(), DECIMAL(0.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[0].spatial.z(), DECIMAL(0.0), DECIMAL(1e-6));

    // ra=90, dec=0 -> (0, 1, 0)
    EXPECT_NEAR(catalog[1].spatial.x(), DECIMAL(0.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[1].spatial.y(), DECIMAL(1.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[1].spatial.z(), DECIMAL(0.0), DECIMAL(1e-6));

    // ra=0, dec=90 -> (0, 0, 1)
    EXPECT_NEAR(catalog[2].spatial.x(), DECIMAL(0.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[2].spatial.y(), DECIMAL(0.0), DECIMAL(1e-6));
    EXPECT_NEAR(catalog[2].spatial.z(), DECIMAL(1.0), DECIMAL(1e-6));

    // ra=0, dec=-90 -> (0, 0, -1)
    EXPECT_NEAR(catalog[4].spatial.z(), DECIMAL(-1.0), DECIMAL(1e-6));
}

// Magnitudes are stored times 100, negative ones included.
TEST(LoadBscTest, ParsesMagnitudesAndNames) {
    Catalog catalog = LoadBsc(kFixtureCatalog);

    EXPECT_EQ(catalog[0].name, 1);
    EXPECT_EQ(catalog[0].magnitude, 600);
    EXPECT_EQ(catalog[1].magnitude, 550);
    EXPECT_EQ(catalog[2].magnitude, 425);
    EXPECT_EQ(catalog[3].magnitude, 300);

    // -1.46 -> -146.
    EXPECT_EQ(catalog[5].name, 6);
    EXPECT_EQ(catalog[5].magnitude, -146);

    // -0.74 -> -74. The integer part is "-0", so a per-field parse would store +74.
    EXPECT_EQ(catalog[6].name, 7);
    EXPECT_EQ(catalog[6].magnitude, -74);
}

TEST(LoadBscTest, ThrowsOnMissingFile) {
    EXPECT_THROW(LoadBsc("test/fixtures/does-not-exist.tsv"), std::runtime_error);
}

TEST(LoadBscTest, ThrowsWhenNoStarsParse) {
    EXPECT_THROW(LoadBsc("test/fixtures/bright-star-catalog-empty.tsv"), std::runtime_error);
}

}  // namespace scope
