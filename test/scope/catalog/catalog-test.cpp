#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "common/decimal.hpp"

#include "scope/catalog/catalog.hpp"

namespace scope {

namespace {
/// Path to the small fixture catalog, relative to the repository root (the
/// working directory used by both the coverage target and the documented
/// single-test invocation).
const char *kFixturePath = "test/fixtures/bright-star-catalog-test.tsv";
}  // namespace

// Check that the fixture parses into the expected number of stars.
TEST(LoadBscTest, ParsesAllStars) {
    Catalog catalog = LoadBsc(kFixturePath);
    EXPECT_EQ(catalog.size(), 6u);
}

// Check that RA/Dec are converted to the expected unit vectors (LOST's
// SphericalToSpatial convention: ra=0,dec=0 -> (1,0,0)).
TEST(LoadBscTest, ConvertsKnownDirections) {
    Catalog catalog = LoadBsc(kFixturePath);

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

// Check that every parsed spatial vector is a unit vector.
TEST(LoadBscTest, SpatialVectorsAreUnitLength) {
    Catalog catalog = LoadBsc(kFixturePath);
    for (const CatalogStar &star : catalog) {
        EXPECT_NEAR(star.spatial.norm(), DECIMAL(1.0), DECIMAL(1e-6));
    }
}

// Check magnitude scaling (true magnitude * 100) and stable names, including the
// negative-magnitude branch where the fractional part is also negative.
TEST(LoadBscTest, ParsesMagnitudesAndNames) {
    Catalog catalog = LoadBsc(kFixturePath);

    EXPECT_EQ(catalog[0].name, 1);
    EXPECT_EQ(catalog[0].magnitude, 600);
    EXPECT_EQ(catalog[1].magnitude, 550);
    EXPECT_EQ(catalog[2].magnitude, 425);
    EXPECT_EQ(catalog[3].magnitude, 300);

    // -1.46 -> -1 * 100 + (-46) = -146.
    EXPECT_EQ(catalog[5].name, 6);
    EXPECT_EQ(catalog[5].magnitude, -146);
}

// Check that a missing catalog file throws rather than returning silently.
TEST(LoadBscTest, ThrowsOnMissingFile) {
    EXPECT_THROW(LoadBsc("test/fixtures/does-not-exist.tsv"), std::runtime_error);
}

// Check that a file that exists but contains no parseable star lines throws.
TEST(LoadBscTest, ThrowsWhenNoStarsParse) {
    EXPECT_THROW(LoadBsc("test/fixtures/bright-star-catalog-empty.tsv"), std::runtime_error);
}

}  // namespace scope
