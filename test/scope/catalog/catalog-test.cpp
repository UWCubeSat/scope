#include <gtest/gtest.h>

#include <cstddef>

#include <stdexcept>
#include <string>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

/// Angle between two lines of sight, in arcseconds.
decimal SeparationArcSec(const found::Vec3 &a, const found::Vec3 &b) {
    return found::RadToArcSec(DECIMAL_ATAN2(a.cross(b).norm(), a.dot(b)));
}

/// Right ascension of a line of sight, in arcseconds.
decimal RaArcSec(const found::Vec3 &v) { return found::RadToArcSec(DECIMAL_ATAN2(v.y(), v.x())); }

/// Declination of a line of sight, in arcseconds.
decimal DecArcSec(const found::Vec3 &v) { return found::RadToArcSec(DECIMAL_ASIN(v.z())); }

const decimal kArcSecTol = DECIMAL(1e-3);

}  // namespace

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

// Proper motion in declination moves a star on the equator straight north.
TEST(PropagateProperMotionTest, MovesAlongDeclination) {
    const found::Vec3 moved =
        PropagateProperMotion(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0), DECIMAL(100.0));

    EXPECT_NEAR(moved.norm(), DECIMAL(1.0), DECIMAL(1e-12));
    EXPECT_NEAR(DecArcSec(moved), DECIMAL(100.0), kArcSecTol);
    EXPECT_NEAR(RaArcSec(moved), DECIMAL(0.0), kArcSecTol);
}

// pmRA is given times cos(dec), so at dec 60 deg a star moving 1 arcsec/yr on
// the sky changes right ascension by 2 arcsec/yr.
TEST(PropagateProperMotionTest, RightAscensionMotionIsScaledByCosDec) {
    const decimal dec = found::DegToRad(DECIMAL(60.0));
    const found::Vec3 start = PropagateProperMotion(DECIMAL(0.0), dec, DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    const found::Vec3 moved = PropagateProperMotion(DECIMAL(0.0), dec, DECIMAL(1.0), DECIMAL(0.0), DECIMAL(100.0));

    EXPECT_NEAR(SeparationArcSec(start, moved), DECIMAL(100.0), kArcSecTol);
    EXPECT_NEAR(RaArcSec(moved), DECIMAL(200.0), DECIMAL(0.01));
}

// At the pole, right ascension still fixes which way is south.
TEST(PropagateProperMotionTest, WorksAtThePole) {
    const decimal pole = found::DegToRad(DECIMAL(90.0));
    const found::Vec3 moved = PropagateProperMotion(DECIMAL(0.0), pole, DECIMAL(0.0), DECIMAL(-1.0), DECIMAL(100.0));

    EXPECT_NEAR(DecArcSec(moved), DECIMAL(90.0 * 3600.0 - 100.0), kArcSecTol);
    EXPECT_NEAR(moved.y(), DECIMAL(0.0), DECIMAL(1e-12));
    EXPECT_GT(moved.x(), DECIMAL(0.0));
}

// Groombridge 1830 (HR 4550), the fastest star in the catalog: 7.06 arcsec/yr,
// so by late 2026 it is over three arcminutes from its J2000 position.
TEST(PropagateProperMotionTest, MovesGroombridge1830) {
    const decimal ra = found::DegToRad(DECIMAL(178.245));
    const decimal dec = found::DegToRad(DECIMAL(37.718611));
    const decimal years = DECIMAL(26.8);
    const found::Vec3 start = PropagateProperMotion(ra, dec, DECIMAL(4.003), DECIMAL(-5.813), DECIMAL(0.0));
    const found::Vec3 moved = PropagateProperMotion(ra, dec, DECIMAL(4.003), DECIMAL(-5.813), years);

    const decimal expected = DECIMAL_SQRT(DECIMAL(4.003) * DECIMAL(4.003) + DECIMAL(5.813) * DECIMAL(5.813)) * years;
    EXPECT_NEAR(SeparationArcSec(start, moved), expected, DECIMAL(0.01));
    // The star moves on a great circle, so its eastward motion also raises its
    // declination slightly (by theta^2 tan(dec) / 2, about 0.02 arcsec here).
    EXPECT_NEAR(DecArcSec(moved) - DecArcSec(start), DECIMAL(-5.813) * years, DECIMAL(0.05));
}

// A catalog with proper-motion columns loads exactly as one without them at
// the catalog epoch, and moves its stars at a later one. Rows without proper
// motion (missing or blank) stay put, and comments and blank lines are skipped.
TEST(LoadBscTest, AppliesProperMotion) {
    TempFile file("catalog-pm.tsv",
                  "# ra|dec|hr|multiple|vmag|pmra|pmde\n"
                  "000.000000|+00.000000|1| | 6.00|+0.000|+1.000\n"
                  "\n"
                  "000.000000|+60.000000|2| | 5.50|+1.000|+0.000\n"
                  "090.000000|+00.000000|3| | 4.00|      |      \n"
                  "180.000000|+00.000000|4| | 3.00\n");

    const Catalog atCatalogEpoch = LoadBsc(file.Path());
    const Catalog later = LoadBsc(file.Path(), DECIMAL(2100.0));

    ASSERT_EQ(atCatalogEpoch.size(), 4u);
    ASSERT_EQ(later.size(), 4u);
    EXPECT_NEAR(atCatalogEpoch[0].spatial.x(), DECIMAL(1.0), DECIMAL(1e-12));

    EXPECT_NEAR(DecArcSec(later[0].spatial), DECIMAL(100.0), kArcSecTol);
    EXPECT_NEAR(SeparationArcSec(atCatalogEpoch[1].spatial, later[1].spatial), DECIMAL(100.0), kArcSecTol);
    EXPECT_EQ(later[2].spatial, atCatalogEpoch[2].spatial);
    EXPECT_EQ(later[3].spatial, atCatalogEpoch[3].spatial);

    EXPECT_EQ(later[1].name, 2);
    EXPECT_EQ(later[1].magnitude, 550);
    EXPECT_EQ(later[3].magnitude, 300);
}

// A catalog downloaded before the proper-motion columns were added gives the
// same stars at any epoch.
TEST(LoadBscTest, OldCatalogIgnoresEpoch) {
    const Catalog listed = LoadBsc(kFixtureCatalog);
    const Catalog later = LoadBsc(kFixtureCatalog, DECIMAL(2026.8));

    ASSERT_EQ(later.size(), listed.size());
    for (std::size_t i = 0; i < listed.size(); ++i) {
        EXPECT_EQ(later[i].spatial, listed[i].spatial);
    }
}

// A row that does not parse fails the load rather than ending it early, so a
// damaged catalog cannot silently lose every star after the damage.
TEST(LoadBscTest, ThrowsOnMalformedRow) {
    TempFile file("catalog-malformed.tsv",
                  "000.000000|+00.000000|1| |6.00\n"
                  "not a catalog row\n"
                  "090.000000|+00.000000|2| |5.50\n");

    try {
        LoadBsc(file.Path());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error &e) {
        EXPECT_NE(std::string(e.what()).find("line 2"), std::string::npos) << e.what();
    }
}

TEST(LoadBscTest, ThrowsWhenOnlyComments) {
    TempFile file("catalog-comments.tsv", "# nothing here\n\n   \n");
    EXPECT_THROW(LoadBsc(file.Path()), std::runtime_error);
}

}  // namespace scope
