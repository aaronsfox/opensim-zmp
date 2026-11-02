/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactSide.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ContactSide.h"

#include "ZeroMomentPointEstimator.h"
#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <memory>

using namespace OpenSim;
using SimTK::Vec3;

namespace {

// 2D helpers in the horizontal plane of Ground. A point's horizontal
// coordinates are (u, v) = (x, z).
double cross2D(const Vec3& o, const Vec3& a, const Vec3& b) {
    return (a[0] - o[0]) * (b[2] - o[2]) - (a[2] - o[2]) * (b[0] - o[0]);
}

double horizontalDistance(const Vec3& a, const Vec3& b) {
    return std::hypot(a[0] - b[0], a[2] - b[2]);
}

double horizontalDistanceToSegment(
        const Vec3& p, const Vec3& a, const Vec3& b) {
    const double du = b[0] - a[0];
    const double dv = b[2] - a[2];
    const double lengthSquared = du * du + dv * dv;
    if (lengthSquared == 0.0) return horizontalDistance(p, a);
    double t = ((p[0] - a[0]) * du + (p[2] - a[2]) * dv) / lengthSquared;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p[0] - (a[0] + t * du), p[2] - (a[2] + t * dv));
}

// Andrew's monotone chain. Returns the convex hull of the points in the (x, z)
// plane, counter-clockwise in (x, z), without collinear or repeated vertices.
std::vector<Vec3> convexHull(std::vector<Vec3> points) {
    std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
        return a[0] < b[0] || (a[0] == b[0] && a[2] < b[2]);
    });
    points.erase(std::unique(points.begin(), points.end(),
                         [](const Vec3& a, const Vec3& b) {
                             return a[0] == b[0] && a[2] == b[2];
                         }),
            points.end());
    if (points.size() < 3) return points;

    std::vector<Vec3> hull(2 * points.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        while (k >= 2 && cross2D(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            --k;
        }
        hull[k++] = points[i];
    }
    for (std::size_t i = points.size() - 1, lower = k + 1; i > 0; --i) {
        while (k >= lower &&
                cross2D(hull[k - 2], hull[k - 1], points[i - 1]) <= 0) {
            --k;
        }
        hull[k++] = points[i - 1];
    }
    // The last vertex repeats the first.
    hull.resize(k - 1);
    return hull;
}

} // namespace

//=============================================================================
// CONSTRUCTION
//=============================================================================
ContactSide::ContactSide() { constructProperties(); }

ContactSide::ContactSide(const PhysicalFrame& contactBody) {
    constructProperties();
    setName(contactBody.getName());
    connectSocket_contact_body(contactBody);
}

ContactSide::ContactSide(
        const std::string& name, const PhysicalFrame& contactBody) {
    constructProperties();
    setName(name);
    connectSocket_contact_body(contactBody);
}

void ContactSide::constructProperties() {
    constructProperty_contact_points();
    constructProperty_minimum_contact_duration(-1); // -1: unused.
}

void ContactSide::setMinimumContactDuration(double duration) {
    set_minimum_contact_duration(duration);
}

//=============================================================================
// CONTACT POINTS
//=============================================================================
int ContactSide::adoptContactPoint(ContactPoint* point) {
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "Expected a ContactPoint, but got a null pointer.");

    // Validate before taking ownership (as Component::addComponent does), so
    // the caller (including the Python and Java bindings) still owns `point`
    // if an exception is thrown.
    ContactPoint* owned = point;
    const int n = getNumContactPoints();

    if (owned->getName().empty()) {
        const std::string prefix =
                getName().empty() ? "point_" : getName() + "_point_";
        int suffix = n + 1;
        std::string name = prefix + std::to_string(suffix);
        auto nameIsUsed = [&](const std::string& candidate) {
            for (int i = 0; i < n; ++i) {
                if (get_contact_points(i).getName() == candidate) return true;
            }
            return false;
        };
        while (nameIsUsed(name)) { name = prefix + std::to_string(++suffix); }
        owned->setName(name);
    } else {
        for (int i = 0; i < n; ++i) {
            OPENSIM_THROW_IF_FRMOBJ(
                    get_contact_points(i).getName() == owned->getName(),
                    Exception,
                    std::format("A ContactPoint named '{}' already exists. "
                                "ContactPoint names must be unique within a "
                                "ContactSide.",
                            owned->getName()));
        }
    }

    return updProperty_contact_points().adoptAndAppendValue(owned);
}

int ContactSide::addContactPoint(const ContactPoint& point) {
    // Copying a component does not copy socket connections made by
    // reference, so reconnect the copy to the original's frame.
    std::unique_ptr<ContactPoint> copy(point.clone());
    if (point.getSocket<PhysicalFrame>("parent_frame").isConnected()) {
        copy->setParentFrame(point.getParentFrame());
    }
    const int index = adoptContactPoint(copy.get());
    copy.release();
    return index;
}

int ContactSide::getNumContactPoints() const {
    return getProperty_contact_points().size();
}

const ContactPoint& ContactSide::getContactPoint(int index) const {
    return get_contact_points(index);
}

const ContactPoint& ContactSide::getContactPoint(
        const std::string& name) const {
    for (int i = 0; i < getNumContactPoints(); ++i) {
        if (get_contact_points(i).getName() == name) {
            return get_contact_points(i);
        }
    }
    OPENSIM_THROW_FRMOBJ(
            Exception, std::format("No ContactPoint named '{}'.", name));
}

ContactPoint& ContactSide::updContactPoint(int index) {
    return upd_contact_points(index);
}

//=============================================================================
// CONTACT BODY AND GROUND
//=============================================================================
const PhysicalFrame& ContactSide::getContactBody() const {
    return getConnectee<PhysicalFrame>("contact_body");
}

void ContactSide::setContactBody(const PhysicalFrame& contactBody) {
    connectSocket_contact_body(contactBody);
}

double ContactSide::getGroundHeight() const {
    if (hasOwner()) {
        if (const auto* estimator =
                        dynamic_cast<const ZeroMomentPointEstimator*>(
                                &getOwner())) {
            return estimator->get_ground_height();
        }
    }
    return 0.0;
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void ContactSide::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    OPENSIM_THROW_IF_FRMOBJ(getNumContactPoints() == 0, InvalidPropertyValue,
            getProperty_contact_points().getName(),
            "Expected at least one ContactPoint, but none were provided.");

    const double duration = get_minimum_contact_duration();
    OPENSIM_THROW_IF_FRMOBJ(duration != -1 && duration <= 0,
            InvalidPropertyValue,
            getProperty_minimum_contact_duration().getName(),
            std::format("Expected minimum_contact_duration to be -1 (unused) "
                        "or positive, but got {}.",
                    duration));
}

//=============================================================================
// CONTACT AND SUPPORT POLYGON
//=============================================================================
bool ContactSide::isInContact(const SimTK::State& state) const {
    for (int i = 0; i < getNumContactPoints(); ++i) {
        if (get_contact_points(i).isInContact(state)) return true;
    }
    return false;
}

int ContactSide::calcNumActiveContactPoints(const SimTK::State& state) const {
    int count = 0;
    for (int i = 0; i < getNumContactPoints(); ++i) {
        if (get_contact_points(i).isInContact(state)) ++count;
    }
    return count;
}

std::vector<Vec3> ContactSide::calcActiveContactPointLocationsInGround(
        const SimTK::State& state) const {
    const double groundHeight = getGroundHeight();
    std::vector<Vec3> locations;
    for (int i = 0; i < getNumContactPoints(); ++i) {
        const ContactPoint& point = get_contact_points(i);
        if (point.isInContact(state)) {
            Vec3 location = point.getLocationInGround(state);
            location[1] = groundHeight;
            locations.push_back(location);
        }
    }
    return locations;
}

Vec3 ContactSide::calcActiveContactPointCentroidInGround(
        const SimTK::State& state) const {
    const std::vector<Vec3> locations =
            calcActiveContactPointLocationsInGround(state);
    if (locations.empty()) return Vec3(SimTK::NaN);
    Vec3 sum(0);
    for (const auto& location : locations) sum += location;
    return sum / static_cast<double>(locations.size());
}

std::vector<Vec3> ContactSide::calcSupportPolygonInGround(
        const SimTK::State& state) const {
    return convexHull(calcActiveContactPointLocationsInGround(state));
}

double ContactSide::calcSignedDistanceToSupportPolygon(
        const SimTK::State& state, const Vec3& point) const {
    const std::vector<Vec3> polygon = calcSupportPolygonInGround(state);
    const std::size_t n = polygon.size();
    if (n == 0) return SimTK::NaN;
    if (n == 1) return horizontalDistance(point, polygon[0]);
    if (n == 2)
        return horizontalDistanceToSegment(point, polygon[0], polygon[1]);

    double distance = std::numeric_limits<double>::infinity();
    bool inside = true;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3& a = polygon[i];
        const Vec3& b = polygon[(i + 1) % n];
        distance = std::min(distance, horizontalDistanceToSegment(point, a, b));
        // Counter-clockwise polygon: inside is to the left of every edge.
        if (cross2D(a, b, point) < 0) inside = false;
    }
    return inside ? -distance : distance;
}

bool ContactSide::isPointInSupportPolygon(
        const SimTK::State& state, const Vec3& point, double tolerance) const {
    const double distance = calcSignedDistanceToSupportPolygon(state, point);
    return !SimTK::isNaN(distance) && distance <= tolerance;
}
