/*
 Copyright (C) 2010-2017 Kristian Duske

 This file is part of TrenchBroom.

 TrenchBroom is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 TrenchBroom is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with TrenchBroom. If not, see <http://www.gnu.org/licenses/>.
 */

#include "BrushFace.h"

#include "Ensure.h"
#include "Exceptions.h"
#include "FloatType.h"
#include "Polyhedron.h"
#include "Assets/Texture.h"
#include "Model/BrushError.h"
#include "Model/MapFormat.h"
#include "Model/ParallelTexCoordSystem.h"
#include "Model/ParaxialTexCoordSystem.h"
#include "Model/TagMatcher.h"
#include "Model/TagVisitor.h"
#include "Model/TexCoordSystem.h"

#include <kdl/overload.h>
#include <kdl/result.h>
#include <kdl/string_utils.h>

#include <vecmath/bbox.h>
#include <vecmath/intersection.h>
#include <vecmath/mat.h>
#include <vecmath/plane.h>
#include <vecmath/polygon.h>
#include <vecmath/scalar.h>
#include <vecmath/util.h>
#include <vecmath/vec.h>
#include <vecmath/vec_io.h>

#include <sstream>
#include <string>

namespace TrenchBroom {
    namespace Model {
        const BrushVertex* BrushFace::TransformHalfEdgeToVertex::operator()(const BrushHalfEdge* halfEdge) const {
            return halfEdge->origin();
        }

        const BrushEdge* BrushFace::TransformHalfEdgeToEdge::operator()(const BrushHalfEdge* halfEdge) const {
            return halfEdge->edge();
        }

        BrushFace::BrushFace(const BrushFace& other) :
        Taggable(other),
        m_points(other.m_points),
        m_boundary(other.m_boundary),
        m_attributes(other.m_attributes),
        m_textureReference(other.m_textureReference),
        m_texCoordSystem(other.m_texCoordSystem ? other.m_texCoordSystem->clone() : nullptr),
        m_geometry(nullptr),
        m_lineNumber(other.m_lineNumber),
        m_lineCount(other.m_lineCount),
        m_selected(other.m_selected),
        m_markedToRenderFace(false) {}
        
        BrushFace::BrushFace(BrushFace&& other) noexcept :
        Taggable(other),
        m_points(std::move(other.m_points)),
        m_boundary(std::move(other.m_boundary)),
        m_attributes(std::move(other.m_attributes)),
        m_textureReference(std::move(other.m_textureReference)),
        m_texCoordSystem(std::move(other.m_texCoordSystem)),
        m_geometry(other.m_geometry),
        m_lineNumber(other.m_lineNumber),
        m_lineCount(other.m_lineCount),
        m_selected(other.m_selected),
        m_markedToRenderFace(false) {}
        
        BrushFace& BrushFace::operator=(BrushFace other) noexcept {
            using std::swap;
            swap(*this, other);
            return *this;
        }

        void swap(BrushFace& lhs, BrushFace& rhs) noexcept {
            using std::swap;
            swap(static_cast<Taggable&>(lhs), static_cast<Taggable&>(rhs));
            swap(lhs.m_points, rhs.m_points);
            swap(lhs.m_boundary, rhs.m_boundary);
            swap(lhs.m_attributes, rhs.m_attributes);
            swap(lhs.m_textureReference, rhs.m_textureReference);
            swap(lhs.m_texCoordSystem, rhs.m_texCoordSystem);
            swap(lhs.m_geometry, rhs.m_geometry);
            swap(lhs.m_lineNumber, rhs.m_lineNumber);
            swap(lhs.m_lineCount, rhs.m_lineCount);
            swap(lhs.m_selected, rhs.m_selected);
            swap(lhs.m_markedToRenderFace, rhs.m_markedToRenderFace);
        }

        BrushFace::~BrushFace() = default;

        kdl::result<BrushFace, BrushError> BrushFace::create(const vm::vec3& point0, const vm::vec3& point1, const vm::vec3& point2, const BrushFaceAttributes& attributes, const MapFormat mapFormat) {
            return Model::isParallelTexCoordSystem(mapFormat)
                   ? BrushFace::create(point0, point1, point2, attributes, std::make_unique<ParallelTexCoordSystem>(point0, point1, point2, attributes))
                   : BrushFace::create(point0, point1, point2, attributes, std::make_unique<ParaxialTexCoordSystem>(point0, point1, point2, attributes));
        }

        kdl::result<BrushFace, BrushError> BrushFace::createFromStandard(const vm::vec3& point0, const vm::vec3& point1, const vm::vec3& point2, const BrushFaceAttributes& inputAttribs, const MapFormat mapFormat) {
            assert(mapFormat != MapFormat::Unknown);

            std::unique_ptr<TexCoordSystem> texCoordSystem;
            BrushFaceAttributes attribs("");

            if (Model::isParallelTexCoordSystem(mapFormat)) {
                // Convert paraxial to parallel
                std::tie(texCoordSystem, attribs) = ParallelTexCoordSystem::fromParaxial(point0, point1, point2, inputAttribs);
            } else {
                // Pass through paraxial
                texCoordSystem = std::make_unique<ParaxialTexCoordSystem>(point0, point1, point2, inputAttribs);
                attribs = inputAttribs;
            }

            return BrushFace::create(point0, point1, point2, attribs, std::move(texCoordSystem));
        }

        kdl::result<BrushFace, BrushError> BrushFace::createFromValve(const vm::vec3& point1, const vm::vec3& point2, const vm::vec3& point3, const BrushFaceAttributes& inputAttribs, const vm::vec3& texAxisX, const vm::vec3& texAxisY, MapFormat mapFormat) {
            assert(mapFormat != MapFormat::Unknown);

            std::unique_ptr<TexCoordSystem> texCoordSystem;
            BrushFaceAttributes attribs("");

            if (Model::isParallelTexCoordSystem(mapFormat)) {
                // Pass through parallel
                texCoordSystem = std::make_unique<ParallelTexCoordSystem>(texAxisX, texAxisY);
                attribs = inputAttribs;
            } else {
                // Convert parallel to paraxial
                std::tie(texCoordSystem, attribs) = ParaxialTexCoordSystem::fromParallel(point1, point2, point3, inputAttribs, texAxisX, texAxisY);
            }

            return BrushFace::create(point1, point2, point3, attribs, std::move(texCoordSystem));
        }

        kdl::result<BrushFace, BrushError> BrushFace::create(const vm::vec3& point0, const vm::vec3& point1, const vm::vec3& point2, const BrushFaceAttributes& attributes, std::unique_ptr<TexCoordSystem> texCoordSystem) {
            Points points = {{ vm::correct(point0), vm::correct(point1), vm::correct(point2) }};
            const auto [result, plane] = vm::from_points(points[0], points[1], points[2]);
            if (result) {
                return BrushFace(points, plane, attributes, std::move(texCoordSystem));
            } else {
                return BrushError::InvalidFace;
            }
        }

        BrushFace::BrushFace(const BrushFace::Points& points, const vm::plane3& boundary, const BrushFaceAttributes& attributes, std::unique_ptr<TexCoordSystem> texCoordSystem) :
        m_points(points),
        m_boundary(boundary),
        m_attributes(attributes),
        m_texCoordSystem(std::move(texCoordSystem)),
        m_geometry(nullptr),
        m_lineNumber(0),
        m_lineCount(0),
        m_selected(false),
        m_markedToRenderFace(false) {
            ensure(m_texCoordSystem != nullptr, "texCoordSystem is null");
        }

        bool operator==(const BrushFace& lhs, const BrushFace& rhs) {
            return lhs.m_points == rhs.m_points &&
            lhs.m_boundary == rhs.m_boundary &&
            lhs.m_attributes == rhs.m_attributes &&
            *lhs.m_texCoordSystem == *rhs.m_texCoordSystem &&
            lhs.m_lineNumber == rhs.m_lineNumber &&
            lhs.m_lineCount == rhs.m_lineCount &&
            lhs.m_selected == rhs.m_selected;
                    
        }
        
        bool operator!=(const BrushFace& lhs, const BrushFace& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const BrushFace& face) {
            str << "{ " << face.m_points[0] << ", " << face.m_points[1] << ", " << face.m_points[2] << " }";
            return str;
        }

        void BrushFace::sortFaces(std::vector<BrushFace>& faces) {
            // Originally, the idea to sort faces came from TxQBSP, but the sorting used there was not entirely clear to me.
            // But it is still desirable to have a deterministic order in which the faces are added to the brush, so I chose
            // to just sort the faces by their normals.

            std::sort(std::begin(faces), std::end(faces), [](const auto& lhs, const auto& rhs) {
                const auto& lhsBoundary = lhs.boundary();
                const auto& rhsBoundary = rhs.boundary();

                const auto cmp = vm::compare(lhsBoundary.normal, rhsBoundary.normal);
                if (cmp < 0) {
                    return true;
                } else if (cmp > 0) {
                    return false;
                } else {
                    // normal vectors are identical -- this should never happen
                    return lhsBoundary.distance < rhsBoundary.distance;
                }
            });
        }

        std::unique_ptr<TexCoordSystemSnapshot> BrushFace::takeTexCoordSystemSnapshot() const {
            return m_texCoordSystem->takeSnapshot();
        }

        void BrushFace::restoreTexCoordSystemSnapshot(const TexCoordSystemSnapshot& coordSystemSnapshot) {
            coordSystemSnapshot.restore(*m_texCoordSystem);
        }

        void BrushFace::copyTexCoordSystemFromFace(const TexCoordSystemSnapshot& coordSystemSnapshot, const BrushFaceAttributes& attributes, const vm::plane3& sourceFacePlane, const WrapStyle wrapStyle) {
            // Get a line, and a reference point, that are on both the source face's plane and our plane
            const auto seam = vm::intersect_plane_plane(sourceFacePlane, m_boundary);
            const auto refPoint = vm::project_point(seam, center());

            coordSystemSnapshot.restore(*m_texCoordSystem);

            // Get the texcoords at the refPoint using the source face's attributes and tex coord system
            const auto desriedCoords = m_texCoordSystem->getTexCoords(refPoint, attributes, vm::vec2f::one());

            m_texCoordSystem->updateNormal(sourceFacePlane.normal, m_boundary.normal, m_attributes, wrapStyle);

            // Adjust the offset on this face so that the texture coordinates at the refPoint stay the same
            if (!vm::is_zero(seam.direction, vm::C::almost_zero())) {
                const auto currentCoords = m_texCoordSystem->getTexCoords(refPoint, m_attributes, vm::vec2f::one());
                const auto offsetChange = desriedCoords - currentCoords;
                m_attributes.setOffset(correct(modOffset(m_attributes.offset() + offsetChange), 4));
            }
        }

        const BrushFace::Points& BrushFace::points() const {
            return m_points;
        }

        const vm::plane3& BrushFace::boundary() const {
            return m_boundary;
        }

        const vm::vec3& BrushFace::normal() const {
            return boundary().normal;
        }

        vm::vec3 BrushFace::center() const {
            ensure(m_geometry != nullptr, "geometry is null");
            const BrushHalfEdgeList& boundary = m_geometry->boundary();
            return vm::average(std::begin(boundary), std::end(boundary), BrushGeometry::GetVertexPosition());
        }

        vm::vec3 BrushFace::boundsCenter() const {
            ensure(m_geometry != nullptr, "geometry is null");

            const auto toPlane = vm::plane_projection_matrix(m_boundary.distance, m_boundary.normal);
            const auto [invertible, fromPlane] = vm::invert(toPlane);
            assert(invertible); unused(invertible);

            const auto* first = m_geometry->boundary().front();
            const auto* current = first;

            vm::bbox3 bounds;
            bounds.min = bounds.max = toPlane * current->origin()->position();

            current = current->next();
            while (current != first) {
                bounds = merge(bounds, toPlane * current->origin()->position());
                current = current->next();
            }
            return fromPlane * bounds.center();
        }

        FloatType BrushFace::projectedArea(const vm::axis::type axis) const {
            FloatType c1 = 0.0;
            FloatType c2 = 0.0;
            for (const BrushHalfEdge* halfEdge : m_geometry->boundary()) {
                const auto origin = vm::swizzle(halfEdge->origin()->position(), axis);
                const auto destination = vm::swizzle(halfEdge->destination()->position(), axis);
                c1 += origin.x() * destination.y();
                c2 += origin.y() * destination.x();
            }
            return vm::abs((c1 - c2) / 2.0);
        }

        FloatType BrushFace::area() const {
            auto result = static_cast<FloatType>(0);

            const auto* firstEdge = m_geometry->boundary().front();
            const auto* currentEdge = firstEdge->next();
            const auto* vertex0 = firstEdge->origin();
            do {
                const auto* vertex1 = currentEdge->origin();
                const auto* vertex2 = currentEdge->destination();
                const auto side0 = vertex1->position() - vertex0->position();
                const auto side1 = vertex2->position() - vertex0->position();
                result += vm::length(vm::cross(side0, side1));

                currentEdge = currentEdge->next();
            } while (currentEdge->next() != firstEdge);

            return result / 2.0;
        }

        bool BrushFace::coplanarWith(const vm::plane3d& plane) const {
            // Test if the face's center lies on the reference plane within an epsilon.
            if (!vm::is_zero(plane.point_distance(center()), vm::constants<FloatType>::almost_zero() * 10.0)) {
                return false;
            }

            // Test if the normals are colinear by checking their enclosed angle.
            if (1.0 - vm::dot(boundary().normal, plane.normal) >= vm::constants<FloatType>::colinear_epsilon()) {
                return false;
            }
            return true;
        }

        const BrushFaceAttributes& BrushFace::attributes() const {
            return m_attributes;
        }

        void BrushFace::setAttributes(const BrushFaceAttributes& attributes) {
            const float oldRotation = m_attributes.rotation();
            m_attributes = attributes;
            m_texCoordSystem->setRotation(m_boundary.normal, oldRotation, m_attributes.rotation());
        }

        bool BrushFace::setAttributes(const BrushFace& other) {
            auto result = false;
            result |= m_attributes.setTextureName(other.attributes().textureName());
            result |= m_attributes.setXOffset(other.attributes().xOffset());
            result |= m_attributes.setYOffset(other.attributes().yOffset());
            result |= m_attributes.setRotation(other.attributes().rotation());
            result |= m_attributes.setXScale(other.attributes().xScale());
            result |= m_attributes.setYScale(other.attributes().yScale());
            result |= m_attributes.setSurfaceContents(other.attributes().surfaceContents());
            result |= m_attributes.setSurfaceFlags(other.attributes().surfaceFlags());
            result |= m_attributes.setSurfaceValue(other.attributes().surfaceValue());
            return result;
        }

        int BrushFace::resolvedSurfaceContents() const {
            if (m_attributes.surfaceContents()) {
                return *m_attributes.surfaceContents();
            }
            if (texture()) {
                if (const auto* q2data = std::get_if<Assets::Q2Data>(&texture()->gameData())) {
                    return q2data->contents;
                }
            }
            return 0;
        }

        int BrushFace::resolvedSurfaceFlags() const {
            if (m_attributes.surfaceFlags()) {
                return *m_attributes.surfaceFlags();
            }
            if (texture()) {
                if (const auto* q2data = std::get_if<Assets::Q2Data>(&texture()->gameData())) {
                    return q2data->flags;
                }
            }
            return 0;
        }

        float BrushFace::resolvedSurfaceValue() const {
            if (m_attributes.surfaceValue()) {
                return *m_attributes.surfaceValue();
            }
            if (texture()) {
                if (const auto* q2data = std::get_if<Assets::Q2Data>(&texture()->gameData())) {
                    return static_cast<float>(q2data->value);
                }
            }
            return 0.0f;
        }

        Color BrushFace::resolvedColor() const {
            return m_attributes.color().value_or(Color{});
        }

        void BrushFace::resetTexCoordSystemCache() {
            if (m_texCoordSystem != nullptr) {
                m_texCoordSystem->resetCache(m_points[0], m_points[1], m_points[2], m_attributes);
            }
        }

        const TexCoordSystem& BrushFace::texCoordSystem() const {
            return *m_texCoordSystem;
        }

        const Assets::Texture* BrushFace::texture() const {
            return m_textureReference.get();
        }

        vm::vec2f BrushFace::textureSize() const {
            if (texture() == nullptr) {
                return vm::vec2f::one();
            }
            const float w = texture()->width()  == 0 ? 1.0f : static_cast<float>(texture()->width());
            const float h = texture()->height() == 0 ? 1.0f : static_cast<float>(texture()->height());
            return vm::vec2f(w, h);
        }

        vm::vec2f BrushFace::modOffset(const vm::vec2f& offset) const {
            return m_attributes.modOffset(offset, textureSize());
        }

        bool BrushFace::setTexture(Assets::Texture* texture) {
            if (texture == this->texture()) {
                return false;
            }

            m_textureReference = Assets::AssetReference(texture);
            return true;
        }

        vm::vec3 BrushFace::textureXAxis() const {
            return m_texCoordSystem->xAxis();
        }

        vm::vec3 BrushFace::textureYAxis() const {
            return m_texCoordSystem->yAxis();
        }

        void BrushFace::resetTextureAxes() {
            m_texCoordSystem->resetTextureAxes(m_boundary.normal);
        }

        void BrushFace::resetTextureAxesToParaxial() {
            m_texCoordSystem->resetTextureAxesToParaxial(m_boundary.normal, 0.0f);
        }

        void BrushFace::convertToParaxial() {
            auto [newTexCoordSystem, newAttributes] = m_texCoordSystem->toParaxial(m_points[0], m_points[1], m_points[2], m_attributes);

            m_attributes = newAttributes;
            m_texCoordSystem = std::move(newTexCoordSystem);
        }

        void BrushFace::convertToParallel() {
            auto [newTexCoordSystem, newAttributes] = m_texCoordSystem->toParallel(m_points[0], m_points[1], m_points[2], m_attributes);

            m_attributes = newAttributes;
            m_texCoordSystem = std::move(newTexCoordSystem);
        }


        void BrushFace::moveTexture(const vm::vec3& up, const vm::vec3& right, const vm::vec2f& offset) {
            m_texCoordSystem->moveTexture(m_boundary.normal, up, right, offset, m_attributes);
        }

        void BrushFace::rotateTexture(const float angle) {
            const float oldRotation = m_attributes.rotation();
            m_texCoordSystem->rotateTexture(m_boundary.normal, angle, m_attributes);
            m_texCoordSystem->setRotation(m_boundary.normal, oldRotation, m_attributes.rotation());
        }

        void BrushFace::shearTexture(const vm::vec2f& factors) {
            m_texCoordSystem->shearTexture(m_boundary.normal, factors);
        }

        void BrushFace::flipTexture(const vm::vec3& /* cameraUp */, const vm::vec3& cameraRight, const vm::direction cameraRelativeFlipDirection) {
            const vm::mat4x4 texToWorld = m_texCoordSystem->fromMatrix(vm::vec2f::zero(), vm::vec2f::one());

            const vm::vec3 texUAxisInWorld = vm::normalize((texToWorld * vm::vec4d(1, 0, 0, 0)).xyz());
            const vm::vec3 texVAxisInWorld = vm::normalize((texToWorld * vm::vec4d(0, 1, 0, 0)).xyz());

            // Get the cos(angle) between cameraRight and the texUAxisInWorld _line_ (so, take the smaller of the angles
            // among -texUAxisInWorld and texUAxisInWorld). Note that larger cos(angle) means smaller angle.
            const FloatType UAxisCosAngle = vm::max(vm::dot(texUAxisInWorld, cameraRight),
                                                    vm::dot(-texUAxisInWorld, cameraRight));

            const FloatType VAxisCosAngle = vm::max(vm::dot(texVAxisInWorld, cameraRight),
                                                    vm::dot(-texVAxisInWorld, cameraRight));

            // If this is true, it means the texture's V axis is closer to the camera's right vector than the
            // texture's U axis is (i.e. we're looking at the texture sideways), so we should map
            // "camera relative horizontal" to "texture space Y".
            const bool cameraRightCloserToTexV = (VAxisCosAngle > UAxisCosAngle);

            bool flipTextureX = (cameraRelativeFlipDirection == vm::direction::left
                                 || cameraRelativeFlipDirection == vm::direction::right);

            if (cameraRightCloserToTexV) {
                flipTextureX = !flipTextureX;
            }

            if (flipTextureX) {
                m_attributes.setXScale(-1.0f * m_attributes.xScale());
            } else {
                m_attributes.setYScale(-1.0f * m_attributes.yScale());
            }
        }

        kdl::result<void, BrushError> BrushFace::transform(const vm::mat4x4& transform, const bool lockTexture) {
            using std::swap;

            const vm::vec3 invariant = m_geometry != nullptr ? center() : m_boundary.anchor();
            const vm::plane3 oldBoundary = m_boundary;

            m_boundary = m_boundary.transform(transform);
            for (size_t i = 0; i < 3; ++i) {
                m_points[i] = transform * m_points[i];
            }

            if (dot(cross(m_points[2] - m_points[0], m_points[1] - m_points[0]), m_boundary.normal) < 0.0) {
                swap(m_points[1], m_points[2]);
            }

            return setPoints(m_points[0], m_points[1], m_points[2])
                .and_then([&]() {
                    m_texCoordSystem->transform(oldBoundary, m_boundary, transform, m_attributes, textureSize(), lockTexture, invariant);
                });
        }

        void BrushFace::invert() {
            using std::swap;

            m_boundary = m_boundary.flip();
            swap(m_points[1], m_points[2]);
        }

        kdl::result<void, BrushError> BrushFace::updatePointsFromVertices() {
            ensure(m_geometry != nullptr, "geometry is null");

            const auto* first = m_geometry->boundary().front();
            const auto oldPlane = m_boundary;
            return setPoints(
                first->next()->origin()->position(),
                first->origin()->position(),
                first->previous()->origin()->position()
            ).and_then([&]() {
                // Get a line, and a reference point, that are on both the old plane
                // (before moving the face) and after moving the face.
                const auto seam = vm::intersect_plane_plane(oldPlane, m_boundary);
                if (!vm::is_zero(seam.direction, vm::C::almost_zero())) {
                    const auto refPoint = project_point(seam, center());

                    // Get the texcoords at the refPoint using the old face's attribs and tex coord system
                    const auto desriedCoords = m_texCoordSystem->getTexCoords(refPoint, m_attributes, vm::vec2f::one());

                    m_texCoordSystem->updateNormal(oldPlane.normal, m_boundary.normal, m_attributes, WrapStyle::Projection);

                    // Adjust the offset on this face so that the texture coordinates at the refPoint stay the same
                    const auto currentCoords = m_texCoordSystem->getTexCoords(refPoint, m_attributes, vm::vec2f::one());
                    const auto offsetChange = desriedCoords - currentCoords;
                    m_attributes.setOffset(correct(modOffset(m_attributes.offset() + offsetChange), 4));
                }
            });
        }

        vm::mat4x4 BrushFace::projectToBoundaryMatrix() const {
            const auto texZAxis = m_texCoordSystem->fromMatrix(vm::vec2f::zero(), vm::vec2f::one()) * vm::vec3::pos_z();
            const auto worldToPlaneMatrix = vm::plane_projection_matrix(m_boundary.distance, m_boundary.normal, texZAxis);
            const auto [invertible, planeToWorldMatrix] = vm::invert(worldToPlaneMatrix); assert(invertible); unused(invertible);
            return planeToWorldMatrix * vm::mat4x4::zero_out<2>() * worldToPlaneMatrix;
        }

        vm::mat4x4 BrushFace::toTexCoordSystemMatrix(const vm::vec2f& offset, const vm::vec2f& scale, const bool project) const {
            if (project) {
                return vm::mat4x4::zero_out<2>() * m_texCoordSystem->toMatrix(offset, scale);
            } else {
                return m_texCoordSystem->toMatrix(offset, scale);
            }
        }

        vm::mat4x4 BrushFace::fromTexCoordSystemMatrix(const vm::vec2f& offset, const vm::vec2f& scale, const bool project) const {
            if (project) {
                return projectToBoundaryMatrix() * m_texCoordSystem->fromMatrix(offset, scale);
            } else {
                return m_texCoordSystem->fromMatrix(offset, scale);
            }
        }

        float BrushFace::measureTextureAngle(const vm::vec2f& center, const vm::vec2f& point) const {
            return m_texCoordSystem->measureAngle(m_attributes.rotation(), center, point);
        }

        size_t BrushFace::vertexCount() const {
            assert(m_geometry != nullptr);
            return m_geometry->boundary().size();
        }

        BrushFace::EdgeList BrushFace::edges() const {
            ensure(m_geometry != nullptr, "geometry is null");
            return EdgeList(m_geometry->boundary(), TransformHalfEdgeToEdge());
        }

        BrushFace::VertexList BrushFace::vertices() const {
            ensure(m_geometry != nullptr, "geometry is null");
            return VertexList(m_geometry->boundary(), TransformHalfEdgeToVertex());
        }

        std::vector<vm::vec3> BrushFace::vertexPositions() const {
            ensure(m_geometry != nullptr, "geometry is null");
            return m_geometry->vertexPositions();
        }

        bool BrushFace::hasVertices(const vm::polygon3& vertices, const FloatType epsilon) const {
            ensure(m_geometry != nullptr, "geometry is null");
            return m_geometry->hasVertexPositions(vertices.vertices(), epsilon);
        }

        vm::polygon3 BrushFace::polygon() const {
            ensure(m_geometry != nullptr, "geometry is null");
            return vm::polygon3(vertexPositions());
        }

        BrushFaceGeometry* BrushFace::geometry() const {
            return m_geometry;
        }

        void BrushFace::setGeometry(BrushFaceGeometry* geometry) {
            m_geometry = geometry;
        }

        size_t BrushFace::lineNumber() const {
            return m_lineNumber;
        }

        void BrushFace::setFilePosition(const size_t lineNumber, const size_t lineCount) const {
            m_lineNumber = lineNumber;
            m_lineCount = lineCount;
        }

        bool BrushFace::selected() const {
            return m_selected;
        }

        void BrushFace::select() {
            assert(!m_selected);
            m_selected = true;
       }

        void BrushFace::deselect() {
            assert(m_selected);
            m_selected = false;
        }

        vm::vec2f BrushFace::textureCoords(const vm::vec3& point) const {
            return m_texCoordSystem->getTexCoords(point, m_attributes, textureSize());
        }

        FloatType BrushFace::intersectWithRay(const vm::ray3& ray) const {
            ensure(m_geometry != nullptr, "geometry is null");

            const FloatType cos = dot(m_boundary.normal, ray.direction);
            if (cos >= FloatType(0.0)) {
                return vm::nan<FloatType>();
            } else {
                return vm::intersect_ray_polygon(ray, m_boundary, m_geometry->boundary().begin(), m_geometry->boundary().end(), BrushGeometry::GetVertexPosition());
            }
        }

        kdl::result<void, BrushError> BrushFace::setPoints(const vm::vec3& point0, const vm::vec3& point1, const vm::vec3& point2) {
            m_points[0] = point0;
            m_points[1] = point1;
            m_points[2] = point2;
            correctPoints();

            const auto [result, plane] = vm::from_points(m_points[0], m_points[1], m_points[2]);
            if (result) {
                m_boundary = plane;
                return kdl::void_success;
            } else {
                return BrushError::InvalidFace;
            }
        }

        void BrushFace::correctPoints() {
            for (size_t i = 0; i < 3; ++i) {
                m_points[i] = correct(m_points[i]);
            }
        }

        void BrushFace::setMarked(const bool marked) const {
            m_markedToRenderFace = marked;
        }

        bool BrushFace::isMarked() const {
            return m_markedToRenderFace;
        }

        void BrushFace::doAcceptTagVisitor(TagVisitor& visitor) {
            visitor.visit(*this);
        }

        void BrushFace::doAcceptTagVisitor(ConstTagVisitor& visitor) const {
            visitor.visit(*this);
        }
    }
}
