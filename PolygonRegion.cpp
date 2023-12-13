#include "PolygonRegion.h"
#include "ViWoRoot.h"
#include <memory>
#include "World.h"
#include "VectorDataRender.h"
#include "Event/EventManager.h"
#include "FPick.h"
#include "GLShaderUtils.h"

namespace VPE {
PolygonRegion::PolygonRegion() {
    vertices.push_back({{0.0, 1.0}});
    vertices.push_back({{0.0, -1.0}});
    vertices.push_back({{1.0, 0.0}});

    sdf = std::make_shared<PolygonRegionSDF>();
    Commit();
}

PolygonRegion PolygonRegion::GetComponent(const PolygonRegion &data, const DeserializeState &state) {
    auto comp = data;
    comp.Commit();
    return comp;
}

void PolygonRegion::Commit() {
    if (vertices.empty()) {
        aabb_min = aabb_max = {};
        return;
    }
    aabb_min = aabb_max = vertices[0].position;
    for (auto &v: vertices) {
        aabb_min.x = std::min(aabb_min.x, v.position.x);
        aabb_min.y = std::min(aabb_min.y, v.position.y);
        aabb_max.x = std::max(aabb_max.x, v.position.x);
        aabb_max.y = std::max(aabb_max.y, v.position.y);
    }
    sdf->Generate(*this);
}

bool PolygonRegion::Contains(const VPE::vec2 &v) const {
    if (v.x < aabb_min.x || v.y < aabb_min.y ||
        v.x > aabb_max.x || v.y > aabb_max.y) {
        return false;
    }

    int intersect_count = 0;
    int vert_count = vertices.size();
    for (int i = 0; i < vert_count; i++) {
        // Get line segment relative to v
        auto p0 = vertices[i].position - v;
        auto p1 = vertices[(i + 1) % vert_count].position - v;

        // Shoot a ray from v at direction (0, 1), calculate intersection
        // with the edge and the ray
        if (VPE::sign(p0.x) * VPE::sign(p1.x) > 0) {
            // End points are on the same side of Y-axis, no intersection.
            continue;
        }
        float k = (p0.y - p1.y) / (p0.x - p1.x);
        float b = p0.y - k * p0.x;
        if (b > 0) {
            intersect_count++;
        }
    }
    // The point is inside polygon if intersection count is odd
    return intersect_count % 2 > 0;
}

void PolygonRegion::DrawGizmos(const VPE::Transform &local_to_world) {
    auto vert_count = vertices.size();
    if (vert_count == 0) {
        return;
    }

    std::vector<VPE::dvec3> pts{};
    for (auto &v: vertices) {
        pts.push_back(LocalToWorldPosition(local_to_world, v.position));
    }

    IndicesVector indices{};
    for (int i = 0; i + 1 < vert_count; i++) {
        indices.push_back(i);
        indices.push_back(i + 1);
    }

    auto edit_state = PolygonRegionEditState::Get();
    bool mouse_inside = false;

    VPE::dvec3 pick_pos_ws{};
    if (edit_state->GetMousePickTerrianPosition(pick_pos_ws)) {
        mouse_inside = Contains(WorldToLocalPosition(local_to_world.Inverse(), pick_pos_ws));
    }

    auto width = [&](float v) {
        return mouse_inside ? 2.0 * v : v;
    };
    auto tint = [&](const VPE::Color &c) {
        if (mouse_inside) {
            return VPE::Color(VPE::mix(c.RGBf(), VPE::WHITE.RGBf(), 0.5f));
        }
        return c;
    };

    auto gizmos = ViWoROOT::GetVectorDataRender();
    gizmos->DrawPoints(pts, width(5.0), tint(VPE::GREEN), RenderModule::CoordMode::kCoordWorldSpace);
    gizmos->DrawLines(pts, indices, width(1.0), tint(VPE::RED), RenderModule::CoordMode::kCoordWorldSpace);
    gizmos->DrawLine(pts.front(), pts.back(), width(1.0), tint(VPE::CYAN), RenderModule::CoordMode::kCoordWorldSpace);
}

VPE::dvec3 PolygonRegion::LocalToWorldPosition(const VPE::Transform &local_to_world, const VPE::vec2 &p) {
    auto m = local_to_world.GetMatrix();
    return VPE::dvec3(m * VPE::dvec4(p.x, 10.0, p.y, 1.0));
}

VPE::vec2 PolygonRegion::WorldToLocalPosition(const VPE::Transform &world_to_local, const VPE::dvec3 &pos) {
    auto m = world_to_local.GetMatrix();
    auto local = m * VPE::dvec4(pos, 1.0);
    return VPE::vec2(local.x, local.z);
}

bool PolygonRegion::DrawInspector(VPE::Entity entity) {
    bool dirty = false;
    if (ImGui::Button("Clear Vertices")) {
        vertices.clear();
        Commit();
        dirty = true;
    }

    auto edit_state = PolygonRegionEditState::Get();
    bool is_edit = entity == edit_state->GetCurrentEntity();
    bool last_is_edit = is_edit;
    if (ImGui::Checkbox("Edit", &is_edit)) {
        if (is_edit) {
            edit_state->SetCurrent(entity);
        } else if (last_is_edit) {
            edit_state->SetCurrent({});
        }
    }

    ImGui::Text("Add Point: Left Click");
    ImGui::Text("Remove Last Point: Right Click");
    ImGui::Text("Finish Editing: Mid Click");

    if (ImGui::Button("Regenerate SDF")) {
        sdf->Generate(*this);
    }
    float display_width = std::min(256, sdf->Width());
    float display_height = sdf->Height() * display_width / sdf->Width();
    ImGui::Text("SDF Size %dx%d", sdf->Width(), sdf->Height());
    ImGui::Image(reinterpret_cast<ImTextureID>(sdf->SDFTextureId()), {display_width, display_height});

    return dirty;
}

void PolygonRegion::Update() {
}

void PolygonRegion::DrawGizmos() {
    auto world = ViWoROOT::World();
    for (auto e: world->view<PolygonRegion>()) {
        auto &region = world->get<PolygonRegion>(e);
        region.DrawGizmos(world->GetTransform(e));
    }
}

PolygonRegionEditState::PolygonRegionEditState() {
    EventManager::RegisterHandler<MouseEvent>(this);
}

bool PolygonRegionEditState::OnLButtonClick(int nFlags, int xPos, int yPos) {
    auto polygon = GetCurrentPolygonRegion();
    if (polygon == nullptr) {
        return false;
    }

    VPE::dvec3 pos_ws{};
    if (ViWoROOT::GetPickObj()->PickTerrainPoint(xPos, yPos, pos_ws) == 0) {
        return false;
    }

    auto world = ViWoROOT::World();
    auto xform = world->GetTransform(GetCurrentEntity());
    PolygonVertex vertex{};
    vertex.position = polygon->WorldToLocalPosition(xform.Inverse(), pos_ws);
    polygon->vertices.push_back(vertex);
    polygon->Commit();

    return true;
}

bool PolygonRegionEditState::OnMButtonClick(int nFlags, int xPos, int yPos) {
    SetCurrent({});
    return false;
}

bool PolygonRegionEditState::OnRButtonClick(int nFlags, int xPos, int yPos) {
    auto polygon = GetCurrentPolygonRegion();
    if (polygon == nullptr) {
        return false;
    }

    if (polygon->vertices.empty()) {
        return false;
    }
    polygon->vertices.pop_back();
    polygon->Commit();

    return true;
}

bool PolygonRegionEditState::OnMouseMove(int nFlags, int xPos, int yPos) {
    _mouse_x = xPos;
    _mouse_y = yPos;
    return false;
}

void PolygonRegionEditState::SetCurrent(VPE::Entity entity) {
    _current_entity = entity;
}

VPE::Entity PolygonRegionEditState::GetCurrentEntity() const {
    return _current_entity;
}

PolygonRegion *PolygonRegionEditState::GetCurrentPolygonRegion() const {
    auto world = ViWoROOT::World();
    auto entity = GetCurrentEntity();
    if (!world->valid(entity)) {
        return nullptr;
    }
    return world->try_get<PolygonRegion>(entity);
}

int PolygonRegionEditState::GetMouseX() const {
    return _mouse_x;
}

int PolygonRegionEditState::GetMouseY() const {
    return _mouse_y;
}

bool PolygonRegionEditState::GetMousePickTerrianPosition(VPE::dvec3 &pos_ws) const {
    return ViWoROOT::GetPickObj()->PickTerrainPoint(GetMouseX(), GetMouseY(), pos_ws);
}

void PolygonRegionEditState::Init() {
    [[maybe_unused]] auto inst = Get();
}

PolygonRegionEditState *PolygonRegionEditState::Get() {
    static auto s_state = std::make_unique<PolygonRegionEditState>();
    return s_state.get();
}

PolygonRegionSDF::PolygonRegionSDF() {
}

namespace {
class PolygonSDFProgram {
public:
    PolygonSDFProgram() {
        OpenGLUtils::ShaderConfig conf{};
        conf.Compute("Sand/GeneratePolygonSDF.comp");
        m_kernel = OpenGLUtils::CompileAndLinkShaderProgram(conf);
    }

    static PolygonSDFProgram *Get() {
        static auto s_prog = std::make_unique<PolygonSDFProgram>();
        return s_prog.get();
    }

    GLuint Id() const {
        return m_kernel.get();
    }

private:
    GLProgramObject m_kernel;
};
}  // namespace

void PolygonRegionSDF::Generate(const PolygonRegion &polygon) {
    auto physical_size = polygon.aabb_max - polygon.aabb_min;
    int width = max(int(ceil(physical_size.x / polygon.sdf_grid_size)), 1);
    int height = max(int(ceil(physical_size.y / polygon.sdf_grid_size)), 1);

    if (m_width != width || m_height != height) {
        m_sdfTex.alloc();
        glBindTexture(GL_TEXTURE_2D, m_sdfTex.get());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, width, height);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // Update self params
    m_width = width;
    m_height = height;
    m_gridPhysicalSize = polygon.sdf_grid_size;

    struct Param {
        int vertex_count;
        int width;
        int height;
        float grid_physical_size;
        VPE::vec2 aabb_min;
    };

    Param param{};
    param.vertex_count = polygon.vertices.size();
    param.width = width;
    param.height = height;
    param.grid_physical_size = polygon.sdf_grid_size;
    param.aabb_min = polygon.aabb_min;

    m_paramBuffer.allocIfNull();
    glBindBuffer(GL_UNIFORM_BUFFER, m_paramBuffer.get());
    glBufferData(GL_UNIFORM_BUFFER, sizeof(Param), &param, GL_STATIC_DRAW);

    std::vector<VPE::vec2> vertices{};
    vertices.reserve(polygon.vertices.size());
    for (auto &v: polygon.vertices) {
        vertices.push_back(v.position);
    }
    m_polygonBuffer.allocIfNull();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_polygonBuffer.get());
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 sizeof(VPE::vec2) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

    glUseProgram(PolygonSDFProgram::Get()->Id());
    GLuint sdf_image = m_sdfTex.get();
    glBindImageTextures(0, 1, &sdf_image);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, m_paramBuffer.get());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_polygonBuffer.get());

    int group_x = (width + 31) / 32;
    int group_y = (height + 31) / 32;
    glDispatchCompute(group_x, group_y, 1);
}

int PolygonRegionSDF::Width() const {
    return m_width;
}

int PolygonRegionSDF::Height() const {
    return m_height;
}

float PolygonRegionSDF::GridPhysicalSize() const {
    return m_gridPhysicalSize;
}

GLuint PolygonRegionSDF::SDFTextureId() const {
    return m_sdfTex.get();
}
}  // namespace VPE