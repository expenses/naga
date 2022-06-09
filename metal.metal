// language: metal2.0
#include <metal_stdlib>
#include <simd/simd.h>

using metal::uint;

struct _mslBufferSizes {
    uint size7;
    uint size12;
    uint size13;
    uint size14;
};

struct Vertex {
    float x;
    float y;
    float z;
    float u;
    float v;
    float nx;
    float ny;
    float nz;
    float tx;
    float ty;
    float tz;
    float bx;
    float by;
    float bz;
};
struct type_13 {
    metal::float3 inner[64u];
};
struct type_17 {
    metal::float2 inner[64u];
};
struct type_20 {
    float inner[1u];
};
struct type_21 {
    metal::float4 inner[4u];
};
struct type_22 {
    type_20 inner[4u];
};
struct gl_MeshPerVertexNV {
    metal::float4 gl_Position;
    float gl_PointSize;
    type_20 gl_ClipDistance;
    type_20 gl_CullDistance;
    char _pad4[4];
    type_21 gl_PositionPerViewNV;
    type_22 gl_ClipDistancePerViewNV;
    type_22 gl_CullDistancePerViewNV;
};
struct type_23 {
    gl_MeshPerVertexNV inner[64u];
};
typedef metal::uint4 type_29[1];
struct MeshletBufferObj {
    type_29 meshlet_descs;
};
struct type_32 {
    metal::float4x4 inner[100u];
};
struct UniformBufferObject {
    type_32 model;
};
struct UniformBufferCameraObject {
    metal::float4x4 view;
    metal::float4x4 proj;
};
struct type_37 {
    uint inner[63u];
};
typedef uint type_40[1];
struct FlatIndexBufferObj {
    type_40 indices;
};
typedef Vertex type_43[1];
struct VertexBufferObj {
    type_43 vertices;
};
typedef uint type_45[1];
struct VertexIndexBufferObj {
    type_45 indices;
};
struct type_49 {
    type_13 member;
    type_13 member_1;
    type_13 member_2;
    type_13 member_3;
    type_17 member_4;
    type_23 member_5;
    type_37 member_6;
    uint member_7;
};
constant metal::uint3 const_type_26_ = {32u, 1u, 1u};

void EmitVertexu1structVertexf1f1f1f1f1f1f1f1f1f1f1f1f1f11mf44mf44u1_(
    thread uint& idx,
    thread Vertex& vertex_,
    thread metal::float4x4& model,
    thread metal::float4x4& pv,
    thread uint& meshlet_id,
    thread type_13& g_frag_pos,
    thread type_13& g_normal,
    thread type_13& g_tangent,
    thread type_13& g_bitangent,
    thread type_17& g_uv,
    thread type_23& gl_MeshVerticesNV
) {
    metal::float4 world_pos = {};
    metal::float4 world_normal = {};
    metal::float4x4 _e57 = model;
    float _e59 = vertex_.x;
    float _e61 = vertex_.y;
    float _e63 = vertex_.z;
    world_pos = _e57 * metal::float4(_e59, _e61, _e63, 1.0);
    metal::float4x4 _e66 = model;
    float _e68 = vertex_.nx;
    float _e70 = vertex_.ny;
    float _e72 = vertex_.nz;
    world_normal = metal::normalize(_e66 * metal::float4(_e68, _e70, _e72, 0.0));
    uint _e76 = idx;
    metal::float4 _e77 = world_pos;
    g_frag_pos.inner[_e76] = metal::float3(_e77.x, _e77.y, _e77.z);
    uint _e83 = idx;
    metal::float4 _e84 = world_normal;
    g_normal.inner[_e83] = _e84.xyz;
    uint _e87 = idx;
    metal::float4x4 _e88 = model;
    float _e90 = vertex_.tx;
    float _e92 = vertex_.ty;
    float _e94 = vertex_.tz;
    g_tangent.inner[_e87] = metal::normalize(_e88 * metal::float4(_e90, _e92, _e94, 0.0)).xyz;
    uint _e100 = idx;
    metal::float4x4 _e101 = model;
    float _e103 = vertex_.bx;
    float _e105 = vertex_.by;
    float _e107 = vertex_.bz;
    g_bitangent.inner[_e100] = metal::normalize(_e101 * metal::float4(_e103, _e105, _e107, 0.0)).xyz;
    uint _e113 = idx;
    float _e115 = vertex_.u;
    float _e117 = vertex_.v;
    g_uv.inner[_e113] = metal::float2(_e115, _e117 * -1.0);
    uint _e121 = idx;
    metal::float4x4 _e122 = pv;
    metal::float4 _e123 = world_pos;
    gl_MeshVerticesNV.inner[_e121].gl_Position = _e122 * _e123;
    return;
}

void DecodeMeshletvu4u1u1u1u1_(
    thread metal::uint4& meshlet_desc,
    thread uint& vert_max,
    thread uint& prim_max,
    thread uint& vert_begin,
    thread uint& prim_begin
) {
    uint _e56 = meshlet_desc[2u];
    vert_begin = (_e56 & 1048575u) * 16u;
    uint _e60 = meshlet_desc[3u];
    prim_begin = (_e60 & 1048575u) * 1u;
    uint _e64 = meshlet_desc[0u];
    vert_max = _e64 >> as_type<uint>(24);
    uint _e68 = meshlet_desc[1u];
    prim_max = _e68 >> as_type<uint>(24);
    return;
}

void main_1(
    thread type_13& g_frag_pos,
    thread type_13& g_normal,
    thread type_13& g_tangent,
    thread type_13& g_bitangent,
    thread type_17& g_uv,
    thread type_23& gl_MeshVerticesNV,
    thread metal::uint3& gl_WorkGroupID_1,
    device MeshletBufferObj const& mb,
    constant UniformBufferObject& ubo,
    constant UniformBufferCameraObject& camera,
    thread metal::uint3& gl_LocalInvocationID_1,
    thread type_37& gl_PrimitiveIndicesNV,
    device FlatIndexBufferObj const& ibi,
    device VertexBufferObj const& vb,
    device VertexIndexBufferObj const& vbi,
    thread uint& gl_PrimitiveCountNV,
    constant _mslBufferSizes& _buffer_sizes
) {
    uint meshlet_id_1 = {};
    metal::uint4 meshlet_desc_1 = {};
    uint vert_max_1 = {};
    uint prim_max_1 = {};
    uint vert_begin_1 = {};
    uint prim_begin_1 = {};
    metal::uint4 param = {};
    uint param_1 = {};
    uint param_2 = {};
    uint param_3 = {};
    uint param_4 = {};
    metal::float4x4 model_1 = {};
    metal::float4x4 pv_1 = {};
    uint num_indices = {};
    uint indices_start = {};
    uint i = {};
    uint pre_k = {};
    uint k = {};
    uint ii = {};
    uint i_1 = {};
    uint k_1 = {};
    uint ii_1 = {};
    Vertex v0_ = {};
    uint param_5 = {};
    Vertex param_6 = {};
    metal::float4x4 param_7 = {};
    metal::float4x4 param_8 = {};
    uint param_9 = {};
    uint _e79 = gl_WorkGroupID_1[0u];
    meshlet_id_1 = _e79;
    uint _e80 = meshlet_id_1;
    metal::uint4 _e83 = mb.meshlet_descs[_e80];
    meshlet_desc_1 = _e83;
    metal::uint4 _e84 = meshlet_desc_1;
    param = _e84;
    DecodeMeshletvu4u1u1u1u1_(param, param_1, param_2, param_3, param_4);
    uint _e85 = param_1;
    vert_max_1 = _e85;
    uint _e86 = param_2;
    prim_max_1 = _e86;
    uint _e87 = param_3;
    vert_begin_1 = _e87;
    uint _e88 = param_4;
    prim_begin_1 = _e88;
    uint _e89 = vert_max_1;
    vert_max_1 = _e89 + 1u;
    uint _e91 = prim_max_1;
    prim_max_1 = _e91 + 1u;
    metal::float4x4 _e95 = ubo.model.inner[0];
    model_1 = _e95;
    metal::float4x4 _e97 = camera.proj;
    metal::float4x4 _e99 = camera.view;
    pv_1 = _e97 * _e99;
    uint _e101 = prim_max_1;
    num_indices = _e101 * 3u;
    uint _e103 = prim_begin_1;
    indices_start = _e103 * 3u;
    i = 0u;
    bool loop_init = true;
    while(true) {
        if (!loop_init) {
            uint _e147 = i;
            i = _e147 + as_type<uint>(1);
        }
        loop_init = false;
        uint _e105 = i;
        if (_e105 < 1u) {
            uint _e108 = gl_LocalInvocationID_1[0u];
            uint _e109 = i;
            uint _e112 = prim_max_1;
            pre_k = metal::min(_e108 + (_e109 * 32u), _e112 - 1u);
            uint _e115 = pre_k;
            k = _e115 * 3u;
            uint _e117 = indices_start;
            uint _e118 = k;
            ii = _e117 + _e118;
            uint _e120 = k;
            uint _e122 = ii;
            uint _e126 = ibi.indices[_e122 + 0u];
            gl_PrimitiveIndicesNV.inner[_e120 + 0u] = as_type<uint>(_e126);
            uint _e129 = k;
            uint _e131 = ii;
            uint _e135 = ibi.indices[_e131 + 1u];
            gl_PrimitiveIndicesNV.inner[_e129 + 1u] = as_type<uint>(_e135);
            uint _e138 = k;
            uint _e140 = ii;
            uint _e144 = ibi.indices[_e140 + 2u];
            gl_PrimitiveIndicesNV.inner[_e138 + 2u] = as_type<uint>(_e144);
            continue;
        } else {
            break;
        }
    }
    i_1 = 0u;
    bool loop_init_1 = true;
    while(true) {
        if (!loop_init_1) {
            uint _e203 = i_1;
            i_1 = _e203 + as_type<uint>(1);
        }
        loop_init_1 = false;
        uint _e150 = i_1;
        if (_e150 < 2u) {
            uint _e153 = gl_LocalInvocationID_1[0u];
            uint _e154 = i_1;
            uint _e157 = vert_max_1;
            k_1 = metal::min(_e153 + (_e154 * 32u), _e157 - 1u);
            uint _e160 = vert_begin_1;
            uint _e161 = k_1;
            ii_1 = _e160 + _e161;
            uint _e163 = ii_1;
            uint _e166 = vbi.indices[_e163];
            Vertex _e169 = vb.vertices[_e166];
            v0_.x = _e169.x;
            v0_.y = _e169.y;
            v0_.z = _e169.z;
            v0_.u = _e169.u;
            v0_.v = _e169.v;
            v0_.nx = _e169.nx;
            v0_.ny = _e169.ny;
            v0_.nz = _e169.nz;
            v0_.tx = _e169.tx;
            v0_.ty = _e169.ty;
            v0_.tz = _e169.tz;
            v0_.bx = _e169.bx;
            v0_.by = _e169.by;
            v0_.bz = _e169.bz;
            uint _e198 = k_1;
            param_5 = _e198;
            Vertex _e199 = v0_;
            param_6 = _e199;
            metal::float4x4 _e200 = model_1;
            param_7 = _e200;
            metal::float4x4 _e201 = pv_1;
            param_8 = _e201;
            uint _e202 = meshlet_id_1;
            param_9 = _e202;
            EmitVertexu1structVertexf1f1f1f1f1f1f1f1f1f1f1f1f1f11mf44mf44u1_(param_5, param_6, param_7, param_8, param_9, g_frag_pos, g_normal, g_tangent, g_bitangent, g_uv, gl_MeshVerticesNV);
            continue;
        } else {
            break;
        }
    }
    uint _e206 = prim_max_1;
    gl_PrimitiveCountNV = _e206;
    return;
}

struct main_Input {
};
struct main_Output {
    metal::float3 member [[user(loc2)]];
    metal::float3 member_1 [[user(loc1)]];
    metal::float3 member_2 [[user(loc3)]];
    metal::float3 member_3 [[user(loc4)]];
    metal::float2 member_4 [[user(loc0)]];
    gl_MeshPerVertexNV member_5 [[]];
;
;
};
[[mesh]] main_Output main_(
  metal::uint3 gl_WorkGroupID [[threadgroup_position_in_grid]]
, metal::uint3 gl_LocalInvocationID [[thread_position_in_threadgroup]]
, device MeshletBufferObj const& mb [[user(fake0)]]
, constant UniformBufferObject& ubo [[user(fake0)]]
, constant UniformBufferCameraObject& camera [[user(fake0)]]
, device FlatIndexBufferObj const& ibi [[user(fake0)]]
, device VertexBufferObj const& vb [[user(fake0)]]
, device VertexIndexBufferObj const& vbi [[user(fake0)]]
, constant _mslBufferSizes& _buffer_sizes [[user(fake0)]]
) {
    type_13 g_frag_pos = {};
    type_13 g_normal = {};
    type_13 g_tangent = {};
    type_13 g_bitangent = {};
    type_17 g_uv = {};
    type_23 gl_MeshVerticesNV = {};
    metal::uint3 gl_WorkGroupID_1 = {};
    metal::uint3 gl_LocalInvocationID_1 = {};
    type_37 gl_PrimitiveIndicesNV = {};
    uint gl_PrimitiveCountNV = {};
    gl_WorkGroupID_1 = gl_WorkGroupID;
    gl_LocalInvocationID_1 = gl_LocalInvocationID;
    main_1(g_frag_pos, g_normal, g_tangent, g_bitangent, g_uv, gl_MeshVerticesNV, gl_WorkGroupID_1, mb, ubo, camera, gl_LocalInvocationID_1, gl_PrimitiveIndicesNV, ibi, vb, vbi, gl_PrimitiveCountNV, _buffer_sizes);
    type_13 _e12 = g_frag_pos;
    type_13 _e13 = g_normal;
    type_13 _e14 = g_tangent;
    type_13 _e15 = g_bitangent;
    type_17 _e16 = g_uv;
    type_23 _e17 = gl_MeshVerticesNV;
    type_37 _e18 = gl_PrimitiveIndicesNV;
    uint _e19 = gl_PrimitiveCountNV;
    const auto _tmp = type_49 {_e12, _e13, _e14, _e15, _e16, _e17, _e18, _e19};
    return main_Output { {_tmp.member.inner[0],_tmp.member.inner[1],_tmp.member.inner[2],_tmp.member.inner[3],_tmp.member.inner[4],_tmp.member.inner[5],_tmp.member.inner[6],_tmp.member.inner[7],_tmp.member.inner[8],_tmp.member.inner[9],_tmp.member.inner[10],_tmp.member.inner[11],_tmp.member.inner[12],_tmp.member.inner[13],_tmp.member.inner[14],_tmp.member.inner[15],_tmp.member.inner[16],_tmp.member.inner[17],_tmp.member.inner[18],_tmp.member.inner[19],_tmp.member.inner[20],_tmp.member.inner[21],_tmp.member.inner[22],_tmp.member.inner[23],_tmp.member.inner[24],_tmp.member.inner[25],_tmp.member.inner[26],_tmp.member.inner[27],_tmp.member.inner[28],_tmp.member.inner[29],_tmp.member.inner[30],_tmp.member.inner[31],_tmp.member.inner[32],_tmp.member.inner[33],_tmp.member.inner[34],_tmp.member.inner[35],_tmp.member.inner[36],_tmp.member.inner[37],_tmp.member.inner[38],_tmp.member.inner[39],_tmp.member.inner[40],_tmp.member.inner[41],_tmp.member.inner[42],_tmp.member.inner[43],_tmp.member.inner[44],_tmp.member.inner[45],_tmp.member.inner[46],_tmp.member.inner[47],_tmp.member.inner[48],_tmp.member.inner[49],_tmp.member.inner[50],_tmp.member.inner[51],_tmp.member.inner[52],_tmp.member.inner[53],_tmp.member.inner[54],_tmp.member.inner[55],_tmp.member.inner[56],_tmp.member.inner[57],_tmp.member.inner[58],_tmp.member.inner[59],_tmp.member.inner[60],_tmp.member.inner[61],_tmp.member.inner[62],_tmp.member.inner[63]}, {_tmp.member_1.inner[0],_tmp.member_1.inner[1],_tmp.member_1.inner[2],_tmp.member_1.inner[3],_tmp.member_1.inner[4],_tmp.member_1.inner[5],_tmp.member_1.inner[6],_tmp.member_1.inner[7],_tmp.member_1.inner[8],_tmp.member_1.inner[9],_tmp.member_1.inner[10],_tmp.member_1.inner[11],_tmp.member_1.inner[12],_tmp.member_1.inner[13],_tmp.member_1.inner[14],_tmp.member_1.inner[15],_tmp.member_1.inner[16],_tmp.member_1.inner[17],_tmp.member_1.inner[18],_tmp.member_1.inner[19],_tmp.member_1.inner[20],_tmp.member_1.inner[21],_tmp.member_1.inner[22],_tmp.member_1.inner[23],_tmp.member_1.inner[24],_tmp.member_1.inner[25],_tmp.member_1.inner[26],_tmp.member_1.inner[27],_tmp.member_1.inner[28],_tmp.member_1.inner[29],_tmp.member_1.inner[30],_tmp.member_1.inner[31],_tmp.member_1.inner[32],_tmp.member_1.inner[33],_tmp.member_1.inner[34],_tmp.member_1.inner[35],_tmp.member_1.inner[36],_tmp.member_1.inner[37],_tmp.member_1.inner[38],_tmp.member_1.inner[39],_tmp.member_1.inner[40],_tmp.member_1.inner[41],_tmp.member_1.inner[42],_tmp.member_1.inner[43],_tmp.member_1.inner[44],_tmp.member_1.inner[45],_tmp.member_1.inner[46],_tmp.member_1.inner[47],_tmp.member_1.inner[48],_tmp.member_1.inner[49],_tmp.member_1.inner[50],_tmp.member_1.inner[51],_tmp.member_1.inner[52],_tmp.member_1.inner[53],_tmp.member_1.inner[54],_tmp.member_1.inner[55],_tmp.member_1.inner[56],_tmp.member_1.inner[57],_tmp.member_1.inner[58],_tmp.member_1.inner[59],_tmp.member_1.inner[60],_tmp.member_1.inner[61],_tmp.member_1.inner[62],_tmp.member_1.inner[63]}, {_tmp.member_2.inner[0],_tmp.member_2.inner[1],_tmp.member_2.inner[2],_tmp.member_2.inner[3],_tmp.member_2.inner[4],_tmp.member_2.inner[5],_tmp.member_2.inner[6],_tmp.member_2.inner[7],_tmp.member_2.inner[8],_tmp.member_2.inner[9],_tmp.member_2.inner[10],_tmp.member_2.inner[11],_tmp.member_2.inner[12],_tmp.member_2.inner[13],_tmp.member_2.inner[14],_tmp.member_2.inner[15],_tmp.member_2.inner[16],_tmp.member_2.inner[17],_tmp.member_2.inner[18],_tmp.member_2.inner[19],_tmp.member_2.inner[20],_tmp.member_2.inner[21],_tmp.member_2.inner[22],_tmp.member_2.inner[23],_tmp.member_2.inner[24],_tmp.member_2.inner[25],_tmp.member_2.inner[26],_tmp.member_2.inner[27],_tmp.member_2.inner[28],_tmp.member_2.inner[29],_tmp.member_2.inner[30],_tmp.member_2.inner[31],_tmp.member_2.inner[32],_tmp.member_2.inner[33],_tmp.member_2.inner[34],_tmp.member_2.inner[35],_tmp.member_2.inner[36],_tmp.member_2.inner[37],_tmp.member_2.inner[38],_tmp.member_2.inner[39],_tmp.member_2.inner[40],_tmp.member_2.inner[41],_tmp.member_2.inner[42],_tmp.member_2.inner[43],_tmp.member_2.inner[44],_tmp.member_2.inner[45],_tmp.member_2.inner[46],_tmp.member_2.inner[47],_tmp.member_2.inner[48],_tmp.member_2.inner[49],_tmp.member_2.inner[50],_tmp.member_2.inner[51],_tmp.member_2.inner[52],_tmp.member_2.inner[53],_tmp.member_2.inner[54],_tmp.member_2.inner[55],_tmp.member_2.inner[56],_tmp.member_2.inner[57],_tmp.member_2.inner[58],_tmp.member_2.inner[59],_tmp.member_2.inner[60],_tmp.member_2.inner[61],_tmp.member_2.inner[62],_tmp.member_2.inner[63]}, {_tmp.member_3.inner[0],_tmp.member_3.inner[1],_tmp.member_3.inner[2],_tmp.member_3.inner[3],_tmp.member_3.inner[4],_tmp.member_3.inner[5],_tmp.member_3.inner[6],_tmp.member_3.inner[7],_tmp.member_3.inner[8],_tmp.member_3.inner[9],_tmp.member_3.inner[10],_tmp.member_3.inner[11],_tmp.member_3.inner[12],_tmp.member_3.inner[13],_tmp.member_3.inner[14],_tmp.member_3.inner[15],_tmp.member_3.inner[16],_tmp.member_3.inner[17],_tmp.member_3.inner[18],_tmp.member_3.inner[19],_tmp.member_3.inner[20],_tmp.member_3.inner[21],_tmp.member_3.inner[22],_tmp.member_3.inner[23],_tmp.member_3.inner[24],_tmp.member_3.inner[25],_tmp.member_3.inner[26],_tmp.member_3.inner[27],_tmp.member_3.inner[28],_tmp.member_3.inner[29],_tmp.member_3.inner[30],_tmp.member_3.inner[31],_tmp.member_3.inner[32],_tmp.member_3.inner[33],_tmp.member_3.inner[34],_tmp.member_3.inner[35],_tmp.member_3.inner[36],_tmp.member_3.inner[37],_tmp.member_3.inner[38],_tmp.member_3.inner[39],_tmp.member_3.inner[40],_tmp.member_3.inner[41],_tmp.member_3.inner[42],_tmp.member_3.inner[43],_tmp.member_3.inner[44],_tmp.member_3.inner[45],_tmp.member_3.inner[46],_tmp.member_3.inner[47],_tmp.member_3.inner[48],_tmp.member_3.inner[49],_tmp.member_3.inner[50],_tmp.member_3.inner[51],_tmp.member_3.inner[52],_tmp.member_3.inner[53],_tmp.member_3.inner[54],_tmp.member_3.inner[55],_tmp.member_3.inner[56],_tmp.member_3.inner[57],_tmp.member_3.inner[58],_tmp.member_3.inner[59],_tmp.member_3.inner[60],_tmp.member_3.inner[61],_tmp.member_3.inner[62],_tmp.member_3.inner[63]}, {_tmp.member_4.inner[0],_tmp.member_4.inner[1],_tmp.member_4.inner[2],_tmp.member_4.inner[3],_tmp.member_4.inner[4],_tmp.member_4.inner[5],_tmp.member_4.inner[6],_tmp.member_4.inner[7],_tmp.member_4.inner[8],_tmp.member_4.inner[9],_tmp.member_4.inner[10],_tmp.member_4.inner[11],_tmp.member_4.inner[12],_tmp.member_4.inner[13],_tmp.member_4.inner[14],_tmp.member_4.inner[15],_tmp.member_4.inner[16],_tmp.member_4.inner[17],_tmp.member_4.inner[18],_tmp.member_4.inner[19],_tmp.member_4.inner[20],_tmp.member_4.inner[21],_tmp.member_4.inner[22],_tmp.member_4.inner[23],_tmp.member_4.inner[24],_tmp.member_4.inner[25],_tmp.member_4.inner[26],_tmp.member_4.inner[27],_tmp.member_4.inner[28],_tmp.member_4.inner[29],_tmp.member_4.inner[30],_tmp.member_4.inner[31],_tmp.member_4.inner[32],_tmp.member_4.inner[33],_tmp.member_4.inner[34],_tmp.member_4.inner[35],_tmp.member_4.inner[36],_tmp.member_4.inner[37],_tmp.member_4.inner[38],_tmp.member_4.inner[39],_tmp.member_4.inner[40],_tmp.member_4.inner[41],_tmp.member_4.inner[42],_tmp.member_4.inner[43],_tmp.member_4.inner[44],_tmp.member_4.inner[45],_tmp.member_4.inner[46],_tmp.member_4.inner[47],_tmp.member_4.inner[48],_tmp.member_4.inner[49],_tmp.member_4.inner[50],_tmp.member_4.inner[51],_tmp.member_4.inner[52],_tmp.member_4.inner[53],_tmp.member_4.inner[54],_tmp.member_4.inner[55],_tmp.member_4.inner[56],_tmp.member_4.inner[57],_tmp.member_4.inner[58],_tmp.member_4.inner[59],_tmp.member_4.inner[60],_tmp.member_4.inner[61],_tmp.member_4.inner[62],_tmp.member_4.inner[63]}, {_tmp.member_5.inner[0],_tmp.member_5.inner[1],_tmp.member_5.inner[2],_tmp.member_5.inner[3],_tmp.member_5.inner[4],_tmp.member_5.inner[5],_tmp.member_5.inner[6],_tmp.member_5.inner[7],_tmp.member_5.inner[8],_tmp.member_5.inner[9],_tmp.member_5.inner[10],_tmp.member_5.inner[11],_tmp.member_5.inner[12],_tmp.member_5.inner[13],_tmp.member_5.inner[14],_tmp.member_5.inner[15],_tmp.member_5.inner[16],_tmp.member_5.inner[17],_tmp.member_5.inner[18],_tmp.member_5.inner[19],_tmp.member_5.inner[20],_tmp.member_5.inner[21],_tmp.member_5.inner[22],_tmp.member_5.inner[23],_tmp.member_5.inner[24],_tmp.member_5.inner[25],_tmp.member_5.inner[26],_tmp.member_5.inner[27],_tmp.member_5.inner[28],_tmp.member_5.inner[29],_tmp.member_5.inner[30],_tmp.member_5.inner[31],_tmp.member_5.inner[32],_tmp.member_5.inner[33],_tmp.member_5.inner[34],_tmp.member_5.inner[35],_tmp.member_5.inner[36],_tmp.member_5.inner[37],_tmp.member_5.inner[38],_tmp.member_5.inner[39],_tmp.member_5.inner[40],_tmp.member_5.inner[41],_tmp.member_5.inner[42],_tmp.member_5.inner[43],_tmp.member_5.inner[44],_tmp.member_5.inner[45],_tmp.member_5.inner[46],_tmp.member_5.inner[47],_tmp.member_5.inner[48],_tmp.member_5.inner[49],_tmp.member_5.inner[50],_tmp.member_5.inner[51],_tmp.member_5.inner[52],_tmp.member_5.inner[53],_tmp.member_5.inner[54],_tmp.member_5.inner[55],_tmp.member_5.inner[56],_tmp.member_5.inner[57],_tmp.member_5.inner[58],_tmp.member_5.inner[59],_tmp.member_5.inner[60],_tmp.member_5.inner[61],_tmp.member_5.inner[62],_tmp.member_5.inner[63]}, {_tmp.member_6.inner[0],_tmp.member_6.inner[1],_tmp.member_6.inner[2],_tmp.member_6.inner[3],_tmp.member_6.inner[4],_tmp.member_6.inner[5],_tmp.member_6.inner[6],_tmp.member_6.inner[7],_tmp.member_6.inner[8],_tmp.member_6.inner[9],_tmp.member_6.inner[10],_tmp.member_6.inner[11],_tmp.member_6.inner[12],_tmp.member_6.inner[13],_tmp.member_6.inner[14],_tmp.member_6.inner[15],_tmp.member_6.inner[16],_tmp.member_6.inner[17],_tmp.member_6.inner[18],_tmp.member_6.inner[19],_tmp.member_6.inner[20],_tmp.member_6.inner[21],_tmp.member_6.inner[22],_tmp.member_6.inner[23],_tmp.member_6.inner[24],_tmp.member_6.inner[25],_tmp.member_6.inner[26],_tmp.member_6.inner[27],_tmp.member_6.inner[28],_tmp.member_6.inner[29],_tmp.member_6.inner[30],_tmp.member_6.inner[31],_tmp.member_6.inner[32],_tmp.member_6.inner[33],_tmp.member_6.inner[34],_tmp.member_6.inner[35],_tmp.member_6.inner[36],_tmp.member_6.inner[37],_tmp.member_6.inner[38],_tmp.member_6.inner[39],_tmp.member_6.inner[40],_tmp.member_6.inner[41],_tmp.member_6.inner[42],_tmp.member_6.inner[43],_tmp.member_6.inner[44],_tmp.member_6.inner[45],_tmp.member_6.inner[46],_tmp.member_6.inner[47],_tmp.member_6.inner[48],_tmp.member_6.inner[49],_tmp.member_6.inner[50],_tmp.member_6.inner[51],_tmp.member_6.inner[52],_tmp.member_6.inner[53],_tmp.member_6.inner[54],_tmp.member_6.inner[55],_tmp.member_6.inner[56],_tmp.member_6.inner[57],_tmp.member_6.inner[58],_tmp.member_6.inner[59],_tmp.member_6.inner[60],_tmp.member_6.inner[61],_tmp.member_6.inner[62]}, _tmp.member_7 };
}
