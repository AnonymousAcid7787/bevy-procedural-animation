#import bevy_pbr::mesh_vertex_output

struct CustomMaterial {
    color: vec4<f32>,
};

@group(1) @binding(0)
var<uniform> material: CustomMaterial;
@group(1) @binding(1) 
var base_color_texture: texture_2d<f32>;
@group(1) @binding(2) 
var base_color_sampler: sampler;

@fragment
fn fragment(
    mesh: MeshVertexOutput,
) -> @location(0) vec4<f32> {
    return material.color * textureSample(base_color_texture, base_color_sampler, mesh.uv) * vec4<f32>(1.0, 1.0, 1.0, 0.5);
}