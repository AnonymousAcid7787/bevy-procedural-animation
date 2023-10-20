#import bevy_pbr::mesh_vertex_output

const pi: f32 = 3.14159265;

@group(1) @binding(0)
var<storage, read> points: array<f32>;

fn draw_line(
    p1: vec2<f32>, 
    p2: vec2<f32>, 
    uv: vec2<f32>, 
    line_thickness: f32, 
    // one_px is the width of a singular pixel in UV
    one_px: f32,
) -> f32 {
    let r = 0.;
    
    let d = distance(p1, p2);

    //distance between current pixel and p1 (uv is the current pixel in viewport coords)
    let d_uv = distance(uv, p1);

    //calculating the point to draw up to
    //  so if distance between two points in line is 4, and the interpolation_percent is 0.75, 
    //  the line will look like this:
    //  o--- o
    //  1.0 interpolation_percent will look like:
    //  o----o
    //  with the full length of 4 units
    let interpolation_percent = clamp(d_uv/d, 0., 1.);
    let draw_to_point = mix(p1, p2, interpolation_percent);

    return 1. - floor(1. - (line_thickness*one_px) + distance(uv, draw_to_point));
}

@fragment
fn fragment(
    mesh: MeshVertexOutput
) -> @location(0) vec4<f32> {
    // return vec4<f32>(lines*line_alpha, 0., 0., 1.);
    return vec4<f32>(0., 0., 0., 0.);
}