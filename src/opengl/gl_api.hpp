/*
Module: rx7900.opengl.gl_api
Purpose: Fixed-width OpenGL 1.1 scalar types and the public enumerant values the
physical Radeon ICD accepts.
Created: 2026-10-23
Architecture: Shared by the portable GL context, the software reference
rasterizer, the GFX11 backend and the ReactOS ICD dispatch table. Values are the
public Khronos OpenGL 1.1 (and BGRA_EXT / CLAMP_TO_EDGE) enumerants; see
licenses/OPENGL.txt. No Windows header is required.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900::opengl::gl {

using Enum = uint32_t;
using Boolean = uint8_t;
using Bitfield = uint32_t;
using Byte = int8_t;
using Short = int16_t;
using Int = int32_t;
using Sizei = int32_t;
using Ubyte = uint8_t;
using Ushort = uint16_t;
using Uint = uint32_t;
using Float = float;
using Double = double;

static_assert(sizeof(Float) == 4 && sizeof(Double) == 8, "GL scalar ABI");

// Errors.
constexpr Enum no_error = 0;
constexpr Enum invalid_enum = 0x0500;
constexpr Enum invalid_value = 0x0501;
constexpr Enum invalid_operation = 0x0502;
constexpr Enum stack_overflow = 0x0503;
constexpr Enum stack_underflow = 0x0504;
constexpr Enum out_of_memory = 0x0505;

// Primitives.
constexpr Enum points = 0x0000;
constexpr Enum lines = 0x0001;
constexpr Enum line_loop = 0x0002;
constexpr Enum line_strip = 0x0003;
constexpr Enum triangles = 0x0004;
constexpr Enum triangle_strip = 0x0005;
constexpr Enum triangle_fan = 0x0006;
constexpr Enum quads = 0x0007;
constexpr Enum quad_strip = 0x0008;
constexpr Enum polygon = 0x0009;

// Data types.
constexpr Enum type_byte = 0x1400;
constexpr Enum type_unsigned_byte = 0x1401;
constexpr Enum type_short = 0x1402;
constexpr Enum type_unsigned_short = 0x1403;
constexpr Enum type_int = 0x1404;
constexpr Enum type_unsigned_int = 0x1405;
constexpr Enum type_float = 0x1406;
constexpr Enum type_2_bytes = 0x1407;
constexpr Enum type_3_bytes = 0x1408;
constexpr Enum type_4_bytes = 0x1409;
constexpr Enum type_double = 0x140a;
constexpr Enum type_bitmap = 0x1a00;

// Comparison functions.
constexpr Enum never = 0x0200;
constexpr Enum less = 0x0201;
constexpr Enum equal = 0x0202;
constexpr Enum lequal = 0x0203;
constexpr Enum greater = 0x0204;
constexpr Enum notequal = 0x0205;
constexpr Enum gequal = 0x0206;
constexpr Enum always = 0x0207;

// Blend factors.
constexpr Enum zero = 0;
constexpr Enum one = 1;
constexpr Enum src_color = 0x0300;
constexpr Enum one_minus_src_color = 0x0301;
constexpr Enum src_alpha = 0x0302;
constexpr Enum one_minus_src_alpha = 0x0303;
constexpr Enum dst_alpha = 0x0304;
constexpr Enum one_minus_dst_alpha = 0x0305;
constexpr Enum dst_color = 0x0306;
constexpr Enum one_minus_dst_color = 0x0307;
constexpr Enum src_alpha_saturate = 0x0308;

// Stencil operations; logic operations share INVERT.
constexpr Enum keep = 0x1e00;
constexpr Enum replace = 0x1e01;
constexpr Enum incr = 0x1e02;
constexpr Enum decr = 0x1e03;
constexpr Enum logic_clear = 0x1500;
constexpr Enum logic_and = 0x1501;
constexpr Enum logic_and_reverse = 0x1502;
constexpr Enum logic_copy = 0x1503;
constexpr Enum logic_and_inverted = 0x1504;
constexpr Enum logic_noop = 0x1505;
constexpr Enum logic_xor = 0x1506;
constexpr Enum logic_or = 0x1507;
constexpr Enum logic_nor = 0x1508;
constexpr Enum logic_equiv = 0x1509;
constexpr Enum invert = 0x150a;
constexpr Enum logic_or_reverse = 0x150b;
constexpr Enum logic_copy_inverted = 0x150c;
constexpr Enum logic_or_inverted = 0x150d;
constexpr Enum logic_nand = 0x150e;
constexpr Enum logic_set = 0x150f;

// Buffers and faces.
constexpr Enum none = 0;
constexpr Enum front_left = 0x0400;
constexpr Enum front_right = 0x0401;
constexpr Enum back_left = 0x0402;
constexpr Enum back_right = 0x0403;
constexpr Enum front = 0x0404;
constexpr Enum back = 0x0405;
constexpr Enum left = 0x0406;
constexpr Enum right = 0x0407;
constexpr Enum front_and_back = 0x0408;
constexpr Enum aux0 = 0x0409;
constexpr Enum cw = 0x0900;
constexpr Enum ccw = 0x0901;

// Capabilities.
constexpr Enum point_smooth = 0x0b10;
constexpr Enum line_smooth = 0x0b20;
constexpr Enum line_stipple = 0x0b24;
constexpr Enum polygon_smooth = 0x0b41;
constexpr Enum polygon_stipple = 0x0b42;
constexpr Enum cull_face = 0x0b44;
constexpr Enum lighting = 0x0b50;
constexpr Enum color_material = 0x0b57;
constexpr Enum fog = 0x0b60;
constexpr Enum depth_test = 0x0b71;
constexpr Enum stencil_test = 0x0b90;
constexpr Enum normalize = 0x0ba1;
constexpr Enum alpha_test = 0x0bc0;
constexpr Enum dither = 0x0bd0;
constexpr Enum blend = 0x0be2;
constexpr Enum index_logic_op = 0x0bf1;
constexpr Enum color_logic_op = 0x0bf2;
constexpr Enum scissor_test = 0x0c11;
constexpr Enum texture_gen_s = 0x0c60;
constexpr Enum texture_gen_t = 0x0c61;
constexpr Enum texture_gen_r = 0x0c62;
constexpr Enum texture_gen_q = 0x0c63;
constexpr Enum auto_normal = 0x0d80;
constexpr Enum map1_color_4 = 0x0d90;
constexpr Enum map1_index = 0x0d91;
constexpr Enum map1_normal = 0x0d92;
constexpr Enum map1_texture_coord_1 = 0x0d93;
constexpr Enum map1_texture_coord_2 = 0x0d94;
constexpr Enum map1_texture_coord_3 = 0x0d95;
constexpr Enum map1_texture_coord_4 = 0x0d96;
constexpr Enum map1_vertex_3 = 0x0d97;
constexpr Enum map1_vertex_4 = 0x0d98;
constexpr Enum map2_color_4 = 0x0db0;
constexpr Enum map2_index = 0x0db1;
constexpr Enum map2_normal = 0x0db2;
constexpr Enum map2_texture_coord_1 = 0x0db3;
constexpr Enum map2_texture_coord_2 = 0x0db4;
constexpr Enum map2_texture_coord_3 = 0x0db5;
constexpr Enum map2_texture_coord_4 = 0x0db6;
constexpr Enum map2_vertex_3 = 0x0db7;
constexpr Enum map2_vertex_4 = 0x0db8;
constexpr Enum texture_1d = 0x0de0;
constexpr Enum texture_2d = 0x0de1;
constexpr Enum polygon_offset_point = 0x2a01;
constexpr Enum polygon_offset_line = 0x2a02;
constexpr Enum polygon_offset_fill = 0x8037;
constexpr Enum clip_plane0 = 0x3000;
constexpr Enum light0 = 0x4000;
constexpr Enum vertex_array = 0x8074;
constexpr Enum normal_array = 0x8075;
constexpr Enum color_array = 0x8076;
constexpr Enum index_array = 0x8077;
constexpr Enum texture_coord_array = 0x8078;
constexpr Enum edge_flag_array = 0x8079;

// Queryable state.
constexpr Enum current_color = 0x0b00;
constexpr Enum current_index = 0x0b01;
constexpr Enum current_normal = 0x0b02;
constexpr Enum current_texture_coords = 0x0b03;
constexpr Enum current_raster_color = 0x0b04;
constexpr Enum current_raster_index = 0x0b05;
constexpr Enum current_raster_texture_coords = 0x0b06;
constexpr Enum current_raster_position = 0x0b07;
constexpr Enum current_raster_position_valid = 0x0b08;
constexpr Enum current_raster_distance = 0x0b09;
constexpr Enum point_size = 0x0b11;
constexpr Enum point_size_range = 0x0b12;
constexpr Enum point_size_granularity = 0x0b13;
constexpr Enum line_width = 0x0b21;
constexpr Enum line_width_range = 0x0b22;
constexpr Enum line_width_granularity = 0x0b23;
constexpr Enum line_stipple_pattern = 0x0b25;
constexpr Enum line_stipple_repeat = 0x0b26;
constexpr Enum list_mode = 0x0b30;
constexpr Enum max_list_nesting = 0x0b31;
constexpr Enum list_base = 0x0b32;
constexpr Enum list_index = 0x0b33;
constexpr Enum polygon_mode = 0x0b40;
constexpr Enum edge_flag = 0x0b43;
constexpr Enum cull_face_mode = 0x0b45;
constexpr Enum front_face = 0x0b46;
constexpr Enum light_model_local_viewer = 0x0b51;
constexpr Enum light_model_two_side = 0x0b52;
constexpr Enum light_model_ambient = 0x0b53;
constexpr Enum shade_model = 0x0b54;
constexpr Enum color_material_face = 0x0b55;
constexpr Enum color_material_parameter = 0x0b56;
constexpr Enum fog_index = 0x0b61;
constexpr Enum fog_density = 0x0b62;
constexpr Enum fog_start = 0x0b63;
constexpr Enum fog_end = 0x0b64;
constexpr Enum fog_mode = 0x0b65;
constexpr Enum fog_color = 0x0b66;
constexpr Enum depth_range = 0x0b70;
constexpr Enum depth_writemask = 0x0b72;
constexpr Enum depth_clear_value = 0x0b73;
constexpr Enum depth_func = 0x0b74;
constexpr Enum accum_clear_value = 0x0b80;
constexpr Enum stencil_clear_value = 0x0b91;
constexpr Enum stencil_func = 0x0b92;
constexpr Enum stencil_value_mask = 0x0b93;
constexpr Enum stencil_fail = 0x0b94;
constexpr Enum stencil_pass_depth_fail = 0x0b95;
constexpr Enum stencil_pass_depth_pass = 0x0b96;
constexpr Enum stencil_ref = 0x0b97;
constexpr Enum stencil_writemask = 0x0b98;
constexpr Enum matrix_mode = 0x0ba0;
constexpr Enum viewport = 0x0ba2;
constexpr Enum modelview_stack_depth = 0x0ba3;
constexpr Enum projection_stack_depth = 0x0ba4;
constexpr Enum texture_stack_depth = 0x0ba5;
constexpr Enum modelview_matrix = 0x0ba6;
constexpr Enum projection_matrix = 0x0ba7;
constexpr Enum texture_matrix = 0x0ba8;
constexpr Enum attrib_stack_depth = 0x0bb0;
constexpr Enum client_attrib_stack_depth = 0x0bb1;
constexpr Enum alpha_test_func = 0x0bc1;
constexpr Enum alpha_test_ref = 0x0bc2;
constexpr Enum blend_dst = 0x0be0;
constexpr Enum blend_src = 0x0be1;
constexpr Enum logic_op_mode = 0x0bf0;
constexpr Enum aux_buffers = 0x0c00;
constexpr Enum draw_buffer = 0x0c01;
constexpr Enum read_buffer = 0x0c02;
constexpr Enum scissor_box = 0x0c10;
constexpr Enum index_clear_value = 0x0c20;
constexpr Enum index_writemask = 0x0c21;
constexpr Enum color_clear_value = 0x0c22;
constexpr Enum color_writemask = 0x0c23;
constexpr Enum index_mode = 0x0c30;
constexpr Enum rgba_mode = 0x0c31;
constexpr Enum doublebuffer = 0x0c32;
constexpr Enum stereo = 0x0c33;
constexpr Enum render_mode = 0x0c40;
constexpr Enum perspective_correction_hint = 0x0c50;
constexpr Enum point_smooth_hint = 0x0c51;
constexpr Enum line_smooth_hint = 0x0c52;
constexpr Enum polygon_smooth_hint = 0x0c53;
constexpr Enum fog_hint = 0x0c54;
constexpr Enum pixel_map_i_to_i = 0x0c70;
constexpr Enum pixel_map_s_to_s = 0x0c71;
constexpr Enum pixel_map_i_to_r = 0x0c72;
constexpr Enum pixel_map_i_to_a = 0x0c75;
constexpr Enum pixel_map_r_to_r = 0x0c76;
constexpr Enum pixel_map_a_to_a = 0x0c79;
constexpr Enum pixel_map_i_to_i_size = 0x0cb0;
constexpr Enum pixel_map_a_to_a_size = 0x0cb9;
constexpr Enum unpack_swap_bytes = 0x0cf0;
constexpr Enum unpack_lsb_first = 0x0cf1;
constexpr Enum unpack_row_length = 0x0cf2;
constexpr Enum unpack_skip_rows = 0x0cf3;
constexpr Enum unpack_skip_pixels = 0x0cf4;
constexpr Enum unpack_alignment = 0x0cf5;
constexpr Enum pack_swap_bytes = 0x0d00;
constexpr Enum pack_lsb_first = 0x0d01;
constexpr Enum pack_row_length = 0x0d02;
constexpr Enum pack_skip_rows = 0x0d03;
constexpr Enum pack_skip_pixels = 0x0d04;
constexpr Enum pack_alignment = 0x0d05;
constexpr Enum map_color = 0x0d10;
constexpr Enum map_stencil = 0x0d11;
constexpr Enum index_shift = 0x0d12;
constexpr Enum index_offset = 0x0d13;
constexpr Enum red_scale = 0x0d14;
constexpr Enum red_bias = 0x0d15;
constexpr Enum zoom_x = 0x0d16;
constexpr Enum zoom_y = 0x0d17;
constexpr Enum green_scale = 0x0d18;
constexpr Enum green_bias = 0x0d19;
constexpr Enum blue_scale = 0x0d1a;
constexpr Enum blue_bias = 0x0d1b;
constexpr Enum alpha_scale = 0x0d1c;
constexpr Enum alpha_bias = 0x0d1d;
constexpr Enum depth_scale = 0x0d1e;
constexpr Enum depth_bias = 0x0d1f;
constexpr Enum max_eval_order = 0x0d30;
constexpr Enum max_lights = 0x0d31;
constexpr Enum max_clip_planes = 0x0d32;
constexpr Enum max_texture_size = 0x0d33;
constexpr Enum max_pixel_map_table = 0x0d34;
constexpr Enum max_attrib_stack_depth = 0x0d35;
constexpr Enum max_modelview_stack_depth = 0x0d36;
constexpr Enum max_name_stack_depth = 0x0d37;
constexpr Enum max_projection_stack_depth = 0x0d38;
constexpr Enum max_texture_stack_depth = 0x0d39;
constexpr Enum max_viewport_dims = 0x0d3a;
constexpr Enum max_client_attrib_stack_depth = 0x0d3b;
constexpr Enum subpixel_bits = 0x0d50;
constexpr Enum index_bits = 0x0d51;
constexpr Enum red_bits = 0x0d52;
constexpr Enum green_bits = 0x0d53;
constexpr Enum blue_bits = 0x0d54;
constexpr Enum alpha_bits = 0x0d55;
constexpr Enum depth_bits = 0x0d56;
constexpr Enum stencil_bits = 0x0d57;
constexpr Enum accum_red_bits = 0x0d58;
constexpr Enum accum_green_bits = 0x0d59;
constexpr Enum accum_blue_bits = 0x0d5a;
constexpr Enum accum_alpha_bits = 0x0d5b;
constexpr Enum name_stack_depth = 0x0d70;
constexpr Enum map1_grid_domain = 0x0dd0;
constexpr Enum map1_grid_segments = 0x0dd1;
constexpr Enum map2_grid_domain = 0x0dd2;
constexpr Enum map2_grid_segments = 0x0dd3;
constexpr Enum feedback_buffer_pointer = 0x0df0;
constexpr Enum feedback_buffer_size = 0x0df1;
constexpr Enum feedback_buffer_type = 0x0df2;
constexpr Enum selection_buffer_pointer = 0x0df3;
constexpr Enum selection_buffer_size = 0x0df4;
constexpr Enum polygon_offset_units = 0x2a00;
constexpr Enum polygon_offset_factor = 0x8038;
constexpr Enum texture_binding_1d = 0x8068;
constexpr Enum texture_binding_2d = 0x8069;
constexpr Enum vertex_array_size = 0x807a;
constexpr Enum vertex_array_type = 0x807b;
constexpr Enum vertex_array_stride = 0x807c;
constexpr Enum normal_array_type = 0x807e;
constexpr Enum normal_array_stride = 0x807f;
constexpr Enum color_array_size = 0x8081;
constexpr Enum color_array_type = 0x8082;
constexpr Enum color_array_stride = 0x8083;
constexpr Enum index_array_type = 0x8085;
constexpr Enum index_array_stride = 0x8086;
constexpr Enum texture_coord_array_size = 0x8088;
constexpr Enum texture_coord_array_type = 0x8089;
constexpr Enum texture_coord_array_stride = 0x808a;
constexpr Enum edge_flag_array_stride = 0x808c;
constexpr Enum vertex_array_pointer = 0x808e;
constexpr Enum normal_array_pointer = 0x808f;
constexpr Enum color_array_pointer = 0x8090;
constexpr Enum index_array_pointer = 0x8091;
constexpr Enum texture_coord_array_pointer = 0x8092;
constexpr Enum edge_flag_array_pointer = 0x8093;

// Hints.
constexpr Enum dont_care = 0x1100;
constexpr Enum fastest = 0x1101;
constexpr Enum nicest = 0x1102;

// Light and material parameters.
constexpr Enum ambient = 0x1200;
constexpr Enum diffuse = 0x1201;
constexpr Enum specular = 0x1202;
constexpr Enum position = 0x1203;
constexpr Enum spot_direction = 0x1204;
constexpr Enum spot_exponent = 0x1205;
constexpr Enum spot_cutoff = 0x1206;
constexpr Enum constant_attenuation = 0x1207;
constexpr Enum linear_attenuation = 0x1208;
constexpr Enum quadratic_attenuation = 0x1209;
constexpr Enum emission = 0x1600;
constexpr Enum shininess = 0x1601;
constexpr Enum ambient_and_diffuse = 0x1602;
constexpr Enum color_indexes = 0x1603;

// Display lists, matrices, copy types and pixel formats.
constexpr Enum compile = 0x1300;
constexpr Enum compile_and_execute = 0x1301;
constexpr Enum modelview = 0x1700;
constexpr Enum projection = 0x1701;
constexpr Enum texture = 0x1702;
constexpr Enum color = 0x1800;
constexpr Enum depth = 0x1801;
constexpr Enum stencil = 0x1802;
constexpr Enum color_index = 0x1900;
constexpr Enum stencil_index = 0x1901;
constexpr Enum depth_component = 0x1902;
constexpr Enum red = 0x1903;
constexpr Enum green = 0x1904;
constexpr Enum blue = 0x1905;
constexpr Enum alpha = 0x1906;
constexpr Enum rgb = 0x1907;
constexpr Enum rgba = 0x1908;
constexpr Enum luminance = 0x1909;
constexpr Enum luminance_alpha = 0x190a;
constexpr Enum bgr_ext = 0x80e0;
constexpr Enum bgra_ext = 0x80e1;
constexpr Enum alpha4 = 0x803b;
constexpr Enum alpha8 = 0x803c;
constexpr Enum alpha12 = 0x803d;
constexpr Enum alpha16 = 0x803e;
constexpr Enum luminance4 = 0x803f;
constexpr Enum luminance16 = 0x8042;
constexpr Enum luminance4_alpha4 = 0x8043;
constexpr Enum luminance16_alpha16 = 0x8048;
constexpr Enum intensity = 0x8049;
constexpr Enum intensity4 = 0x804a;
constexpr Enum intensity16 = 0x804d;
constexpr Enum r3_g3_b2 = 0x2a10;
constexpr Enum rgb4 = 0x804f;
constexpr Enum rgb16 = 0x8054;
constexpr Enum rgba2 = 0x8055;
constexpr Enum rgba16 = 0x805b;

// Rasterization, render mode, shading and strings.
constexpr Enum point = 0x1b00;
constexpr Enum line = 0x1b01;
constexpr Enum fill = 0x1b02;
constexpr Enum render = 0x1c00;
constexpr Enum feedback = 0x1c01;
constexpr Enum select = 0x1c02;
constexpr Enum flat = 0x1d00;
constexpr Enum smooth = 0x1d01;
constexpr Enum vendor = 0x1f00;
constexpr Enum renderer = 0x1f01;
constexpr Enum version = 0x1f02;
constexpr Enum extensions = 0x1f03;

// Feedback.
constexpr Enum feedback_2d = 0x0600;
constexpr Enum feedback_3d = 0x0601;
constexpr Enum feedback_3d_color = 0x0602;
constexpr Enum feedback_3d_color_texture = 0x0603;
constexpr Enum feedback_4d_color_texture = 0x0604;
constexpr Enum pass_through_token = 0x0700;
constexpr Enum point_token = 0x0701;
constexpr Enum line_token = 0x0702;
constexpr Enum polygon_token = 0x0703;
constexpr Enum bitmap_token = 0x0704;
constexpr Enum draw_pixel_token = 0x0705;
constexpr Enum copy_pixel_token = 0x0706;
constexpr Enum line_reset_token = 0x0707;

// Evaluator queries and fog modes.
constexpr Enum coeff = 0x0a00;
constexpr Enum order = 0x0a01;
constexpr Enum domain = 0x0a02;
constexpr Enum exp = 0x0800;
constexpr Enum exp2 = 0x0801;

// Accumulation.
constexpr Enum accum = 0x0100;
constexpr Enum load = 0x0101;
constexpr Enum return_value = 0x0102;
constexpr Enum mult = 0x0103;
constexpr Enum add = 0x0104;

// Texturing.
constexpr Enum modulate = 0x2100;
constexpr Enum decal = 0x2101;
constexpr Enum texture_env_mode = 0x2200;
constexpr Enum texture_env_color = 0x2201;
constexpr Enum texture_env = 0x2300;
constexpr Enum eye_linear = 0x2400;
constexpr Enum object_linear = 0x2401;
constexpr Enum sphere_map = 0x2402;
constexpr Enum texture_gen_mode = 0x2500;
constexpr Enum object_plane = 0x2501;
constexpr Enum eye_plane = 0x2502;
constexpr Enum coord_s = 0x2000;
constexpr Enum coord_t = 0x2001;
constexpr Enum coord_r = 0x2002;
constexpr Enum coord_q = 0x2003;
constexpr Enum nearest = 0x2600;
constexpr Enum linear = 0x2601;
constexpr Enum nearest_mipmap_nearest = 0x2700;
constexpr Enum linear_mipmap_nearest = 0x2701;
constexpr Enum nearest_mipmap_linear = 0x2702;
constexpr Enum linear_mipmap_linear = 0x2703;
constexpr Enum texture_mag_filter = 0x2800;
constexpr Enum texture_min_filter = 0x2801;
constexpr Enum texture_wrap_s = 0x2802;
constexpr Enum texture_wrap_t = 0x2803;
constexpr Enum clamp = 0x2900;
constexpr Enum repeat = 0x2901;
constexpr Enum clamp_to_edge = 0x812f;
constexpr Enum texture_width = 0x1000;
constexpr Enum texture_height = 0x1001;
constexpr Enum texture_internal_format = 0x1003;
constexpr Enum texture_border_color = 0x1004;
constexpr Enum texture_border = 0x1005;
constexpr Enum texture_red_size = 0x805c;
constexpr Enum texture_green_size = 0x805d;
constexpr Enum texture_blue_size = 0x805e;
constexpr Enum texture_alpha_size = 0x805f;
constexpr Enum texture_luminance_size = 0x8060;
constexpr Enum texture_intensity_size = 0x8061;
constexpr Enum proxy_texture_1d = 0x8063;
constexpr Enum proxy_texture_2d = 0x8064;
constexpr Enum texture_priority = 0x8066;
constexpr Enum texture_resident = 0x8067;

// Attribute groups.
constexpr Bitfield current_bit = 0x00000001;
constexpr Bitfield point_bit = 0x00000002;
constexpr Bitfield line_bit = 0x00000004;
constexpr Bitfield polygon_bit = 0x00000008;
constexpr Bitfield polygon_stipple_bit = 0x00000010;
constexpr Bitfield pixel_mode_bit = 0x00000020;
constexpr Bitfield lighting_bit = 0x00000040;
constexpr Bitfield fog_bit = 0x00000080;
constexpr Bitfield depth_buffer_bit = 0x00000100;
constexpr Bitfield accum_buffer_bit = 0x00000200;
constexpr Bitfield stencil_buffer_bit = 0x00000400;
constexpr Bitfield viewport_bit = 0x00000800;
constexpr Bitfield transform_bit = 0x00001000;
constexpr Bitfield enable_bit = 0x00002000;
constexpr Bitfield color_buffer_bit = 0x00004000;
constexpr Bitfield hint_bit = 0x00008000;
constexpr Bitfield eval_bit = 0x00010000;
constexpr Bitfield list_bit = 0x00020000;
constexpr Bitfield texture_bit = 0x00040000;
constexpr Bitfield scissor_bit = 0x00080000;
constexpr Bitfield all_attrib_bits = 0x000fffff;
constexpr Bitfield client_pixel_store_bit = 0x00000001;
constexpr Bitfield client_vertex_array_bit = 0x00000002;

// Interleaved array formats.
constexpr Enum v2f = 0x2a20;
constexpr Enum v3f = 0x2a21;
constexpr Enum c4ub_v2f = 0x2a22;
constexpr Enum c4ub_v3f = 0x2a23;
constexpr Enum c3f_v3f = 0x2a24;
constexpr Enum n3f_v3f = 0x2a25;
constexpr Enum c4f_n3f_v3f = 0x2a26;
constexpr Enum t2f_v3f = 0x2a27;
constexpr Enum t4f_v4f = 0x2a28;
constexpr Enum t2f_c4ub_v3f = 0x2a29;
constexpr Enum t2f_c3f_v3f = 0x2a2a;
constexpr Enum t2f_n3f_v3f = 0x2a2b;
constexpr Enum t2f_c4f_n3f_v3f = 0x2a2c;
constexpr Enum t4f_c4f_n3f_v4f = 0x2a2d;

} // namespace rx7900::opengl::gl
