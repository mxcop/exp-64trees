#pragma once

#include <cmath>
#include <cstdint>

/* Voxel Block Compression 4x4x4. */
struct Vbc64 {
    /* Number of bits per voxel used for interpolation between the endpoints. */
    static constexpr uint32_t LERP_BITS = 3u;
    /* Number of interpolation steps per voxel between the endpoints. */
    static constexpr uint32_t LERP_STEPS = 1u << LERP_BITS;
    static constexpr uint32_t LERP_STEPS_MASK = LERP_STEPS - 1u;
    /* Number of words stored per block. */
    static constexpr uint32_t NUM_WORDS = (64u * LERP_BITS + 31u) / 32u;

    /* Endpoints */
    uint8_t min_value{}, max_value{};

    Vbc64() = default;
    Vbc64(float min_value, float max_value) : min_value(min_value), max_value(max_value) {};
};

/* Write interpolation point. */
inline void write_point(uint32_t* dwords, uint32_t i, uint32_t point) {
    const uint32_t bit_pos = i * Vbc64::LERP_BITS;
    const uint32_t word = bit_pos >> 5;
    const uint32_t bit_offset = bit_pos & 0b11111u;
    dwords[word] |= point << bit_offset;
    if (bit_offset + Vbc64::LERP_BITS > 32u) {
        dwords[word + 1u] |= point >> (32u - bit_offset);
    }
}

/* Read interpolation point. */
inline uint32_t read_point(uint32_t* dwords, uint32_t i) {
    const uint32_t bit_pos = i * Vbc64::LERP_BITS;
    const uint32_t word = bit_pos >> 5;
    const uint32_t bit_offset = bit_pos & 0b11111u;
    uint32_t point = (dwords[word] >> bit_offset) & Vbc64::LERP_STEPS_MASK;
    if (bit_offset + Vbc64::LERP_BITS > 32u) {
        point |= (dwords[word + 1u] << (32u - bit_offset)) & Vbc64::LERP_STEPS_MASK;
    }
    return point;
}

/* Compresses 64 raw floating point values into a 4x4x4 block compressed voxel grid. */
inline Vbc64 compress_vbc64(const uint8_t* data, uint32_t* output, uint64_t mask) {
    /* Find the lower and upper endpoint */
    float min_value = 1e30f, max_value = -1e30f;
    for (uint64_t m = mask; m != 0u; m &= m - 1u) {
        const uint32_t i = _tzcnt_u64(m);
        min_value = fminf(min_value, data[i]);
        max_value = fmaxf(max_value, data[i]);
    }
    const float rcp_range = 1.0f / (float)(max_value - min_value);
    const float scale = Vbc64::LERP_STEPS_MASK * rcp_range;

    /* Find the best interpolation bits for each voxel */
    Vbc64 vbc(min_value, max_value);
    uint32_t j = 0u;
    for (uint64_t m = mask; m != 0u; m &= m - 1u) {
        const uint32_t i = _tzcnt_u64(m);

        /* Find the optimal interpolation point */
        const uint32_t point = (uint32_t)((float)(data[i] - min_value) * scale + 0.5f);

        /* Write the interpolation point */
        write_point(output, j, point);
        j++;
    }
    return vbc;
}
 
/* Reads a voxel from a block compressed voxel grid. */
inline uint8_t read_vbc64(const Vbc64& vbc, uint32_t* dwords, const uint32_t i) {
    /* Bounds check */
    if (i >= 64u) return 0.0f;
    const float range = (float)(vbc.max_value - vbc.min_value);

    /* Read the interpolation point */
    const uint32_t point = read_point(dwords, i);
    const float u = (float)point * (1.0f / Vbc64::LERP_STEPS_MASK);

    /* Interpolate */
    return (uint8_t)((float)vbc.min_value + u * range);
}
