#pragma once

#include <cstddef>
#include <cstdint>

#include "i2c.hpp"

// Minimal Si5351A driver for the VFO.
//
// Strategy: use PLLA with a fixed VCO of 900 MHz (feedback divider = 900/25 = 36,
// integer) and program MultiSynth <clk> as VCO / Fout (fractional when needed).
// Covers roughly 1 MHz .. 150 MHz on a single output, which is plenty for an HF VFO.
class Si5351 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x60;
    static constexpr uint32_t DEFAULT_XTAL_HZ = 25000000;

    Si5351(espp::I2c &i2c, uint8_t address = DEFAULT_ADDRESS)
        : i2c_(i2c), address_(address) {}

    // Basic power-up: disable outputs, power down all clock drivers, select the
    // crystal as PLL source, disable spread spectrum, set crystal load, reset PLLs.
    bool init() {
        if (!write_reg(3, 0xFF)) {                 // power down all outputs (OEB)
            return false;
        }
        for (uint8_t r = 16; r <= 23; ++r) {       // power down every CLKx driver
            if (!write_reg(r, 0x80)) {
                return false;
            }
        }
        if (!write_reg(15, 0x00)) {                // PLLA/PLLB source = XTAL
            return false;
        }
        if (!write_reg(149, 0x00)) {               // spread spectrum off
            return false;
        }
        if (!write_reg(183, 0xD2)) {               // crystal load ~10 pF
            return false;
        }
        if (!write_reg(177, 0xA0)) {               // soft reset both PLLs
            return false;
        }
        return true;
    }

    // Set the output frequency (Hz) on the given clock (0..2).
    bool set_frequency(uint32_t freq_hz, uint8_t clk = 0) {
        if (freq_hz < 1000000UL || freq_hz > 150000000UL || clk > 2) {
            return false;
        }

        // Fixed VCO = 900 MHz (PLLA feedback = 36, integer).
        const uint32_t vco_hz = 900000000UL;
        const uint32_t pll_a = vco_hz / DEFAULT_XTAL_HZ; // 36
        if (!write_pll(26, pll_a, 0, 1)) {               // PLLA regs 26..33
            return false;
        }

        // MultiSynth divider = VCO / Fout = a + b/c
        uint32_t a = vco_hz / freq_hz;
        uint32_t rem = vco_hz % freq_hz;
        uint32_t b = 0;
        uint32_t c = 1;
        if (rem != 0) {
            // 20-bit fractional denominator for good resolution (~0.1 Hz step here).
            c = 1048575u;
            b = static_cast<uint32_t>((static_cast<uint64_t>(rem) * c + freq_hz / 2) / freq_hz);
            if (b >= c) {
                a += b / c;
                b %= c;
            }
        }
        const uint8_t ms_base = static_cast<uint8_t>(42 + clk * 8);
        if (!write_ms(ms_base, a, b, c)) {
            return false;
        }

        // CLKx control: source = MSx from PLLA, 2 mA drive; integer mode if b == 0
        // (0x40 = MSx_INT). Bits: [3:2]=11 (MSx), [5]=0 (PLLA).
        uint8_t ctrl = 0x0C | ((b == 0) ? 0x40 : 0x00);
        if (!write_reg(static_cast<uint8_t>(16 + clk), ctrl)) {
            return false;
        }

        // Reset both PLLs so PLLA locks to the new configuration, then enable output.
        if (!write_reg(177, 0xA0)) {
            return false;
        }
        if (!write_reg(3, static_cast<uint8_t>(0xFF & ~(1u << clk)))) {
            return false;
        }

        freq_hz_ = freq_hz;
        return true;
    }

    uint32_t frequency() const { return freq_hz_; }

private:
    bool write_reg(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        return i2c_.write(address_, data, 2);
    }

    // Assemble MultiSynth registers from divider a + b/c and write the 8-byte block.
    bool write_pll(uint8_t base, uint32_t a, uint32_t b, uint32_t c) {
        uint32_t p1 = 128u * a + (128u * b) / c - 512u;
        uint32_t p2 = 128u * b - c * ((128u * b) / c);
        uint32_t p3 = c;
        uint8_t regs[8] = {
            static_cast<uint8_t>((p3 >> 8) & 0xFF),
            static_cast<uint8_t>(p3 & 0xFF),
            static_cast<uint8_t>((p1 >> 16) & 0x03),
            static_cast<uint8_t>((p1 >> 8) & 0xFF),
            static_cast<uint8_t>(p1 & 0xFF),
            static_cast<uint8_t>(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)),
            static_cast<uint8_t>((p2 >> 8) & 0xFF),
            static_cast<uint8_t>(p2 & 0xFF),
        };
        return write_block(base, regs, 8);
    }

    // Same equations, but reg[2] holds R<n>_DIV[2:0] | P1[17:16] (rDiv = 0 here).
    bool write_ms(uint8_t base, uint32_t a, uint32_t b, uint32_t c) {
        uint32_t p1 = 128u * a + (128u * b) / c - 512u;
        uint32_t p2 = 128u * b - c * ((128u * b) / c);
        uint32_t p3 = c;
        uint8_t regs[8] = {
            static_cast<uint8_t>((p3 >> 8) & 0xFF),
            static_cast<uint8_t>(p3 & 0xFF),
            static_cast<uint8_t>((p1 >> 16) & 0x03),
            static_cast<uint8_t>((p1 >> 8) & 0xFF),
            static_cast<uint8_t>(p1 & 0xFF),
            static_cast<uint8_t>(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)),
            static_cast<uint8_t>((p2 >> 8) & 0xFF),
            static_cast<uint8_t>(p2 & 0xFF),
        };
        return write_block(base, regs, 8);
    }

    bool write_block(uint8_t base, const uint8_t *data, size_t len) {
        uint8_t buf[9];
        buf[0] = base;
        for (size_t i = 0; i < len; ++i) {
            buf[i + 1] = data[i];
        }
        return i2c_.write(address_, buf, len + 1);
    }

    espp::I2c &i2c_;
    uint8_t address_;
    uint32_t freq_hz_ = 0;
};
