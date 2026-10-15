#pragma once

#include <cstddef>
#include <cstdint>

#include "i2c.hpp"

// Minimal Si5351A driver for the VFO.
//
//   CLK0 : local oscillator (LO)    - PLLA, fixed 900 MHz VCO + fractional MS.
//   CLK1 : carrier / BFO "I"        }  PLLB, integer MultiSynth, CLK2 shifted
//   CLK2 : carrier / BFO "Q"        }  90 degrees via the phase-offset register.
//
// The quadrature carrier uses the AN619 phase-offset trick: with an integer
// MultiSynth divider N (9..127) shared by two outputs on the same PLL, writing N
// into one output's phase-offset register yields an exact 90 degree shift. This
// limits the carrier to roughly 4.8 .. 100 MHz.
class Si5351 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x60;
    static constexpr uint32_t DEFAULT_XTAL_HZ = 25000000;

    Si5351(espp::I2c &i2c, uint8_t address = DEFAULT_ADDRESS)
        : i2c_(i2c), address_(address) {}

    // Basic power-up: disable outputs, power down all clock drivers, select the
    // crystal as PLL source, disable spread spectrum, set crystal load, reset PLLs.
    bool init() {
        oeb_ = 0xFF;
        if (!write_reg(3, oeb_)) {                 // disable all outputs (OEB)
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

    // Local oscillator (Hz) on the given clock (0..2, default CLK0). The VCO is
    // kept in 600..900 MHz: outputs up to ~112.5 MHz use a 900 MHz VCO with a
    // fractional MultiSynth divider, higher outputs an integer divider
    // (VCO = N * Fout), which is required once the fractional divider would
    // fall below its minimum of 8 (and lets us reach the VHF band centres).
    bool set_frequency(uint32_t freq_hz, uint8_t clk = 0) {
        if (freq_hz < 1000000UL || freq_hz > 200000000UL || clk > 2) {
            return false;
        }
        uint32_t vco_hz = 900000000UL;
        uint32_t msa = 0, msb = 0, msc = 0;
        if (vco_hz / freq_hz >= 8) {
            divider(vco_hz, freq_hz, msa, msb, msc);   // fractional MultiSynth
        } else {
            uint32_t n = vco_hz / freq_hz;             // integer MultiSynth
            if (n < 4) {
                n = 4;
            }
            vco_hz = n * freq_hz;
            msa = n;
            msb = 0;
            msc = 1;
        }
        uint32_t pa = 0, pb = 0, pc = 0;
        divider(DEFAULT_XTAL_HZ, vco_hz, pa, pb, pc);  // PLLA feedback
        if (!write_pll(26, pa, pb, pc)) {              // PLLA regs 26..33
            return false;
        }
        const uint8_t ms_base = static_cast<uint8_t>(42 + clk * 8);
        if (!write_ms(ms_base, msa, msb, msc)) {
            return false;
        }
        // CLKx control: source = MSx from PLLA, 2 mA drive; integer mode if b == 0.
        uint8_t ctrl = 0x0C | ((msb == 0) ? 0x40 : 0x00);
        if (!write_reg(static_cast<uint8_t>(16 + clk), ctrl)) {
            return false;
        }
        if (!write_reg(177, 0x20)) {                   // reset PLLA so it locks
            return false;
        }
        if (!enable_clock(clk)) {
            return false;
        }
        freq_hz_ = freq_hz;
        return true;
    }

    // Carrier / BFO on CLK1 (I) and CLK2 (Q), 90 degrees apart. enable = false
    // powers the two outputs back down.
    bool set_carrier(uint32_t freq_hz, bool enable) {
        if (!enable) {
            if (!write_reg(17, 0x80) || !write_reg(18, 0x80)) {
                return false;
            }
            return disable_clock(1) && disable_clock(2);
        }
        if (freq_hz < 4800000UL || freq_hz > 100000000UL) {
            return false;
        }
        // Integer divider N in [9,127] so that VCO = N * f lands in 600..900 MHz,
        // and the output phase-offset register (max 127) can express 90 degrees.
        uint32_t n = 900000000UL / freq_hz;
        if (n < 9) {
            n = 9;
        }
        if (n > 127) {
            n = 127;
        }
        const uint32_t vco_hz = n * freq_hz;
        uint32_t a = 0, b = 0, c = 0;
        divider(DEFAULT_XTAL_HZ, vco_hz, a, b, c);     // PLLB feedback
        if (!write_pll(34, a, b, c)) {                 // PLLB regs 34..41
            return false;
        }
        if (!write_ms(50, n, 0, 1)) {                  // MS1 (I), integer divider
            return false;
        }
        if (!write_ms(58, n, 0, 1)) {                  // MS2 (Q), integer divider
            return false;
        }
        if (!write_reg(166, 0)) {                      // CLK1 phase offset = 0
            return false;
        }
        if (!write_reg(167, static_cast<uint8_t>(n & 0x7F))) { // CLK2 = 90 deg
            return false;
        }
        // CLK1/CLK2: PLLB (bit5) + MSx select (0x0C) + 8 mA (0x03), fractional
        // mode (MSx_INT = 0, required for the phase offset to take effect).
        if (!write_reg(17, 0x2F) || !write_reg(18, 0x2F)) {
            return false;
        }
        if (!write_reg(177, 0x80)) {                   // reset PLLB to latch phase
            return false;
        }
        if (!enable_clock(1) || !enable_clock(2)) {
            return false;
        }
        return true;
    }

    uint32_t frequency() const { return freq_hz_; }

    // True when the given output (CLK0..CLK2) is enabled in the OEB shadow.
    bool clock_enabled(uint8_t clk) const {
        return clk < 8 && !(oeb_ & (1u << clk));
    }

private:
    bool write_reg(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        return i2c_.write(address_, data, 2);
    }

    // Output enable register (3) shadow: a set bit *disables* the matching clock.
    bool enable_clock(uint8_t clk) {
        oeb_ = static_cast<uint8_t>(oeb_ & ~(1u << clk));
        return write_reg(3, oeb_);
    }
    bool disable_clock(uint8_t clk) {
        oeb_ = static_cast<uint8_t>(oeb_ | (1u << clk));
        return write_reg(3, oeb_);
    }

    // Decompose ref / out into a MultiSynth divider a + b/c (c = 1 when exact).
    static void divider(uint32_t ref_hz, uint32_t out_hz, uint32_t &a, uint32_t &b,
                        uint32_t &c) {
        a = ref_hz / out_hz;
        uint32_t rem = ref_hz % out_hz;
        b = 0;
        c = 1;
        if (rem != 0) {
            c = 1048575u; // 20-bit denominator for ~sub-Hz resolution here
            b = static_cast<uint32_t>((static_cast<uint64_t>(rem) * c + out_hz / 2) /
                                      out_hz);
            if (b >= c) {
                a += b / c;
                b %= c;
            }
        }
    }

    // Assemble a MultiSynth/PLL divider (a + b/c) and write the 8-byte register block.
    bool write_pll(uint8_t base, uint32_t a, uint32_t b, uint32_t c) {
        return write_div(base, a, b, c, 0);
    }
    bool write_ms(uint8_t base, uint32_t a, uint32_t b, uint32_t c) {
        return write_div(base, a, b, c, 0);
    }

    // regs[2] holds R<n>_DIV[2:0] | P1[17:16] (rDiv = 0 here).
    bool write_div(uint8_t base, uint32_t a, uint32_t b, uint32_t c, uint8_t rdiv) {
        uint32_t p1 = 128u * a + (128u * b) / c - 512u;
        uint32_t p2 = 128u * b - c * ((128u * b) / c);
        uint32_t p3 = c;
        uint8_t regs[8] = {
            static_cast<uint8_t>((p3 >> 8) & 0xFF),
            static_cast<uint8_t>(p3 & 0xFF),
            static_cast<uint8_t>((rdiv & 0x07) | ((p1 >> 16) & 0x03)),
            static_cast<uint8_t>((p1 >> 8) & 0xFF),
            static_cast<uint8_t>(p1 & 0xFF),
            static_cast<uint8_t>(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)),
            static_cast<uint8_t>((p2 >> 8) & 0xFF),
            static_cast<uint8_t>(p2 & 0xFF),
        };
        uint8_t buf[9];
        buf[0] = base;
        for (size_t i = 0; i < 8; ++i) {
            buf[i + 1] = regs[i];
        }
        return i2c_.write(address_, buf, 9);
    }

    espp::I2c &i2c_;
    uint8_t address_;
    uint8_t oeb_ = 0xFF;
    uint32_t freq_hz_ = 0;
};
