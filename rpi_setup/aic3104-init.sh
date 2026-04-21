#!/bin/bash

sleep 2

# ----------------------
# PAGE 0
# ----------------------
i2cset -y 1 0x18 0x00 0x00   # Reg 0 = 00000000

# ----------------------
# PLL CONFIG (12 MHz → 48 kHz)
# ----------------------
# Reg 3: PLL ON, Q=2, P=1
i2cset -y 1 0x18 0x03 0x91   # 10010001

# Reg 4: J = 8
i2cset -y 1 0x18 0x04 0x20   # 00100000

# Reg 5–6: D = 1920 (0x0780)
i2cset -y 1 0x18 0x05 0x07   # 00000111
i2cset -y 1 0x18 0x06 0x80   # 10000000

# Reg 11: R = 1
i2cset -y 1 0x18 0x0B 0x01   # 00000001

sleep 0.1

# ----------------------
# SAMPLE RATE = fS(ref)
# ----------------------
# ADC = DAC = fS(ref)
i2cset -y 1 0x18 0x02 0x00   # Reg 2 = 00000000

# ----------------------
# ENABLE DAC DATA PATH (CRITICAL)
# ----------------------
# Left → left, Right → right
#i2cset -y 1 0x18 0x07 0x0A   # Reg 7 = 00001010

# ----------------------
# AUDIO INTERFACE (I2S, 16-bit, slave)
# ----------------------
#i2cset -y 1 0x18 0x08 0x00   # Reg 8 = 00000000
#i2cset -y 1 0x18 0x09 0x00   # Reg 9 = 00000000

# ----------------------
# POWER UP DAC
# ----------------------
#i2cset -y 1 0x18 0x25 0xC0   # Reg 37 = 11000000

# ----------------------
# UNMUTE DIGITAL VOLUME
# ----------------------
#i2cset -y 1 0x18 0x2B 0x00   # Reg 43 = 00000000
#i2cset -y 1 0x18 0x2C 0x00   # Reg 44 = 00000000

# ----------------------
# ROUTE DAC → LINE OUTPUT PATH
# ----------------------
# Select DAC_L3 / DAC_R3 path
#i2cset -y 1 0x18 0x29 0x50   # Reg 41 = 01010000

# ----------------------
# ANALOG ROUTING TO LINE OUT
# ----------------------
# Left DAC → LEFT_LOP/M
#i2cset -y 1 0x18 0x52 0x80   # Reg 82 = 10000000

# Right DAC → RIGHT_LOP/M
#i2cset -y 1 0x18 0x5C 0x80   # Reg 92 = 10000000

# ----------------------
# POWER UP LINE OUTPUTS
# ----------------------
# LEFT_LOP/M: unmute + power
#i2cset -y 1 0x18 0x56 0x99   # Reg 86 = 00001001

# RIGHT_LOP/M: unmute + power
#i2cset -y 1 0x18 0x5D 0x99   # Reg 93 = 00001001

# ----------------------
# POP SUPPRESSION
# ----------------------
#i2cset -y 1 0x18 0x2A 0x02   # Reg 42 = 00000010

echo "TLV320AIC3104 initialized (PLL + DAC + LINE OUT OK)"


# MIC2L disconnected, MIC2R to right ADC, max gain
#i2cset -y 1 0x18 0x12 0xF0

# Left ADC power down
#i2cset -y 1 0x18 0x13 0x78

# Right ADC power up
#i2cset -y 1 0x18 0x16 0x7C

# Unmute left PGA, mid gain
#i2cset -y 1 0x18 0x0F 0x0F

# MIC1R to right ADC, MIC1L to left ADC, max gain
#i2cset -y 1 0x18 0x13 0x04
#i2cset -y 1 0x18 0x16 0x04

# Unmute right PGA, max gain
#i2cset -y 1 0x18 0x10 0x7f
