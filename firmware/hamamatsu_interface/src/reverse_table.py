#!/usr/bin/env python3

# generate a lookup table of 12 bit numbers to reverse the bits

for i in range(4096):
  print(hex(int(bin(i)[2:].zfill(12)[::-1],2)), end=", ")
