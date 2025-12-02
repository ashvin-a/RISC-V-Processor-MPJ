  .text
  .align 2       # Make sure we're aligned to 4 bytes
  .globl _start
_start:
  # Image filter benchmark - simulates a 2D convolution/smoothing filter
  # Image: 8x8 pixels (64 pixels) at addresses 0x100-0x1FF
  # Each "pixel" is 1 word (4 bytes) for simplicity

  # Initialize 8x8 image with gradient values
  # Row 0
  addi t0, x0, 10
  sw   t0, 0x100(x0)
  addi t0, x0, 20
  sw   t0, 0x104(x0)
  addi t0, x0, 30
  sw   t0, 0x108(x0)
  addi t0, x0, 40
  sw   t0, 0x10c(x0)
  addi t0, x0, 50
  sw   t0, 0x110(x0)
  addi t0, x0, 60
  sw   t0, 0x114(x0)
  addi t0, x0, 70
  sw   t0, 0x118(x0)
  addi t0, x0, 80
  sw   t0, 0x11c(x0)

  # Row 1
  addi t0, x0, 15
  sw   t0, 0x120(x0)
  addi t0, x0, 25
  sw   t0, 0x124(x0)
  addi t0, x0, 35
  sw   t0, 0x128(x0)
  addi t0, x0, 45
  sw   t0, 0x12c(x0)
  addi t0, x0, 55
  sw   t0, 0x130(x0)
  addi t0, x0, 65
  sw   t0, 0x134(x0)
  addi t0, x0, 75
  sw   t0, 0x138(x0)
  addi t0, x0, 85
  sw   t0, 0x13c(x0)

  # Row 2
  addi t0, x0, 12
  sw   t0, 0x140(x0)
  addi t0, x0, 22
  sw   t0, 0x144(x0)
  addi t0, x0, 32
  sw   t0, 0x148(x0)
  addi t0, x0, 42
  sw   t0, 0x14c(x0)
  addi t0, x0, 52
  sw   t0, 0x150(x0)
  addi t0, x0, 62
  sw   t0, 0x154(x0)
  addi t0, x0, 72
  sw   t0, 0x158(x0)
  addi t0, x0, 82
  sw   t0, 0x15c(x0)

  # Row 3
  addi t0, x0, 18
  sw   t0, 0x160(x0)
  addi t0, x0, 28
  sw   t0, 0x164(x0)
  addi t0, x0, 38
  sw   t0, 0x168(x0)
  addi t0, x0, 48
  sw   t0, 0x16c(x0)
  addi t0, x0, 58
  sw   t0, 0x170(x0)
  addi t0, x0, 68
  sw   t0, 0x174(x0)
  addi t0, x0, 78
  sw   t0, 0x178(x0)
  addi t0, x0, 88
  sw   t0, 0x17c(x0)

  # Row 4
  addi t0, x0, 14
  sw   t0, 0x180(x0)
  addi t0, x0, 24
  sw   t0, 0x184(x0)
  addi t0, x0, 34
  sw   t0, 0x188(x0)
  addi t0, x0, 44
  sw   t0, 0x18c(x0)
  addi t0, x0, 54
  sw   t0, 0x190(x0)
  addi t0, x0, 64
  sw   t0, 0x194(x0)
  addi t0, x0, 74
  sw   t0, 0x198(x0)
  addi t0, x0, 84
  sw   t0, 0x19c(x0)

  # Row 5
  addi t0, x0, 16
  sw   t0, 0x1a0(x0)
  addi t0, x0, 26
  sw   t0, 0x1a4(x0)
  addi t0, x0, 36
  sw   t0, 0x1a8(x0)
  addi t0, x0, 46
  sw   t0, 0x1ac(x0)
  addi t0, x0, 56
  sw   t0, 0x1b0(x0)
  addi t0, x0, 66
  sw   t0, 0x1b4(x0)
  addi t0, x0, 76
  sw   t0, 0x1b8(x0)
  addi t0, x0, 86
  sw   t0, 0x1bc(x0)

  # Row 6
  addi t0, x0, 11
  sw   t0, 0x1c0(x0)
  addi t0, x0, 21
  sw   t0, 0x1c4(x0)
  addi t0, x0, 31
  sw   t0, 0x1c8(x0)
  addi t0, x0, 41
  sw   t0, 0x1cc(x0)
  addi t0, x0, 51
  sw   t0, 0x1d0(x0)
  addi t0, x0, 61
  sw   t0, 0x1d4(x0)
  addi t0, x0, 71
  sw   t0, 0x1d8(x0)
  addi t0, x0, 81
  sw   t0, 0x1dc(x0)

  # Row 7
  addi t0, x0, 13
  sw   t0, 0x1e0(x0)
  addi t0, x0, 23
  sw   t0, 0x1e4(x0)
  addi t0, x0, 33
  sw   t0, 0x1e8(x0)
  addi t0, x0, 43
  sw   t0, 0x1ec(x0)
  addi t0, x0, 53
  sw   t0, 0x1f0(x0)
  addi t0, x0, 63
  sw   t0, 0x1f4(x0)
  addi t0, x0, 73
  sw   t0, 0x1f8(x0)
  addi t0, x0, 83
  sw   t0, 0x1fc(x0)

filter_passes:
  # Apply smoothing filter multiple times (5 passes)
  # Each pass: for each interior pixel, replace with average of neighbors
  #
  # Filter: new[i] = (old[i] + old[i-1] + old[i+1] + old[i-8] + old[i+8]) / 5

  addi s0, x0, 5           # s0 = number of passes

pass_loop:
  blez s0, verify

  # Only filter interior pixels (row 1-6, col 1-6)
  # to avoid boundary checking overhead
  # Start at pixel[9] (row 1, col 1) = address 0x124

  addi s1, x0, 6           # s1 = rows to process (1-6)
  addi s2, x0, 1           # s2 = current row

row_loop:
  blez s1, pass_done

  addi s3, x0, 6           # s3 = cols to process
  addi s4, x0, 1           # s4 = current col

col_loop:
  blez s3, row_done

  # Calculate address of current pixel: 0x100 + (row*8 + col)*4
  slli t0, s2, 3           # t0 = row * 8
  add  t0, t0, s4          # t0 = row*8 + col
  slli t0, t0, 2           # t0 = (row*8 + col) * 4
  addi t0, t0, 0x100       # t0 = pixel address

  # Load center pixel
  lw   t1, 0(t0)           # t1 = center

  # Load left neighbor (col-1)
  lw   t2, -4(t0)          # t2 = left

  # Load right neighbor (col+1)
  lw   t3, 4(t0)           # t3 = right

  # Load top neighbor (row-1)
  lw   t4, -32(t0)         # t4 = top (8 pixels back = 32 bytes)

  # Load bottom neighbor (row+1)
  lw   t5, 32(t0)          # t5 = bottom

  # Compute average: (center + left + right + top + bottom) / 5
  add  t6, t1, t2
  add  t6, t6, t3
  add  t6, t6, t4
  add  t6, t6, t5

  # Divide by 5 using iterative subtraction
  # a0 = quotient, a1 = counter
  addi a0, x0, 0           # quotient = 0
  addi a1, t6, 0           # remainder = sum

div_loop:
  addi a2, x0, 5
  blt a1, a2, div_done     # if remainder < 5, done
  addi a0, a0, 1           # quotient++
  addi a1, a1, -5          # remainder -= 5
  j div_loop

div_done:
  # a0 now contains sum/5

  # Write new value back to SAME location
  # This is the key: repeated writes to same addresses
  sw   a0, 0(t0)

  # Next column
  addi s4, s4, 1
  addi s3, s3, -1
  j col_loop

row_done:
  # Next row
  addi s2, s2, 1
  addi s1, s1, -1
  j row_loop

pass_done:
  # Next pass
  addi s0, s0, -1
  j pass_loop

verify:
  # Verify that filtering occurred
  # After smoothing, center pixel (row 3, col 3 = pixel 27) should be smoothed
  # Address: 0x100 + 27*4 = 0x16c
  lw t0, 0x16c(x0)

  # Original value was 48, after 5 passes of in-place averaging
  # values tend to converge toward local average
  # Just check it's in reasonable range (10-100)
  addi t1, x0, 10
  blt t0, t1, fail         # too small
  addi t1, x0, 100
  blt t1, t0, fail         # too large

  # Verify edge pixel didn't change (we don't filter edges)
  # Pixel[0] should still be 10
  lw t0, 0x100(x0)
  addi t1, x0, 10
  bne t0, t1, fail

  # Verify another edge pixel
  # Pixel[7] (row 0, col 7) should still be 80
  lw t0, 0x11c(x0)
  addi t1, x0, 80
  bne t0, t1, fail

  # Check a filtered pixel changed
  # Pixel[9] (row 1, col 1) should have changed from 25
  lw t0, 0x124(x0)
  addi t1, x0, 25
  beq t0, t1, fail         # Should NOT be original value

success:
  li a0, 1
  ebreak

fail:
  li a0, 0xdead
  ebreak

end:
  ebreak
