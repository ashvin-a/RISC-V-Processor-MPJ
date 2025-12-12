  .text
  .align 2       # Make sure we're aligned to 4 bytes
  .globl _start
_start:
  # Matrix multiplication benchmark: C = A * B (4x4 matrices)
  #
  # Memory layout:
  # Matrix A: 0x100 - 0x13F (16 words)
  # Matrix B: 0x140 - 0x17F (16 words)
  # Matrix C: 0x180 - 0x1BF (16 words)

  # Initialize Matrix A (4x4) with small sequential values
  # A = [[1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16]]
  addi t0, x0, 1
  sw   t0, 0x100(x0)
  addi t0, x0, 2
  sw   t0, 0x104(x0)
  addi t0, x0, 3
  sw   t0, 0x108(x0)
  addi t0, x0, 4
  sw   t0, 0x10c(x0)

  addi t0, x0, 5
  sw   t0, 0x110(x0)
  addi t0, x0, 6
  sw   t0, 0x114(x0)
  addi t0, x0, 7
  sw   t0, 0x118(x0)
  addi t0, x0, 8
  sw   t0, 0x11c(x0)

  addi t0, x0, 9
  sw   t0, 0x120(x0)
  addi t0, x0, 10
  sw   t0, 0x124(x0)
  addi t0, x0, 11
  sw   t0, 0x128(x0)
  addi t0, x0, 12
  sw   t0, 0x12c(x0)

  addi t0, x0, 13
  sw   t0, 0x130(x0)
  addi t0, x0, 14
  sw   t0, 0x134(x0)
  addi t0, x0, 15
  sw   t0, 0x138(x0)
  addi t0, x0, 16
  sw   t0, 0x13c(x0)

  # Initialize Matrix B (4x4) with small values
  # B = [[16,15,14,13], [12,11,10,9], [8,7,6,5], [4,3,2,1]]
  addi t0, x0, 16
  sw   t0, 0x140(x0)
  addi t0, x0, 15
  sw   t0, 0x144(x0)
  addi t0, x0, 14
  sw   t0, 0x148(x0)
  addi t0, x0, 13
  sw   t0, 0x14c(x0)

  addi t0, x0, 12
  sw   t0, 0x150(x0)
  addi t0, x0, 11
  sw   t0, 0x154(x0)
  addi t0, x0, 10
  sw   t0, 0x158(x0)
  addi t0, x0, 9
  sw   t0, 0x15c(x0)

  addi t0, x0, 8
  sw   t0, 0x160(x0)
  addi t0, x0, 7
  sw   t0, 0x164(x0)
  addi t0, x0, 6
  sw   t0, 0x168(x0)
  addi t0, x0, 5
  sw   t0, 0x16c(x0)

  addi t0, x0, 4
  sw   t0, 0x170(x0)
  addi t0, x0, 3
  sw   t0, 0x174(x0)
  addi t0, x0, 2
  sw   t0, 0x178(x0)
  addi t0, x0, 1
  sw   t0, 0x17c(x0)

  # Zero-initialize Matrix C (result matrix)
  sw   x0, 0x180(x0)
  sw   x0, 0x184(x0)
  sw   x0, 0x188(x0)
  sw   x0, 0x18c(x0)
  sw   x0, 0x190(x0)
  sw   x0, 0x194(x0)
  sw   x0, 0x198(x0)
  sw   x0, 0x19c(x0)
  sw   x0, 0x1a0(x0)
  sw   x0, 0x1a4(x0)
  sw   x0, 0x1a8(x0)
  sw   x0, 0x1ac(x0)
  sw   x0, 0x1b0(x0)
  sw   x0, 0x1b4(x0)
  sw   x0, 0x1b8(x0)
  sw   x0, 0x1bc(x0)

matmul:
  # Register allocation:
  # s0: i (outer loop counter, 0 to 3)
  # s1: j (middle loop counter, 0 to 3)
  # s2: k (inner loop counter, 0 to 3)
  # s3: constant 4 (matrix dimension)
  # t0-t6: temp values
  # a0-a3: addresses and intermediate values

  addi s3, x0, 4           # s3 = 4 (matrix dimension)
  addi s0, x0, 0           # i = 0

loop_i:
  # Check if i < 4
  blt s0, s3, loop_i_body
  j matmul_done            # Exit if i >= 4

loop_i_body:
  addi s1, x0, 0           # j = 0

loop_j:
  # Check if j < 4
  blt s1, s3, loop_j_body
  j loop_i_next            # Exit j loop if j >= 4

loop_j_body:
  addi s2, x0, 0           # k = 0

  # Calculate address of C[i][j] = C[i*4 + j]
  # Address = 0x180 + (i*4 + j)*4
  slli t0, s0, 2           # t0 = i * 4
  add  t0, t0, s1          # t0 = i*4 + j
  slli t0, t0, 2           # t0 = (i*4 + j) * 4
  addi a0, t0, 0x180       # a0 = address of C[i][j]

  # Load current value of C[i][j]
  lw   a1, 0(a0)           # a1 = C[i][j]

loop_k:
  # Check if k < 4
  blt s2, s3, loop_k_body

  # Store accumulated result back to C[i][j]
  sw   a1, 0(a0)           # C[i][j] = accumulated sum
  j loop_j_next            # Exit k loop if k >= 4

loop_k_body:
  # Compute C[i][j] += A[i][k] * B[k][j]

  # Calculate address of A[i][k] = A[i*4 + k]
  slli t1, s0, 2           # t1 = i * 4
  add  t1, t1, s2          # t1 = i*4 + k
  slli t1, t1, 2           # t1 = (i*4 + k) * 4
  addi t1, t1, 0x100       # t1 = address of A[i][k]
  lw   t2, 0(t1)           # t2 = A[i][k]

  # Calculate address of B[k][j] = B[k*4 + j]
  slli t3, s2, 2           # t3 = k * 4
  add  t3, t3, s1          # t3 = k*4 + j
  slli t3, t3, 2           # t3 = (k*4 + j) * 4
  addi t3, t3, 0x140       # t3 = address of B[k][j]
  lw   t4, 0(t3)           # t4 = B[k][j]

  # Multiply A[i][k] * B[k][j] using shift-and-add
  addi t5, x0, 0           # t5 = product accumulator
  addi t6, x0, 0           # t6 = bit counter

multiply_loop:
  addi a2, x0, 32          # check if we've done 32 bits
  bge t6, a2, multiply_done

  # Check LSB of t4 (multiplier)
  andi a3, t4, 1
  beq  a3, x0, multiply_skip_add

  # Add t2 to product if bit is set
  add  t5, t5, t2

multiply_skip_add:
  # Shift multiplicand left, multiplier right
  slli t2, t2, 1
  srli t4, t4, 1
  addi t6, t6, 1
  j multiply_loop

multiply_done:
  # Add product to C[i][j]
  add  a1, a1, t5

  # Increment k
  addi s2, s2, 1
  j loop_k

loop_j_next:
  # Increment j
  addi s1, s1, 1
  j loop_j

loop_i_next:
  # Increment i
  addi s0, s0, 1
  j loop_i

matmul_done:
  # Verify first element of result matrix C[0][0]
  # Expected: 1*16 + 2*12 + 3*8 + 4*4 = 16+24+24+16 = 80
  lw t0, 0x180(x0)
  addi t1, x0, 80
  beq t0, t1, verify_second
  j fail

verify_second:
  # Verify C[0][1] = 1*15 + 2*11 + 3*7 + 4*3 = 15+22+21+12 = 70
  lw t0, 0x184(x0)
  addi t1, x0, 70
  beq t0, t1, verify_third
  j fail

verify_third:
  # Verify C[1][0] = 5*16 + 6*12 + 7*8 + 8*4 = 80+72+56+32 = 240
  lw t0, 0x190(x0)
  addi t1, x0, 240
  beq t0, t1, success
  j fail

success:
  li a0, 1
  ebreak

fail:
  li a0, 0xdead
  ebreak

end:
  ebreak
