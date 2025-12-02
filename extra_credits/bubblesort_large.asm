  .text
  .align 2       # Make sure we're aligned to 4 bytes
  .globl _start
_start:
  # Large bubble sort benchmark with multiple passes
  #
  # Array of 32 elements at 0x100-0x17F

  # Initialize array with pseudo-random values
  # Using values that will require many swaps
  addi t0, x0, 157
  sw   t0, 0x100(x0)
  addi t0, x0, 89
  sw   t0, 0x104(x0)
  addi t0, x0, 234
  sw   t0, 0x108(x0)
  addi t0, x0, 12
  sw   t0, 0x10c(x0)

  addi t0, x0, 199
  sw   t0, 0x110(x0)
  addi t0, x0, 45
  sw   t0, 0x114(x0)
  addi t0, x0, 178
  sw   t0, 0x118(x0)
  addi t0, x0, 67
  sw   t0, 0x11c(x0)

  addi t0, x0, 223
  sw   t0, 0x120(x0)
  addi t0, x0, 98
  sw   t0, 0x124(x0)
  addi t0, x0, 156
  sw   t0, 0x128(x0)
  addi t0, x0, 34
  sw   t0, 0x12c(x0)

  addi t0, x0, 211
  sw   t0, 0x130(x0)
  addi t0, x0, 78
  sw   t0, 0x134(x0)
  addi t0, x0, 189
  sw   t0, 0x138(x0)
  addi t0, x0, 23
  sw   t0, 0x13c(x0)

  addi t0, x0, 245
  sw   t0, 0x140(x0)
  addi t0, x0, 56
  sw   t0, 0x144(x0)
  addi t0, x0, 167
  sw   t0, 0x148(x0)
  addi t0, x0, 101
  sw   t0, 0x14c(x0)

  addi t0, x0, 134
  sw   t0, 0x150(x0)
  addi t0, x0, 87
  sw   t0, 0x154(x0)
  addi t0, x0, 212
  sw   t0, 0x158(x0)
  addi t0, x0, 43
  sw   t0, 0x15c(x0)

  addi t0, x0, 176
  sw   t0, 0x160(x0)
  addi t0, x0, 91
  sw   t0, 0x164(x0)
  addi t0, x0, 203
  sw   t0, 0x168(x0)
  addi t0, x0, 29
  sw   t0, 0x16c(x0)

  addi t0, x0, 198
  sw   t0, 0x170(x0)
  addi t0, x0, 65
  sw   t0, 0x174(x0)
  addi t0, x0, 143
  sw   t0, 0x178(x0)
  addi t0, x0, 18
  sw   t0, 0x17c(x0)

bubblesort:
  # Bubble sort implementation with optimization flag
  # Register allocation:
  # s0: outer loop counter (starts at 31, counts down)
  # s1: swapped flag (1 if any swap occurred)
  # s2: inner loop counter
  # s3: array size (32)
  # t0-t6: temporary values

  addi s3, x0, 32          # s3 = array size
  addi s0, s3, -1          # s0 = 31 (outer loop counter)

outer_loop:
  # Check if we need more passes
  blez s0, sort_done       # If s0 <= 0, we're done

  # Initialize for this pass
  addi s1, x0, 0           # swapped = false
  addi s2, s0, 0           # inner counter = outer counter
  addi t2, x0, 0x100       # ptr_curr = base address
  addi t3, x0, 0x104       # ptr_next = base + 4

inner_loop:
  # Check if inner loop is done
  blez s2, check_swapped   # If s2 <= 0, end inner loop

  # Load current and next elements
  lw   t0, 0(t2)           # t0 = arr[i]
  lw   t1, 0(t3)           # t1 = arr[i+1]

  # Compare elements (unsigned)
  bltu t1, t0, do_swap     # If arr[i+1] < arr[i], swap

continue_inner:
  # Move to next pair
  addi t2, t2, 4           # ptr_curr++
  addi t3, t3, 4           # ptr_next++
  addi s2, s2, -1          # counter--
  j inner_loop

do_swap:
  # Swap elements
  sw   t1, 0(t2)           # arr[i] = arr[i+1]
  sw   t0, 0(t3)           # arr[i+1] = arr[i]
  addi s1, x0, 1           # swapped = true
  j continue_inner

check_swapped:
  # Early exit optimization: if no swaps, array is sorted
  beq s1, x0, sort_done    # If swapped == false, done

  # Continue to next pass
  addi s0, s0, -1          # outer_counter--
  j outer_loop

sort_done:
  # Verification: check if array is sorted
  addi t0, x0, 0x100       # ptr = base
  addi t1, x0, 31          # counter = 31 (n-1 comparisons)

verify_loop:
  blez t1, verify_success  # If counter <= 0, verification done

  # Load consecutive elements
  lw   t2, 0(t0)           # current element
  lw   t3, 4(t0)           # next element

  # Check if current <= next (sorted order)
  bltu t3, t2, verify_fail # If next < current, not sorted!

  # Move to next element
  addi t0, t0, 4           # ptr++
  addi t1, t1, -1          # counter--
  j verify_loop

verify_success:
  # Additional check: verify first element is smallest (12)
  lw t0, 0x100(x0)
  addi t1, x0, 12
  beq t0, t1, check_last
  j verify_fail

check_last:
  # Verify last element is largest (245)
  lw t0, 0x17c(x0)
  addi t1, x0, 245
  beq t0, t1, check_middle
  j verify_fail

check_middle:
  # Verify a middle element (16th element should be around median)
  # After sorting: position 15 (0x13c) should have value <= 143
  lw t0, 0x13c(x0)
  addi t1, x0, 143
  bltu t0, t1, success     # If arr[15] < 143, likely correct
  beq t0, t1, success      # Or equal
  j verify_fail

verify_fail:
  li a0, 0xdead
  ebreak

success:
  li a0, 1
  ebreak

end:
  ebreak