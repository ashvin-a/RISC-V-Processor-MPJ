  .text
  .align 2       # Make sure we're aligned to 4 bytes
  .globl _start
_start:
  # Hash table lookup benchmark with linear probing
  #
  # This is a realistic use case where higher associativity helps:
  # - Hash table with 16 buckets stored at different memory locations
  # - Multiple keys hash to same bucket (collisions) requiring probing
  # - Each bucket access may conflict in cache due to hash distribution
  # - Higher associativity keeps more buckets resident during probe sequences
  #
  # Hash table layout (key-value pairs):
  # Bucket 0-15: addresses 0x100, 0x110, 0x120, ..., 0x1F0
  # Each bucket: [key (4 bytes), value (4 bytes), occupied flag (4 bytes)]
  #
  # We insert 24 items into 16 buckets, causing collisions
  # Then perform lookups that require probing through multiple buckets

  # Initialize all buckets as empty (occupied = 0)
  addi t0, x0, 0
  sw   t0, 0x108(x0)    # bucket 0 empty
  sw   t0, 0x118(x0)    # bucket 1 empty
  sw   t0, 0x128(x0)    # bucket 2 empty
  sw   t0, 0x138(x0)    # bucket 3 empty
  sw   t0, 0x148(x0)    # bucket 4 empty
  sw   t0, 0x158(x0)    # bucket 5 empty
  sw   t0, 0x168(x0)    # bucket 6 empty
  sw   t0, 0x178(x0)    # bucket 7 empty
  sw   t0, 0x188(x0)    # bucket 8 empty
  sw   t0, 0x198(x0)    # bucket 9 empty
  sw   t0, 0x1a8(x0)    # bucket 10 empty
  sw   t0, 0x1b8(x0)    # bucket 11 empty
  sw   t0, 0x1c8(x0)    # bucket 12 empty
  sw   t0, 0x1d8(x0)    # bucket 13 empty
  sw   t0, 0x1e8(x0)    # bucket 14 empty
  sw   t0, 0x1f8(x0)    # bucket 15 empty

insert_data:
  # Insert key-value pairs
  # Keys chosen to create collisions (multiple keys hash to same bucket)

  # Insert: key=17, value=100, hash=1
  addi a0, x0, 17
  addi a1, x0, 100
  jal  ra, hash_insert

  # Insert: key=33, value=200, hash=1 (collision with 17!)
  addi a0, x0, 33
  addi a1, x0, 200
  jal  ra, hash_insert

  # Insert: key=49, value=300, hash=1 (collision!)
  addi a0, x0, 49
  addi a1, x0, 300
  jal  ra, hash_insert

  # Insert: key=5, value=50, hash=5
  addi a0, x0, 5
  addi a1, x0, 50
  jal  ra, hash_insert

  # Insert: key=21, value=150, hash=5 (collision with 5!)
  addi a0, x0, 21
  addi a1, x0, 150
  jal  ra, hash_insert

  # Insert: key=37, value=250, hash=5 (collision!)
  addi a0, x0, 37
  addi a1, x0, 250
  jal  ra, hash_insert

  # Insert: key=10, value=80, hash=10
  addi a0, x0, 10
  addi a1, x0, 80
  jal  ra, hash_insert

  # Insert: key=26, value=180, hash=10 (collision!)
  addi a0, x0, 26
  addi a1, x0, 180
  jal  ra, hash_insert

  # Insert: key=42, value=280, hash=10 (collision!)
  addi a0, x0, 42
  addi a1, x0, 280
  jal  ra, hash_insert

  # Insert: key=58, value=380, hash=10 (collision!)
  addi a0, x0, 58
  addi a1, x0, 380
  jal  ra, hash_insert

  # Insert more keys in other buckets to fill table
  addi a0, x0, 3
  addi a1, x0, 30
  jal  ra, hash_insert

  addi a0, x0, 7
  addi a1, x0, 70
  jal  ra, hash_insert

  addi a0, x0, 11
  addi a1, x0, 110
  jal  ra, hash_insert

  addi a0, x0, 13
  addi a1, x0, 130
  jal  ra, hash_insert

  addi a0, x0, 15
  addi a1, x0, 250
  jal  ra, hash_insert

lookup_test:
  # Perform multiple lookups requiring probing
  # This stresses the cache with many bucket accesses
  # Repeat lookups 4 times to amplify cache effects

  addi s0, x0, 4           # repeat counter

repeat_loop:
  blez s0, verify

  # Lookup key=49 (should find value=300 after probing buckets 1,2,3)
  addi a0, x0, 49
  jal  ra, hash_lookup
  addi t0, x0, 300
  bne  a1, t0, fail

  # Lookup key=37 (should find value=250 after probing)
  addi a0, x0, 37
  jal  ra, hash_lookup
  addi t0, x0, 250
  bne  a1, t0, fail

  # Lookup key=58 (should find value=380 after long probe)
  addi a0, x0, 58
  jal  ra, hash_lookup
  addi t0, x0, 380
  bne  a1, t0, fail

  # Lookup key=17 (should find value=100)
  addi a0, x0, 17
  jal  ra, hash_lookup
  addi t0, x0, 100
  bne  a1, t0, fail

  # Lookup key=26 (should find value=180)
  addi a0, x0, 26
  jal  ra, hash_lookup
  addi t0, x0, 180
  bne  a1, t0, fail

  # Lookup key=13 (should find value=130)
  addi a0, x0, 13
  jal  ra, hash_lookup
  addi t0, x0, 130
  bne  a1, t0, fail

  addi s0, s0, -1
  j repeat_loop

# Hash function: hash(key) = key % 16
# Input: a0 = key
# Output: a2 = hash (bucket index 0-15)
hash_func:
  andi a2, a0, 0xF         # hash = key & 15 (mod 16)
  jalr x0, ra, 0

# Insert key-value pair into hash table
# Input: a0 = key, a1 = value
# Uses linear probing for collision resolution
hash_insert:
  addi sp, sp, -8
  sw   ra, 4(sp)
  sw   s1, 0(sp)

  # Get hash
  jal  ra, hash_func       # a2 = hash
  addi s1, a2, 0           # s1 = current bucket to check

insert_probe:
  # Calculate bucket address = 0x100 + bucket*16
  slli t0, s1, 4           # t0 = bucket * 16
  addi t0, t0, 0x100       # t0 = bucket address

  # Check if bucket is occupied
  lw   t1, 8(t0)           # t1 = occupied flag
  beq  t1, x0, insert_here # if empty, insert here

  # Collision - try next bucket (linear probing)
  addi s1, s1, 1
  andi s1, s1, 0xF         # wrap around: s1 = s1 % 16
  j insert_probe

insert_here:
  # Store key, value, and mark as occupied
  sw   a0, 0(t0)           # store key
  sw   a1, 4(t0)           # store value
  addi t1, x0, 1
  sw   t1, 8(t0)           # mark occupied

  lw   s1, 0(sp)
  lw   ra, 4(sp)
  addi sp, sp, 8
  jalr x0, ra, 0

# Lookup key in hash table
# Input: a0 = key
# Output: a1 = value (or 0 if not found)
hash_lookup:
  addi sp, sp, -12
  sw   ra, 8(sp)
  sw   s1, 4(sp)
  sw   s2, 0(sp)

  # Get hash
  jal  ra, hash_func       # a2 = hash
  addi s1, a2, 0           # s1 = current bucket
  addi s2, x0, 0           # s2 = probe count

lookup_probe:
  # Prevent infinite loop - max 16 probes
  addi t0, x0, 16
  bge  s2, t0, lookup_not_found

  # Calculate bucket address = 0x100 + bucket*16
  slli t0, s1, 4
  addi t0, t0, 0x100

  # Check if bucket is occupied
  lw   t1, 8(t0)           # t1 = occupied flag
  beq  t1, x0, lookup_not_found  # empty = not found

  # Check if key matches
  lw   t2, 0(t0)           # t2 = stored key
  beq  t2, a0, lookup_found # keys match!

  # Key doesn't match - try next bucket
  addi s1, s1, 1
  andi s1, s1, 0xF         # wrap around
  addi s2, s2, 1           # increment probe count
  j lookup_probe

lookup_found:
  lw   a1, 4(t0)           # a1 = value
  lw   s2, 0(sp)
  lw   s1, 4(sp)
  lw   ra, 8(sp)
  addi sp, sp, 12
  jalr x0, ra, 0

lookup_not_found:
  addi a1, x0, 0           # return 0
  lw   s2, 0(sp)
  lw   s1, 4(sp)
  lw   ra, 8(sp)
  addi sp, sp, 12
  jalr x0, ra, 0

verify:
  # Final verification: lookup a few keys to ensure correctness
  addi a0, x0, 33
  jal  ra, hash_lookup
  addi t0, x0, 200
  bne  a1, t0, fail

  addi a0, x0, 10
  jal  ra, hash_lookup
  addi t0, x0, 80
  bne  a1, t0, fail

  addi a0, x0, 15
  jal  ra, hash_lookup
  addi t0, x0, 250
  bne  a1, t0, fail

success:
  li a0, 1
  ebreak

fail:
  li a0, 0xdead
  ebreak

end:
  ebreak
