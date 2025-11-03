.global _start
_start:
addi x1,x0,4
addi x2,x0,0
addi x7,x0,0x8
addi x11,x0,0x8
addi x12,x0,0x000003FF
addi x0,x0,0
addi x0,x0,0
addi x0,x0,0
sw x7,0(x2)
sw x11,4(x1)
sw x12,8(x1)
addi x0,x0,0
addi x0,x0,0
lw x1,0(x2)
addi x0,x0,0
addi x0,x0,0
addi x0,x0,0
addi x0,x0,0
lw x3,4(x1)
addi x0,x0,0
addi x0,x0,0
add x5,x3,x3
ebreak