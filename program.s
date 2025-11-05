.global _start
_start:
.text
main:
    addi x8, x0, 0x100
    addi x0,x0,0
    addi x0,x0,0
test_2:
    jal x1 , reset
    addi x0,x0,0
    addi x0,x0,0
    lui x5 , 74561
    addi x0,x0,0
    addi x0,x0,0
    addi x5,x5,564
    addi x0,x0,0
    addi x0,x0,0
    sw x5, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    lw x6, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    addi x3, x0, 2
    addi x0,x0,0
    addi x0,x0,0
    bne x5 , x6, fail
test_3:
    addi x0,x0,0
    addi x0,x0,0
    jal x1, reset
    addi x0,x0,0
    addi x0,x0,0
    addi x5,x0, -1234
    addi x0,x0,0
    addi x0,x0,0
    sh x5, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    lh x6, 0(s0)
    addi x0,x0,0
    addi x0,x0,0
    lhu x7, 0(s0)
    addi x0,x0,0
    addi x0,x0,0
    addi x3, x0, 3
    addi x0,x0,0
    addi x0,x0,0
    bne x5, x6, fail
    addi x0,x0,0
    addi x0,x0,0
    lui x28, 16
    addi x0,x0,0
    addi x0,x0,0
    addi x28, x28, -1234
    addi x0,x0,0
    addi x0,x0,0
    bne x7, x28, fail
test_4:
    addi x0,x0,0
    addi x0,x0,0
    jal x1, reset
    addi x0,x0,0
    addi x0,x0,0
    addi x5, x0, -123
    addi x0,x0,0
    addi x0,x0,0
    sh x5, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    lb x6, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    lbu x7, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    addi x3, x0, 4
    addi x0,x0,0
    addi x0,x0,0
    bne x5, x6, fail
    addi x0,x0,0
    addi x0,x0,0
    addi x28, x0, 133
    addi x0,x0,0
    addi x0,x0,0
    bne x7, x28, fail
pass:
    addi x0,x0,0
    addi x0,x0,0
    addi x10, x0, 1
    addi x0,x0,0
    addi x0,x0,0
    ecall
fail:
    addi x0,x0,0
    addi x0,x0,0
    lui x10, 14
    addi x0,x0,0
    addi x0,x0,0
    addi x10, x10, -339
    addi x0,x0,0
    addi x0,x0,0
    ecall
reset:
    addi x0,x0,0
    addi x0,x0,0
    addi x5, x0, 0	
    addi x0,x0,0
    addi x0,x0,0
    addi x6, x0, 0	
    addi x0,x0,0
    addi x0,x0,0
    addi x7, x0, 0	
    addi x0,x0,0
    addi x0,x0,0
    addi x28, x0, 0
    addi x0,x0,0
    addi x0,x0,0
    sw x0, 0(x8)
    addi x0,x0,0
    addi x0,x0,0
    jalr x0,x1, 0
