.section .data
ball_pattern:
    .byte 0x7e
    .byte 0xff
    .byte 0xff
    .byte 0x7e
ball_x:
    .byte 0x07
ball_y:
    .byte 0x00
ball_dx:
    .byte 0x01
ball_dy:
    .byte 0x01

delay:
    .byte 0x10

.section .text
main:
    # Draw the ball
    mov cs r0
    li 0x10 r1
    add r1 r0
    li draw_ball r1
    call r0 r1

    # Move the ball
    mov cs r0
    li 0x20 r1
    add r1 r0
    li move_ball r1
    call r0 r1

    # Loop to the next frame
    li main r0
    jmp r0

    .byte 0x00 0xdc

draw_ball:
    # Draw the ball
    li ball_x r0
    lr r0 r0
    li ball_y r1
    lr r1 r1
    li ball_pattern r2
    li 0x04 r3
    push lr
    push ls
    mov cs r4
    li 0x20 r5
    add r5 r4
    li draw_pattern r5
    call r4 r5
    pop ls
    pop lr

    # Make a pause
    li delay r0
    lr r0 r0
    li 0x01 r1
_pause:
    sub r1 r0
    cmp r1 r0
    li _pause r2
    jae r2

    # Clear the ball
    li ball_x r0
    lr r0 r0
    li ball_y r1
    lr r1 r1
    li 0x04 r2
    push lr
    push ls
    mov cs r3
    li 0x20 r4
    add r4 r3
    li clear_square r4
    call r3 r4
    pop ls
    pop lr

    ret

    .byte 0x00 0x91

move_ball:
    # Move the ball
    li ball_x r0
    lr r0 r0
    li ball_y r1
    lr r1 r1
    li ball_dx r2
    lr r2 r2
    li ball_dy r3
    lr r3 r3
    add r2 r0
    add r3 r1

    # Check for collisions
_right:
    li 0x0f r4
    cmp r4 r0
    li _left r4
    jb r4
    not r2
    li 0x01 r4
    add r4 r2
_left:
    li 0x00 r4
    cmp r4 r0
    li _bottom r4
    ja r4
    not r2
    li 0x01 r4
    add r4 r2
_bottom:
    li 0x0f r4
    cmp r4 r1
    li _top r4
    jb r4
    not r3
    li 0x01 r4
    add r4 r3
_top:
    li 0x00 r4
    cmp r4 r1
    li _save r4
    ja r4
    not r3
    li 0x01 r4
    add r4 r3

_save:
    # Save the new state of the ball
    li ball_x r4
    sr r0 r4
    li ball_y r4
    sr r1 r4
    li ball_dx r4
    sr r2 r4
    li ball_dy r4
    sr r3 r4

    ret

    .byte 0x00 0x73

draw_pattern:
    # Convert x and y coords into a linear memory address
    push r2
    push r3
    push lr
    push ls
    li xy_to_ds_off r2
    call cs r2
    pop ls
    pop lr
    pop r3
    pop r2

    # Draw n bytes of the given pattern
    li 0x00 r4
_pattern_loop:
    push ds
    lr r2 r5
    li 0xc0 ds
    add r0 ds
    sr r5 r1
    pop ds
    li 0x10 r5
    add r5 r1
    li 0x01 r5
    add r5 r2
    add r5 r4
    cmp r3 r4
    li _pattern_loop r5
    jl r5

    ret

clear_square:
    # Convert x and y coords into a linear memory address
    push r2
    push lr
    push ls
    li xy_to_ds_off r2
    call cs r2
    pop ls
    pop lr
    pop r2

    # Clear n bytes
    push ds
    li 0xc0 ds
    add r0 ds
    li 0x00 r0
    li 0x10 r3
    li 0x01 r4
_clear_loop:
    sr r0 r1
    add r3 r1
    sub r4 r2
    cmp r0 r2
    li _clear_loop r5
    jge r5
    pop ds

    ret

xy_to_ds_off:
    # Convert 8-bit x and y coords into a 16-bit linear coord (y * 64 (16 bytes per row * 4 rows) + x)
    mov r1 r2
    mov r1 r3
    li 0x06 r4
    shl r4 r2
    li 0x02 r4
    shr r4 r3
    add r0 r2
    li 0x00 r4
    adc r4 r3

    # Convert the 16-bit linear coord into a segment (coord / 16) and offset (coord % 16) form
    mov r2 r0
    mov r2 r1
    li 0x0f r4
    and r4 r1
    li 0x04 r4
    shr r4 r0
    shl r4 r3
    or r3 r0

    ret
