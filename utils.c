#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

/* Unpacks the 32-bit machine code instruction given into the correct
 * type within the instruction struct */
Instruction parse_instruction(uint32_t instruction_bits) {
  /* YOUR CODE HERE */
  Instruction instruction;
  // add x9, x20, x21   hex: 01 5A 04 B3, binary = 0000 0001 0101 1010 0000 0100 1011 0011
  // Opcode: 0110011 (0x33) Get the Opcode by &ing 0x1111111, bottom 7 bits
  instruction.opcode = instruction_bits & ((1U << 7) - 1);

  // Shift right to move to pointer to interpret next fields in instruction.
  instruction_bits >>= 7;
  
  
  switch (instruction.opcode) {
  // R-Type
  case 0x33:
    // instruction: 0000 0001 0101 1010 0000 0100 1, destination : 01001
    instruction.rtype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // instruction: 0000 0001 0101 1010 0000, func3 : 000
    instruction.rtype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    // instruction: 0000 0001 0101 1010 0, src1: 10100
    instruction.rtype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // instruction: 0000 0001 0101, src2: 10101
    instruction.rtype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // funct7: 0000 000
    instruction.rtype.funct7 = instruction_bits & ((1U << 7) - 1);
    break;
  // cases for other types of instructions
  //SB-Type
  case 0x63:
    instruction.sbtype.imm5 = instruction_bits & ((1U << 5) - 1); // Use bitmasking to extract imm5 bits from instruction bits
    instruction_bits >>= 5; // Shift instruction bits to be able to extract next section

    instruction.sbtype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    instruction.sbtype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.sbtype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // funct7: 0000 000
    instruction.sbtype.imm7 = instruction_bits & ((1U << 7) - 1);
    break;
  //UJ-Type
  case 0x6F:
    instruction.ujtype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.ujtype.imm = instruction_bits & ((1U << 20) - 1);
    break;

  //I-Type (Load, Immediate, and ecall)
  case 0x03: // Load
  case 0x13: // Immediate
  case 0x73: // ecall
    instruction.itype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.itype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    instruction.itype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.itype.imm = instruction_bits & ((1U << 12) - 1);
    break;

  //S-Type
  case 0x23:
    instruction.stype.imm5 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.stype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    instruction.stype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.stype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.stype.imm7 = instruction_bits & ((1U << 7) - 1);    
    break;

  //U-Type
  case 0x37:
    instruction.utype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    instruction.utype.imm = instruction_bits & ((1U << 20) - 1);
    break;

  #ifndef TESTING
  default:
    exit(EXIT_FAILURE);
  #endif
  }
  return instruction;
}

/************************Helper functions************************/
/* Here, you will need to implement a few common helper functions, 
 * which you will call in other functions when parsing, printing, 
 * or executing the instructions. */

/* Sign extends the given field to a 32-bit integer where field is
 * interpreted an n-bit integer. */
int sign_extend_number(unsigned int field, unsigned int n) {
  int shift = 32-n;    // Determine number of bits to shift to make 32 bit
  int extendedNum = field << shift;    // Shift the original number by "shift" bits to pad with zeros on right
  extendedNum = extendedNum >> shift;  // Shift left shifted number by "shift" bits to pad with signed digits on left
  return extendedNum;
}

/* Return the number of bytes (from the current PC) to the branch label using
 * the given branch instruction */
int get_branch_offset(Instruction instruction) {

  int twelve = (instruction.sbtype.imm7 >> 6) & 1U;     // Use bitmasking to extract desired bits
  int eleven = instruction.sbtype.imm5 & 1U;     
  int ten_to_five = instruction.sbtype.imm7 & 0x3F;         
  int four_to_one = (instruction.sbtype.imm5 >> 1) & 0xF;    

  int offset = (twelve << 12) | (eleven << 11) | (ten_to_five << 5) | (four_to_one << 1);  // Use bitwise OR to combine immediate bits

  return sign_extend_number(offset, 13); // Use signed extender function to return 32 bit signed number
}

/* Returns the number of bytes (from the current PC) to the jump label using the
 * given jump instruction */
int get_jump_offset(Instruction instruction) {
  int twenty = (instruction.ujtype.imm >> 19) & 1U;
  int nineteen_to_twelve = instruction.ujtype.imm & 0xFF;
  int eleven = (instruction.ujtype.imm >> 8) & 1U;
  int ten_to_one = (instruction.ujtype.imm >> 9) & 0x3FF;
  int output = (twenty << 20) | (nineteen_to_twelve << 12) | (eleven << 11) | (ten_to_one << 1);
  return sign_extend_number(output, 21);
}

/* Returns the number of bytes (from the current PC) to the base address using the
 * given store instruction */
int get_store_offset(Instruction instruction) {
  int eleven_to_five = instruction.stype.imm7 << 5;
  int output = eleven_to_five | instruction.stype.imm5;
  return sign_extend_number(output, 12);
}
/************************Helper functions************************/

void handle_invalid_instruction(Instruction instruction) {
  printf("Invalid Instruction: 0x%08x\n", instruction.bits);
}

void handle_invalid_read(Address address) {
  printf("Bad Read. Address: 0x%08x\n", address);
  exit(-1);
}

void handle_invalid_write(Address address) {
  printf("Bad Write. Address: 0x%08x\n", address);
  exit(-1);
}
