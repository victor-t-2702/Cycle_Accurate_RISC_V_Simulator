#ifndef __STAGE_HELPERS_H__
#define __STAGE_HELPERS_H__

#include <stdio.h>
#include "utils.h"
#include "pipeline.h"

/// EXECUTE STAGE HELPERS ///

/**
 * input  : idex_reg_t
 * output : uint32_t alu_control signal
 **/
uint32_t gen_alu_control(idex_reg_t idex_reg)
{
  uint32_t alu_control = 0;
  Instruction instr = idex_reg.instr;
  switch(idex_reg.instr.opcode) {
    // R-Type
    case 0x33:
      switch(idex_reg.instr.rtype.funct3) {
        case 0x0:
          if(idex_reg.instr.rtype.funct7 == 0x00) {
            alu_control = 0x0; // Addition
          }
          else {
            alu_control = 0x1; // Subtraction
          }
          break;
        case 0x4:
          alu_control = 0x3; // XOR
          break;
        case 0x6:
          alu_control = 0x4; // OR
          break;
        case 0x7:
          alu_control = 0x5; // AND
          break;
        case 0x1:
          alu_control = 0x6; // SLL (Shift Left Logical)
          break;
        case 0x5:
          if(idex_reg.instr.rtype.funct7 == 0x00) {
            alu_control = 0x7; // SRL (Shift Right Logical)
          }
          else {
            alu_control = 0x8; // SRA (Shift Right Arithmetic)
          }
          break;
        case 0x2:
          alu_control = 0x9; // SLT (Set Less Than)
          break;
        case 0x3:
          alu_control = 0x10; // SLTU (Set Less Than Unsigned)
          break;
        default:
          alu_control = 0x111; // Invalid instruction
      }
      break;
    // cases for other types of instructions


    //I-Type (Load, Immediate, and ecall)
    case 0x13: // Immediate
      switch(idex_reg.instr.itype.funct3) {
        case 0x0: 
          alu_control = 0x0;  // ADD
          break;  // ADDI
        case 0x1: 
          alu_control = 0x6; // SLL (Shift Left Logical)
          break;
        case 0x2: 
          alu_control = 0x9; // SLT (Set Less Than)
          break;  // SLTI
        case 0x3: 
          alu_control = 0x10; // SLTU (Set Less Than Unsigned)
          break; // SLTIU
        case 0x4: 
          alu_control = 0x3; // XOR
          break;  // XORI
        case 0x5: // SRLI or SRAI
          int immShifted = (instr.itype.imm >> 5);  
          int funct7 = immShifted & ((1U << 7) - 1); // Use bitmasking to extract funct7 bits for slli, srli, and srai instructions
          if(funct7 == 0x00) { // SRLI             
            alu_control = 0x7; // SRL (Shift Right Logical)
          }
          else {   // SRAI           
            alu_control = 0x8; // SRA (Shift Right Arithmetic)
          }
          break;  
        case 0x6: 
          alu_control = 0x4; // OR;
          break;   // ORI
        case 0x7: 
          alu_control = 0x5; // AND
          break;  // ANDI
        default: 
          alu_control = 0x111; // Invalid instruction
          break;
        }
      break;
      
      case 0x03: // Load 
      case 0x23: // Store
        alu_control = 0x0; // Addition  (Effective Address = base address + offset from immediate)
        break;

        //SB-Type (Branch)
    case 0x63:
      alu_control = 0x1; // Subtraction  (Branch uses subtraction to determine if condition is true (comparisons))
      break;

    case 0x6F:  // JAL
      alu_control = 0x0; // Addition  (To add 4 to PC for return address)
      break;

    default:
      alu_control = 0x111; // Invalid instruction
    }

  return alu_control;
}

/**
 * input  : alu_inp1, alu_inp2, alu_control
 * output : uint32_t alu_result
 **/
uint32_t execute_alu(uint32_t alu_inp1, uint32_t alu_inp2, uint32_t alu_control)
{
  uint32_t result;
  switch(alu_control){
    case 0x0: //add
      result = alu_inp1 + alu_inp2;
      break;
    case 0x1: //subtract
      result = alu_inp1 - alu_inp2;
      break;
    case 0x3: //XOR
      result = alu_inp1 ^ alu_inp2;
      break;
    case 0x4: //OR
      result = alu_inp1 | alu_inp2;
      break;
    case 0x5: //AND
      result = alu_inp1 & alu_inp2;
      break;
    case 0x6: //SLL
      result = alu_inp1 << alu_inp2;
      break;
    case 0x7: //SRL  
      result = (uint32_t)alu_inp1 >> alu_inp2;  // Cast to unsigned first so that we don't pad with sign bit
      break;
    case 0x8: //SRA 
      result = (int32_t)alu_inp1 >> alu_inp2;   // Convert to signed int first to ensure sign bit padding
      break;
    case 0x9: //SLT
      result = ((int32_t)alu_inp1 < (int32_t)alu_inp2)?1:0;  // Convert to signed int first to do signed comparison
      break;
    case 0x10: //SLTU
      result = ((uint32_t)alu_inp1 < (uint32_t)alu_inp2)?1:0;
      break;

    default:
      result = 0xBADCAFFE;
      break;
  };
  return result;
}

/// DECODE STAGE HELPERS ///

/**
 * input  : Instruction
 * output : idex_reg_t
 **/
uint32_t gen_imm(Instruction instruction)
{
  int imm_val = 0;
  switch(instruction.opcode) {
    case 0x63: //B-type
      imm_val = get_branch_offset(instruction);
      break;
    //Non-load I-type
    case 0x13:
      imm_val = sign_extend_number(instruction.itype.imm, 12);
      break;
    //Load I-type
    case 0x03:
      imm_val = sign_extend_number(instruction.itype.imm, 12);
      break;
    //S-type
    case 0x23:
      imm_val = get_store_offset(instruction);
      break;
    //lui 
    case 0x37:
      //I dunnoe if i sign extend or nah, i mightve done this one wrong
      imm_val = instruction.utype.imm;
      break;
    //jal
    case 0x6f:
      imm_val = get_jump_offset(instruction);
      break;
    default: // R and undefined opcode
      break;
  };
  return imm_val;
}

/**
 * generates all the control logic that flows around in the pipeline
 * input  : Instruction
 * output : idex_reg_t
 **/
idex_reg_t gen_control(Instruction instruction)
{
  idex_reg_t idex_reg = {0};
  switch(instruction.opcode) {
    case 0x33:  //R-type
      idex_reg.ALUSrc = 0;
      idex_reg.MemWrite = 0;
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 1;
      idex_reg.Branch = 0;
      break;
    //Non-load I-type
    case 0x13:
      idex_reg.ALUSrc = 1;
      idex_reg.MemWrite = 0;
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 1;
      idex_reg.Branch = 0;
      break;
    //Load I-type
    case 0x03:
      idex_reg.ALUSrc = 1;
      idex_reg.MemWrite = 0;
      idex_reg.MemRead = 1;
      idex_reg.MemtoReg = 1;
      idex_reg.RegWrite = 1;
      idex_reg.Branch = 0;
      break;
    //S-type
    case 0x23:
      idex_reg.ALUSrc = 1;
      idex_reg.MemWrite = 1;
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 0;
      idex_reg.Branch = 0;
      break;
    //B-type
    case 0x63:
      idex_reg.ALUSrc = 0;
      idex_reg.MemWrite = 0;
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 0;
      idex_reg.Branch = 1;
      break;
    //lui 
    case 0x37:
      idex_reg.ALUSrc = 1;
      idex_reg.MemWrite = 0;
      //I think it directly loads the immediate into the rd, so no mem read/write, and 0 for mem to reg
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 1;
      idex_reg.Branch = 0;
      break;
    //jal
    case 0x6f:
      idex_reg.ALUSrc = 1;
      idex_reg.MemWrite = 0;
      idex_reg.MemRead = 0;
      idex_reg.MemtoReg = 0;
      idex_reg.RegWrite = 1;
      //I think this is right
      idex_reg.Branch = 1;
      break;
    default:  // Remaining opcodes
      break;
  }
  return idex_reg;
}

/// MEMORY STAGE HELPERS ///

/**
 * evaluates whether a branch must be taken
 * input  : <open to implementation>
 * output : bool
 **/
bool gen_branch(exmem_reg_t exmem_reg)  // For conditional: determine if condition is met; For unconditional: just set PCSrc to 1
{
  bool PCSrc = 0;
  uint32_t ALU_Result = exmem_reg.ALU_Result;
  bool isZero = exmem_reg.isZero;
  Register rs1_val = exmem_reg.rs1_val;
  Register rs2_val = exmem_reg.rs2_val;

  if(exmem_reg.instr.opcode == 0x6F) {  // JAL
    PCSrc = 1;  // Unconditional branch, so immediately set PCSrc to 1
  }
  else if(exmem_reg.instr.opcode == 0x63) {   // BRANCH
    switch(exmem_reg.instr.sbtype.funct3) {
      case 0x0:  // beq
        PCSrc = (isZero == 1)?1:0;
        break;
      case 0x1:   // bne
        PCSrc = (isZero == 0)?1:0;
        break;
      case 0x4:   // blt
        PCSrc = ((int32_t)rs1_val < (int32_t)rs2_val)?1:0;   // Signed, so cast to signed int first
        break;
      case 0x5:   // bge
        PCSrc = ((int32_t)rs1_val >= (int32_t)rs2_val)?1:0;
        break;
      case 0x6:   // bltu
        PCSrc = (rs1_val < rs2_val)?1:0;
        break;
      case 0x7:   // bgeu
        PCSrc = (rs1_val >= rs2_val)?1:0;
        break;
      default:
        PCSrc = 0;
        break;
    }
  }
  else {
    PCSrc = 0;
  }

  return PCSrc;
}


/// PIPELINE FEATURES ///

/**
 * Task   : Sets the pipeline wires for the forwarding unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
*/
void gen_forward(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p)
{
  /**
   * YOUR CODE HERE
   */
}

/**
 * Task   : Sets the pipeline wires for the hazard unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
*/
void detect_hazard(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  /**
   * YOUR CODE HERE
   */
}


///////////////////////////////////////////////////////////////////////////////


/// RESERVED FOR PRINTING REGISTER TRACE AFTER EACH CLOCK CYCLE ///
void print_register_trace(regfile_t* regfile_p)
{
  // print
  for (uint8_t i = 0; i < 8; i++)       // 8 columns
  {
    for (uint8_t j = 0; j < 4; j++)     // of 4 registers each
    {
      printf("r%2d=%08x ", i * 4 + j, regfile_p->R[i * 4 + j]);
    }
    printf("\n");
  }
  printf("\n");
}

#endif // __STAGE_HELPERS_H__
