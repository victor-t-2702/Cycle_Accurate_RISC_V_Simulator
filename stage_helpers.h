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

  //Just moved it up
  int immShifted = (instr.itype.imm >> 5);  
  int funct7 = immShifted & ((1U << 7) - 1); // Use bitmasking to extract funct7 bits for slli, srli, and srai instructions
  switch(idex_reg.instr.opcode) {
    // R-Type
    case 0x33:
      switch(idex_reg.instr.rtype.funct3) {
        case 0x0:
          if(idex_reg.instr.rtype.funct7 == 0x00) {
            alu_control = 0x0; // Addition
          }
          else if(idex_reg.instr.rtype.funct7 == 0x20) {
            alu_control = 0x1; // Subtraction
          }
          else if(idex_reg.instr.rtype.funct7 == 0x01){
            alu_control = 0x11; //Multiply
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
          //int immShifted = (instr.itype.imm >> 5);  
          //int funct7 = immShifted & ((1U << 7) - 1); // Use bitmasking to extract funct7 bits for slli, srli, and srai instructions
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
    case 0x11: //MUL
      result = (alu_inp1 * alu_inp2) & (0xffffffff);
      break;
    default:
      result = 0xBACCAFFE;
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
  pwires_p->forwardA = 0;
  pwires_p->forwardB = 0;
  // Detecting EX hazard with Previous Instruction and MEM hazard
  //NOT SURE IF I'M SUPPOSED TO USE preg.out or .inp
  if(pregs_p->exmem_preg.out.RegWrite && (pregs_p->exmem_preg.out.RegisterRd != 0) && (pregs_p->exmem_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs1)) {
    pwires_p->forwardA = 2;  //Forward from EX/MEM pipe stage
    printf("[FWD]: Resolving EX hazard on rs1: x%d\n", pregs_p->idex_preg.out.RegisterRs1);
  }

  if(pregs_p->exmem_preg.out.RegWrite && (pregs_p->exmem_preg.out.RegisterRd != 0) && (pregs_p->exmem_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs2)) {
    pwires_p->forwardB = 2;  //Forward from EX/MEM pipe stage
    printf("[FWD]: Resolving EX hazard on rs2: x%d\n", pregs_p->idex_preg.out.RegisterRs2);
  }
  
  if(pregs_p->memwb_preg.out.RegWrite && (pregs_p->memwb_preg.out.RegisterRd != 0) && (pregs_p->memwb_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs1) && !(pregs_p->exmem_preg.out.RegWrite && (pregs_p->exmem_preg.out.RegisterRd != 0) && (pregs_p->exmem_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs1))) {
    pwires_p->forwardA = 1;  //Forward from MEM/WB pipe stage
    printf("[FWD]: Resolving MEM hazard on rs1: x%d\n", pregs_p->idex_preg.out.RegisterRs1);
  }
  
  if(pregs_p->memwb_preg.out.RegWrite && (pregs_p->memwb_preg.out.RegisterRd != 0) && (pregs_p->memwb_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs2) && !(pregs_p->exmem_preg.out.RegWrite && (pregs_p->exmem_preg.out.RegisterRd != 0) && (pregs_p->exmem_preg.out.RegisterRd == pregs_p->idex_preg.out.RegisterRs2))) {
    pwires_p->forwardB = 1;  //Forward from MEM/WB pipe stage
    printf("[FWD]: Resolving MEM hazard on rs2: x%d\n", pregs_p->idex_preg.out.RegisterRs2);
  }

}

/**
 * Task   : Sets the pipeline wires for the hazard unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
*/
void detect_hazard(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  ifid_reg_t ifid = pregs_p->ifid_preg.out;
  Instruction instruction = ifid.instr;
  Register ifid_rs1 = 0;
  Register ifid_rs2 = 0;

  idex_reg_t idex = pregs_p->idex_preg.out;

    switch(instruction.opcode){
    //R-type
    case 0x33:
      ifid_rs1 = instruction.rtype.rs1;
      ifid_rs2 = instruction.rtype.rs2;

      break;
    //Non-load I-type
    case 0x13:
      ifid_rs1 = instruction.itype.rs1;
      ifid_rs2 = 0;
 
      break;
    //Load I-type
    case 0x03:
      ifid_rs1 = instruction.itype.rs1;
      ifid_rs2 = 0;

      break;
    //S-type
    case 0x23:
      ifid_rs1 = instruction.stype.rs1;
      ifid_rs2 = instruction.stype.rs2;
      break;
    //B-type
    case 0x63:
      ifid_rs1 = instruction.sbtype.rs1;
      ifid_rs2 = instruction.sbtype.rs2;
      break;
    //lui 
    case 0x37:
      ifid_rs1 = 0;
      ifid_rs2 = 0;
  
      break;
    //jal
    case 0x6f:
      ifid_rs1 = 0;
      ifid_rs2 = 0;

      break;
    default:
      ifid_rs1 = 0;
      ifid_rs2 = 0;
      break;
  }
  
  if(idex.MemRead && ((idex.RegisterRd == ifid_rs1) || (idex.RegisterRd == ifid_rs2)) && ((ifid_rs1 != 0) || (ifid_rs2 != 0))){
    pwires_p->stall_id = 1;
    pwires_p->stall_if = 1;
    pwires_p->insert_bubble = 1;

    stall_counter++;

    //#ifdef DEBUG_HAZARD
    printf("[HZD]: Stalling and rewriting PC: 0x%08x\n", regfile_p->PC);//come back and change this
    //#endif
  }
  else{
    pwires_p->stall_id = 0;
    pwires_p->stall_if = 0;
    pwires_p->insert_bubble = 0;
  }
}

void flush(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p) {
  if(pwires_p->pcsrc == 1) {
    // Zero out IFID register
    pregs_p->ifid_preg.inp.instruction_bits = 0x00000013; // NOP
    pregs_p->ifid_preg.inp.instr = parse_instruction(0x00000013);

    // Zero out IDEX register
    pregs_p->idex_preg.inp.instruction_bits = 0x00000013; // NOP
    pregs_p->idex_preg.inp.instr = parse_instruction(0x00000013);
    pregs_p->idex_preg.inp.rs1_val = 0;
    pregs_p->idex_preg.inp.rs2_val = 0;
    pregs_p->idex_preg.inp.RegisterRs1 = 0;
    pregs_p->idex_preg.inp.RegisterRs2 = 0;
    pregs_p->idex_preg.inp.RegisterRd = 0;
    pregs_p->idex_preg.inp.imm = 0;
    pregs_p->idex_preg.inp.rd_address = 0;
    pregs_p->idex_preg.inp.ALUSrc = 0;
    pregs_p->idex_preg.inp.ALUcontrol = 0;
    pregs_p->idex_preg.inp.MemWrite = 0;
    pregs_p->idex_preg.inp.MemRead = 0;
    pregs_p->idex_preg.inp.MemtoReg = 0;
    pregs_p->idex_preg.inp.RegWrite = 0;
    pregs_p->idex_preg.inp.Branch = 0;

    // Zero out EXMEM register
    pregs_p->exmem_preg.inp.instruction_bits = 0x00000013; // NOP
    pregs_p->exmem_preg.inp.instr = parse_instruction(0x00000013);
    pregs_p->exmem_preg.inp.rs1_val = 0;
    pregs_p->exmem_preg.inp.rs2_val = 0;
    pregs_p->exmem_preg.inp.RegisterRs1 = 0;
    pregs_p->exmem_preg.inp.RegisterRs2 = 0;
    pregs_p->exmem_preg.inp.RegisterRd = 0;
    pregs_p->exmem_preg.inp.imm = 0;
    pregs_p->exmem_preg.inp.rd_address = 0;
    pregs_p->exmem_preg.inp.MemWrite = 0;
    pregs_p->exmem_preg.inp.MemRead = 0;
    pregs_p->exmem_preg.inp.MemtoReg = 0;
    pregs_p->exmem_preg.inp.RegWrite = 0;
    pregs_p->exmem_preg.inp.Branch = 0;
    pregs_p->exmem_preg.inp.isZero = 0;
    pregs_p->exmem_preg.inp.PC_Offset = 0;
    pregs_p->exmem_preg.inp.ALU_Result = 0;

    printf("[CPL]: Pipeline Flushed\n");
  }
  else {
    return;
  }
}

idex_reg_t no_op(idex_reg_t idex){

    idex_reg_t bubble = idex;
    //bubble.instruction_bits = 0x00000013; // NOP
    //bubble.instr = parse_instruction(0x00000013);
    //bubble.rs1_val = 0;
    //bubble.rs2_val = 0;
    //bubble.RegisterRs1 = 0;
    //bubble.RegisterRs2 = 0;
    //bubble.RegisterRd = 0;
    //bubble.imm = 0;
    //bubble.rd_address = 0;
    bubble.ALUSrc = 0;
    bubble.ALUcontrol = 0;
    bubble.MemWrite = 0;
    bubble.MemRead = 0;
    bubble.MemtoReg = 0;
    bubble.RegWrite = 0;
    bubble.Branch = 0;
    return bubble;
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
