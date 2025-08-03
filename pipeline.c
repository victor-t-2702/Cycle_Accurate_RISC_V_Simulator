#include <stdbool.h>
#include "cache.h"
#include "riscv.h"
#include "types.h"
#include "utils.h"
#include "pipeline.h"
#include "stage_helpers.h"

uint64_t total_cycle_counter = 0;
uint64_t miss_count = 0;
uint64_t hit_count = 0;
uint64_t stall_counter = 0;
uint64_t branch_counter = 0;
uint64_t fwd_exex_counter = 0;
uint64_t fwd_exmem_counter = 0;

simulator_config_t sim_config = {0};

///////////////////////////////////////////////////////////////////////////////

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p)
{
  // PC src must get the same value as the default PC value
  pwires_p->pc_src0 = regfile_p->PC;
}

///////////////////////////
/// STAGE FUNCTIONALITY ///
///////////////////////////

/**
 * STAGE  : stage_fetch
 * output : ifid_reg_t
 **/ 
//This stage reads memory using PC for instruction, then writes the instruction and PC to register
//Im not actually sure if its meant to read the instruction bits or the instruction itself
ifid_reg_t stage_fetch(pipeline_wires_t* pwires_p, regfile_t* regfile_p, Byte* memory_p)
{
  //Initialize the ifid_reg
  ifid_reg_t ifid_reg = {0};

  //Using the PC as base address, extract the 32 bits of instruction
  uint32_t PC = regfile_p->PC;
  uint32_t instruction_bits = (memory_p[PC]) | (memory_p[PC+1] << 8) | (memory_p[PC+2] << 16) | (memory_p[PC+3] << 24);
  
  //Parse the instruction bits
  Instruction instruction = parse_instruction(instruction_bits);

  #ifdef DEBUG_CYCLE
  printf("[IF ]: Instruction [%08x]@[%08x]: ", instruction_bits, regfile_p->PC);
  decode_instruction(instruction_bits);
  #endif

  //Fill in the values of instruction address and instruction for ifid_reg
  ifid_reg.instr_addr = regfile_p->PC;
  ifid_reg.instr = instruction;

  //Increments PC by 4, im not sure if this is meant to be here because we also have
  //execute ____type that increments PC, so idk
  regfile_p->PC += 4;

  return ifid_reg;
}

/**
 * STAGE  : stage_decode
 * output : idex_reg_t
 **/ 
//This stage reads the ifid_reg for instruction and PC
//Then it writes to idex_reg rs1, rs2, sign extended imm, PC, and generates all the signals?
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  idex_reg_t idex_reg = {0};

  uint32_t PC = ifid_reg.instr_addr;
  Instruction instruction = ifid_reg.instr;
  uint32_t imm;
  Register rs1;
  Register rs2;

  //I based these commands off our stuff from lab 2 and 3, which for some reason has different commands
  //From the green card he gave, idk if this is done right
  switch(instruction.opcode){
    //R-type
    case 0x33:
      imm = 0;
      rs1 = instruction.rtype.rs1;
      rs2 = instruction.rtype.rs2;
    //Non-load I-type
    case 0x13:
      imm = sign_extend_number(instruction.itype.imm, 12);
      rs1 = instruction.itype.rs1;
      rs2 = 0;
    //Load I-type
    case 0x03:
      imm = sign_extend_number(instruction.itype.imm, 12);
      rs1 = instruction.itype.rs1;
      rs2 = 0;
    //S-type
    case 0x23:
      imm = get_store_offset(instruction);
      rs1 = instruction.stype.rs1;
      rs2 = instruction.stype.rs2;
    //B-type
    case 0x63:
      imm = get_branch_offset(instruction);
      rs1 = instruction.sbtype.rs1;
      rs2 = instruction.sbtype.rs2;
    //lui 
    case 0x37:
      //I dunnoe if i sign extend or nah
      imm = instruction.utype.imm;
      rs1 = 0;
      rs2 = 0;
    //jal
    case 0x6f:
      imm = get_jump_offset(instruction);
      rs1 = 0;
      rs2 = 0;
    default:
      imm = 0;
      rs1 = 0;
      rs2 = 0;
  }

  int rs1_val = regfile_p->R[rs1];
  int rs2_val = regfile_p->R[rs2];
  idex_reg.instr = instruction;
  idex_reg.instr_addr = PC;
  idex_reg.imm = imm;
  idex_reg.rs1_val = rs1_val;
  idex_reg.rs2_val = rs2_val;
  return idex_reg;
}

//I think this entire function is actually meant to go in stage_helpers, and repalce part of the stage_decode function above
//Ill fix this later
uint32_t get_imm(Instruction instruction){
  //Calculate offset depending on instruction type
  switch(instruction.opcode){
    //R-type
    case 0x33:
      return 0;
    
    //Non-load I-type
    case 0x13:
      return sign_extend_number(instruction.itype.imm, 12);

    //Load I-type
    case 0x03:
      return sign_extend_number(instruction.itype.imm, 12);

    //S-type
    case 0x23:
      return get_store_offset(instruction);
    
    //B-type
    case 0x63:
      return get_branch_offset(instruction);

    //lui 
    case 0x37:
      //I dunnoe if i sign extend or nah
      return instruction.utype.imm;

    //jal
    case 0x6f:
      return get_jump_offset(instruction);

    default:
      return 0;
  }
    
}

/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/ 
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p)
{
  exmem_reg_t exmem_reg = {0};
  /**
   * YOUR CODE HERE
   */
  return exmem_reg;
}

/**
 * STAGE  : stage_mem
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory_p, Cache* cache_p)
{
  memwb_reg_t memwb_reg = {0};
  /**
   * YOUR CODE HERE
   */
  return memwb_reg;
}

/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  /**
   * YOUR CODE HERE
   */
}

///////////////////////////////////////////////////////////////////////////////

/** 
 * excite the pipeline with one clock cycle
 **/
void cycle_pipeline(regfile_t* regfile_p, Byte* memory_p, Cache* cache_p, pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, bool* ecall_exit)
{
  #ifdef DEBUG_CYCLE
  printf("v==============");
  printf("Cycle Counter = %5ld", total_cycle_counter);
  printf("==============v\n\n");
  #endif

  // process each stage

  /* Output               |    Stage      |       Inputs  */
  pregs_p->ifid_preg.inp  = stage_fetch     (pwires_p, regfile_p, memory_p);
  
  pregs_p->idex_preg.inp  = stage_decode    (pregs_p->ifid_preg.out, pwires_p, regfile_p);

  pregs_p->exmem_preg.inp = stage_execute   (pregs_p->idex_preg.out, pwires_p);

  pregs_p->memwb_preg.inp = stage_mem       (pregs_p->exmem_preg.out, pwires_p, memory_p, cache_p);

                            stage_writeback (pregs_p->memwb_preg.out, pwires_p, regfile_p);

  // update all the output registers for the next cycle from the input registers in the current cycle
  pregs_p->ifid_preg.out  = pregs_p->ifid_preg.inp;
  pregs_p->idex_preg.out  = pregs_p->idex_preg.inp;
  pregs_p->exmem_preg.out = pregs_p->exmem_preg.inp;
  pregs_p->memwb_preg.out = pregs_p->memwb_preg.inp;

  /////////////////// NO CHANGES BELOW THIS ARE REQUIRED //////////////////////

  // increment the cycle
  total_cycle_counter++;

  #ifdef DEBUG_REG_TRACE
  print_register_trace(regfile_p);
  #endif

  /**
   * check ecall condition
   * To do this, the value stored in R[10] (a0 or x10) should be 10.
   * Hence, the ecall condition is checked by the existence of following
   * two instructions in sequence:
   * 1. <instr>  x10, <val1>, <val2> 
   * 2. ecall
   * 
   * The first instruction must write the value 10 to x10.
   * The second instruction is the ecall (opcode: 0x73)
   * 
   * The condition checks whether the R[10] value is 10 when the
   * `memwb_reg.instr.opcode` == 0x73 (to propagate the ecall)
   * 
   * If more functionality on ecall needs to be added, it can be done
   * by adding more conditions on the value of R[10]
   */
  if( (pregs_p->memwb_preg.out.instr.bits == 0x00000073) &&
      (regfile_p->R[10] == 10) )
  {
    *(ecall_exit) = true;
  }
}

