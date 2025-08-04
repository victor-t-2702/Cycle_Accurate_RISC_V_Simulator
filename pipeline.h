#ifndef __PIPELINE_H__
#define __PIPELINE_H__

#include "config.h"
#include "types.h"
#include "cache.h"
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
/// Functionality
///////////////////////////////////////////////////////////////////////////////

extern simulator_config_t sim_config;
extern uint64_t miss_count;
extern uint64_t hit_count;
extern uint64_t total_cycle_counter;
extern uint64_t stall_counter;
extern uint64_t branch_counter;
extern uint64_t fwd_exex_counter;
extern uint64_t fwd_exmem_counter;
extern uint64_t mem_access_counter;

///////////////////////////////////////////////////////////////////////////////
/// RISC-V Pipeline Register Types
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  Instruction instr;
  uint32_t    instr_addr;

  //I added this
  uint32_t    instruction_bits;

}ifid_reg_t;

typedef struct
{
  Instruction instr;
  uint32_t    instr_addr;
  
  //Fields I added
  uint32_t    instruction_bits;
  Register rs1_val;
  Register rs2_val;
  uint32_t imm;
  Register rd_address;
  
  bool ALUSrc;
  uint32_t ALUcontrol;
  bool MemWrite;
  bool MemRead;
  bool MemtoReg;
  bool RegWrite;
  bool Branch;
}idex_reg_t;

typedef struct
{
  Instruction instr;
  uint32_t    instr_addr;
  
  uint32_t    instruction_bits;
  Register rs1_val;
  Register rs2_val;
  Register rd_address;
  uint32_t imm;

  bool MemWrite;
  bool MemRead;
  bool MemtoReg;
  bool RegWrite;
  bool Branch;
  bool isZero; // Zero checking for BRANCH
  uint32_t PC_Offset;
  uint32_t ALU_Result;
}exmem_reg_t;

typedef struct
{
  Instruction instr;
  uint32_t    instr_addr;
  
  //Fields I added
  uint32_t    instruction_bits;
  uint32_t wb_v;
  Register rs1_val;
  Register rs2_val;
  Register rd_address;
  uint32_t imm;

  bool MemWrite;
  bool MemRead;
  bool MemtoReg;
  bool RegWrite;
  bool Branch;
  bool isZero; // Zero checking for BRANCH
  uint32_t PC_Offset;
  uint32_t ALU_Result;
}memwb_reg_t;


///////////////////////////////////////////////////////////////////////////////
/// Register types with input and output variants for simulator
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  ifid_reg_t inp;
  ifid_reg_t out;
}ifid_reg_pair_t;

typedef struct
{
  idex_reg_t inp;
  idex_reg_t out;
}idex_reg_pair_t;

typedef struct
{
  exmem_reg_t inp;
  exmem_reg_t out;
}exmem_reg_pair_t;

typedef struct
{
  memwb_reg_t inp;
  memwb_reg_t out;
}memwb_reg_pair_t;

///////////////////////////////////////////////////////////////////////////////
/// Functional pipeline requirements
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  ifid_reg_pair_t  ifid_preg;
  idex_reg_pair_t  idex_preg;
  exmem_reg_pair_t exmem_preg;
  memwb_reg_pair_t memwb_preg;
}pipeline_regs_t;

typedef struct
{
  bool      pcsrc;
  uint32_t  pc_src0;
  uint32_t  pc_src1;
  /**
   * Add other fields here
   */
}pipeline_wires_t;


///////////////////////////////////////////////////////////////////////////////
/// Function definitions for different stages
///////////////////////////////////////////////////////////////////////////////

/**
 * output : ifid_reg_t
 **/ 
ifid_reg_t stage_fetch(pipeline_wires_t* pwires_p, regfile_t* regfile_p, Byte* memory_p);

/**
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p);

/**
 * output : exmem_reg_t
 **/ 
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p);

/**
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory, Cache* cache_p);

/**
 * output : write_data
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p);

void cycle_pipeline(regfile_t* regfile_p, Byte* memory_p, Cache* cache_p, pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, bool* ecall_exit);

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p);

#endif  // __PIPELINE_H__;
