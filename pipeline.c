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
uint64_t mem_access_counter = 0;

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
  ifid_reg.instr_addr = PC;
  ifid_reg.instr = instruction;
  ifid_reg.instruction_bits = instruction_bits;

  //Increments PC by 4, store it in the wires for pc_src0
  pwires_p->pc_src0 = PC + 4;

  if(pwires_p->pcsrc == 1){
    regfile_p->PC = pwires_p->pc_src1;
  }
  else{
    regfile_p->PC = pwires_p->pc_src0;
  }
  
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
  //Read the PC and the instruction from ifid_reg
  uint32_t PC = ifid_reg.instr_addr;
  Instruction instruction = ifid_reg.instr;

  //Generate all the control signals
  idex_reg_t idex_reg = gen_control(instruction);

  //Generate the immediate and initialize the registers
  uint32_t imm = gen_imm(instruction);
  Register rs1 = 0;
  Register rs2 = 0;
  Register rd = 0;
  uint32_t ALU_control = 0;

  //I based these commands off our stuff from lab 2 and 3, which for some reason has different commands
  //From the green card he gave, idk if this is done right
  //Extract the rs1 and rs2 addresses based on the instruction type
  switch(instruction.opcode){
    //R-type
    case 0x33:
      rs1 = instruction.rtype.rs1;
      rs2 = instruction.rtype.rs2;
      rd = instruction.rtype.rd;
      break;
    //Non-load I-type
    case 0x13:
      rs1 = instruction.itype.rs1;
      rs2 = 0;
      rd = instruction.itype.rd;
      break;
    //Load I-type
    case 0x03:
      rs1 = instruction.itype.rs1;
      rs2 = 0;
      rd = instruction.itype.rd;
      break;
    //S-type
    case 0x23:
      rs1 = instruction.stype.rs1;
      rs2 = instruction.stype.rs2;
      break;
    //B-type
    case 0x63:
      rs1 = instruction.sbtype.rs1;
      rs2 = instruction.sbtype.rs2;
      break;
    //lui 
    case 0x37:
      rs1 = 0;
      rs2 = 0;
      rd = instruction.utype.rd;
      break;
    //jal
    case 0x6f:
      rs1 = 0;
      rs2 = 0;
      rd = instruction.ujtype.rd;
      break;
    default:
      rs1 = 0;
      rs2 = 0;
      break;
  }

  uint32_t instruction_bits = ifid_reg.instruction_bits;
  #ifdef DEBUG_CYCLE
  printf("[ID ]: Instruction [%08x]@[%08x]: ", instruction_bits, PC);
  decode_instruction(instruction_bits);
  #endif
  

  //Read the values of rs1 and rs2
  int32_t rs1_val = regfile_p->R[rs1];
  int32_t rs2_val = regfile_p->R[rs2];

  //For hazard detection
  idex_reg.RegisterRs1 = rs1;
  idex_reg.RegisterRs2 = rs2;
  idex_reg.RegisterRd = rd;

  //Put the instruction, PC, generated immediate, rs1 and rs2 values, rd address, and the ALU control command into the idex register
  idex_reg.instruction_bits = instruction_bits;
  idex_reg.instr = instruction;
  idex_reg.instr_addr = PC;
  idex_reg.imm = imm;
  idex_reg.rs1_val = rs1_val;
  idex_reg.rs2_val = rs2_val;
  idex_reg.rd_address = rd;
  idex_reg.ALUcontrol = gen_alu_control(idex_reg);
  return idex_reg;
}

/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/ 
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p)
{
  exmem_reg_t exmem_reg = {0};

 //For hazard detection
  exmem_reg.RegisterRs1 = idex_reg.RegisterRs1;
  exmem_reg.RegisterRs2 = idex_reg.RegisterRs2;
  exmem_reg.RegisterRd = idex_reg.RegisterRd;

  Instruction instruction = idex_reg.instr;
  uint32_t PC = idex_reg.instr_addr;
  uint32_t rs1_val = idex_reg.rs1_val;
  uint32_t rs2_val = idex_reg.rs2_val;
  Register rd_address = idex_reg.rd_address;
  bool ALUSrc = idex_reg.ALUSrc;
  uint32_t ALUcontrol = idex_reg.ALUcontrol;
  uint32_t imm = idex_reg.imm;

  exmem_reg.PC_Offset = imm + PC; // Get the sum of the immediate and the PC (at the time of FETCH) and put in exmem register 

  uint32_t alu_inp1 = 0;
  uint32_t alu_inp2 = 0;

// MUX to determine whether to use RS1 for ALU operation (or forward)
  if(pwires_p->forwardA == 0) { // No forwarding  
    alu_inp1 = rs1_val;     
  }
  else if(pwires_p->forwardA == 2) { // Forwarding 
    alu_inp1 = pwires_p->exmem_reg_ALU_Result;   // The first ALU operand is forwarded from the prior ALU result.  (Source: EX/MEM)
  }
  else if(pwires_p->forwardA == 1) { // Forwarding 
    alu_inp1 = pwires_p->memwb_reg_wb_v;   // The first ALU operand is forwarded from data memory or an earlier ALU result. (Source: MEM/WB)
  }

  // MUX to determine whether to use IMM or RS2 for ALU operation (or forward)
  if(ALUSrc == 0 && pwires_p->forwardB == 0) { // No forwarding  
    alu_inp2 = rs2_val;     
  }
  else if(ALUSrc == 0 && pwires_p->forwardB == 2) { // Forwarding 
    alu_inp2 = pwires_p->exmem_reg_ALU_Result;   // The second ALU operand is forwarded from the prior ALU result    (Source: EX/MEM)
  }
  else if(ALUSrc == 0 && pwires_p->forwardB == 1) { // Forwarding 
    alu_inp2 = pwires_p->memwb_reg_wb_v;   // The second ALU operand is forwarded from data memory or an earlier ALU result. (Source: MEM/WB)
  }
  else {   // Use IMM
    alu_inp2 = imm;
  }

  int32_t instruction_bits = idex_reg.instruction_bits;
  #ifdef DEBUG_CYCLE
  printf("[EX ]: Instruction [%08x]@[%08x]: ", instruction_bits, PC);
  decode_instruction(instruction_bits);
  #endif
  exmem_reg.instruction_bits = instruction_bits;
  exmem_reg.instr_addr = PC;

  if(instruction.opcode == 0x37){ //Special case for LUI command
    exmem_reg.ALU_Result = imm << 12;
  }
  else{
    exmem_reg.ALU_Result = execute_alu(alu_inp1, alu_inp2, ALUcontrol);  // Calculate ALU result and put in exmem register
  }

  pwires_p->exmem_reg_ALU_Result = exmem_reg.ALU_Result;   // For forwarding

  if((exmem_reg.ALU_Result == 0) && (instruction.opcode == 0x63)){//Only need to evaluate branch, because JAL does not need isZero, it always jumps
    exmem_reg.isZero = 1;
  }
  else{
    exmem_reg.isZero = 0;
  }
  //exmem_reg.isZero = (exmem_reg.ALU_Result == 0)?1:0;  // Zero checking signal for BRANCH

  exmem_reg.Branch = idex_reg.Branch;
  exmem_reg.RegWrite = idex_reg.RegWrite;
  exmem_reg.MemtoReg = idex_reg.MemtoReg;
  exmem_reg.MemRead = idex_reg.MemRead;
  exmem_reg.MemWrite = idex_reg.MemWrite;  // Move signals along from ID/EX register

  exmem_reg.rs1_val = rs1_val;
  exmem_reg.rs2_val = rs2_val;
  exmem_reg.instr = instruction;
  exmem_reg.imm = imm;
  exmem_reg.rd_address = rd_address;



  /*
  bool take_branch = gen_branch(exmem_reg);
  pwires_p->pcsrc = take_branch;
  pwires_p->pc_src1 = exmem_reg.PC_Offset;
  */

  return exmem_reg;
}

/**
 * STAGE  : stage_mem
 * output : memwb_reg_t
 **/ 
//For load read the exmem register rs1 + imm, write memwb rs1 + imm
//For store read the exmem register rs1 + imm and rs2, no need to write anything
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory_p, Cache* cache_p)
{
  memwb_reg_t memwb_reg = {0};

  // Branch decision
  bool take_branch = gen_branch(exmem_reg);
  pwires_p->pcsrc = take_branch;
  pwires_p->pc_src1 = exmem_reg.PC_Offset;

  //For hazard detection
  memwb_reg.RegisterRs1 = exmem_reg.RegisterRs1;
  memwb_reg.RegisterRs2 = exmem_reg.RegisterRs2;
  memwb_reg.RegisterRd = exmem_reg.RegisterRd;

  Instruction instruction = exmem_reg.instr;
  uint32_t PC = exmem_reg.instr_addr;
  uint32_t rs1_val = exmem_reg.rs1_val;
  uint32_t rs2_val = exmem_reg.rs2_val;
  uint32_t ALU_Result = exmem_reg.ALU_Result;  
  Register rd_address = exmem_reg.rd_address;
  uint32_t imm = exmem_reg.imm;

  bool RegWrite = exmem_reg.RegWrite;

  uint32_t load_val = 0;

  //For loading a value from memory
  if(exmem_reg.MemRead == 1){
    //No need to exclude non-load I-type instructions here, as they would not have memread = 1
    switch(instruction.itype.funct3){
      case 0x0: //Load byte
        load_val = sign_extend_number(memory_p[ALU_Result], 8);
        break;
      case 0x1: //Load half word
        load_val = sign_extend_number((memory_p[ALU_Result] | (memory_p[ALU_Result + 1] << 8)), 16);
        break;
      case 0x2: //Load word
        load_val = memory_p[ALU_Result] | (memory_p[ALU_Result + 1] << 8) | (memory_p[ALU_Result + 2] << 16) | (memory_p[ALU_Result + 3] << 24);
        break;
      default:
        break;
     }
   }

    //For storing a value into memory
  if(exmem_reg.MemWrite == 1){
     switch(instruction.stype.funct3){
       case 0x0: //Store byte
         memory_p[ALU_Result] = rs2_val & 0xFF;
         break;
       case 0x1: //Store half word
         memory_p[ALU_Result] = rs2_val & 0xFF;
         memory_p[ALU_Result + 1] = (rs2_val >> 8) & 0xFF;
         break;
       case 0x2: //Store word
         memory_p[ALU_Result] = rs2_val & 0xFF;
         memory_p[ALU_Result + 1] = (rs2_val >> 8) & 0xFF;
         memory_p[ALU_Result + 2] = (rs2_val >> 16) & 0xFF;
         memory_p[ALU_Result + 3] = (rs2_val >> 24) & 0xFF;
         break;
       default:
         break;
     }
   }

  int32_t instruction_bits = exmem_reg.instruction_bits;
  #ifdef DEBUG_CYCLE
  printf("[MEM]: Instruction [%08x]@[%08x]: ", instruction_bits, PC);
  decode_instruction(instruction_bits);
  #endif
  memwb_reg.instruction_bits = instruction_bits;
  memwb_reg.instr_addr = PC;
  

  memwb_reg.instr = instruction;
  memwb_reg.imm = imm;
  memwb_reg.rd_address = rd_address;
  memwb_reg.RegWrite = RegWrite;
  
  if(exmem_reg.MemtoReg == 1){
    memwb_reg.wb_v = load_val;
  }
  else{
    memwb_reg.wb_v = ALU_Result;
  }

  pwires_p->memwb_reg_wb_v = memwb_reg.wb_v;   // For forwarding

  return memwb_reg;
}

/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  if((memwb_reg.RegWrite == 1) && (memwb_reg.rd_address != 0)){
    regfile_p->R[memwb_reg.rd_address] = memwb_reg.wb_v;
  }
  int32_t PC = memwb_reg.instr_addr;
  int32_t instruction_bits = memwb_reg.instruction_bits;
  #ifdef DEBUG_CYCLE
  printf("[WB ]: Instruction [%08x]@[%08x]: ", instruction_bits, PC);
  decode_instruction(instruction_bits);
  #endif
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

                            gen_forward     (pregs_p, pwires_p);  // Stage that generates forwarding signals

  pregs_p->exmem_preg.inp = stage_execute   (pregs_p->idex_preg.out, pwires_p); 

  pregs_p->memwb_preg.inp = stage_mem       (pregs_p->exmem_preg.out, pwires_p, memory_p, cache_p);

                            stage_writeback (pregs_p->memwb_preg.out, pwires_p, regfile_p);

                            flush           (pregs_p, pwires_p);  // Stage that flushes IFID, IDEX, and EXMEM registers if PCSrc == 1 (i.e. branch is taken)

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

