# rimi-manipulation
Playing with the documentation given at: https://gist.github.com/ibndias/c410c27e5eb1c3a9724051bc0353b5dd


# RIMI Documentation

This is dev environment setup for RIMI.

Optional: We use VS Code (https://code.visualstudio.com/download) for the IDE.

## RISCV GCC Toolchain Setup

Get the riscv toolchain, clone from (https://github.com/riscv/riscv-gnu-toolchain) 

    $ git clone https://github.com/riscv/riscv-gnu-toolchain 

Do recursive update for each submodule 

    $ cd riscv-gnu-toolchain 
    $ git submodule update --init --recursive 

Install the tools needed for building the toolchain: 

    $ sudo apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev 

Build for GCC 32bit environment, then add /opt/riscv/bin to PATH. 

    $ ./configure --prefix=/opt/riscv --with-arch=rv32g --with-abi=ilp32d 
    $ make 
    $ export PATH=$PATH:/opt/riscv/bin 

## RISCV Tools Environment Setup

Get the riscv tools, clone the riscv-tool repository at (https://github.com/riscv/riscv-tools). 

    $ git clone https://github.com/riscv/riscv-tools
    $ cd riscv-tools 

Copy the `build-rv32ima.sh` as `build-rv32g.sh`.

    $ cp build-rv32ima.sh build-rv32g.sh 

We need to set Spike to use default ISA as RV32G. Open `build-rv32g.sh` then replace the line

    build_project riscv-isa-sim --prefix=$RISCV --with-isa=rv32ima 

with

    build_project riscv-isa-sim --prefix=$RISCV --with-isa=rv32g 

Now initialize the submodules

    $ git submodule update --init --recursive
    $ export RISCV=/opt/riscv/bin
    $ ./build-rv32g.sh

Now we have working GCC, Spike and PK. To test it, try compile a simple hello world program.

    $ riscv32-unknown-elf-gcc hello.c

Now run using Spike on top of PK.

    $ spike pk a.out

This should be working by now.

## Creating Custom Instruction / Spike Modification

Inside riscv-tools repository, we have riscv-opcodes which able to generate our opcode mask.
Open the riscv-opcodes/opcodes file. We assign our instruction inside this file as

    slw      rd rs1       imm12 14..12=7 6..2=0x00 1..0=3
    ssw     imm12hi rs1 rs2 imm12lo 14..12=7 6..2=0x08 1..0=3

The we generate the mask for these instruction using

    $ cat opcodes-pseudo opcodes opcodes-rvc opcodes-rvc-pseudo opcodes-custom | ./parse-opcodes -c > ~/temp.h

Now inside ~/temp.h we have our MATCH and MASK for our custom instruction.

    #define MATCH_SLW 0x7003
    #define MASK_SLW 0x707F
    #define MATCH_SSW 0x7023
    #define MASK_SSW 0x707F

We add these definition on riscv-isa-sim/riscv/encoding.h.
And also duplicating the lw.h and sw.h inside riscv-isa-sim/riscv/insns as slw.h and ssw.h.

We create new access type for our instruction at memtracer.h

    enum access_type {
      LOAD,
      STORE,
      FETCH,
      S_LOAD,
      S_STORE,
      S_FETCH
    };

...

More detailed version can be explain by **jinjae**. Or use the his patch.


## Modifying The Proxy Kernel

First, add the CSR encoding into `riscv-pk/machine/encoding.h`

    #define CSR_SETISOLATION 0x404
    ...
    DECLARE_CSR(setisolation,CSR_SETISOLATION)

Modify the `setup_pmp()` to match our region.
Also, we set our CSR value here.

    void setup_pmp(void)
    {
    		uintptr_t pmpc = PMP_TOR | PMP_R | PMP_W | PMP_X;
    		pmpc |= ((pmpc << 8) | (pmpc << 16));
    	 
    	asm volatile ("la t0, 1f\n\t"
    	"csrrw t0, mtvec, t0\n\t"
    	"csrw pmpaddr0, %1\n\t"
    	"csrw pmpcfg0, %0\n\t"
    	"csrw pmpaddr1, %2\n\t"
    	"csrw pmpaddr2, %3\n\t"
    	".align 2\n\t"
    	"1: csrw mtvec, t0"
    	: : "r" (pmpc), "r" (0x8041B000>>2), "r" (0xFFC00000>>2), "r"(0xFFFFFFFF) : "t0");
    	write_csr(0x404,0x00000004);
    
    }

Modify the `pk_vm_init()` to allocate space for shadow stack

    uintptr_t pk_vm_init()
    {
      // HTIF address signedness and va2pa macro both cap memory size to 2 GiB
      mem_size = MIN(mem_size, 1U << 31);
      size_t mem_pages = mem_size >> RISCV_PGSHIFT;
      free_pages = MAX(8, mem_pages >> (RISCV_PGLEVEL_BITS-1));
    
      extern char _end;
      first_free_page = ROUNDUP((uintptr_t)&_end, RISCV_PGSIZE);
      first_free_paddr = first_free_page + free_pages * RISCV_PGSIZE;
    
      root_page_table = (void*)__page_alloc();
      __map_kernel_range(DRAM_BASE, DRAM_BASE, first_free_paddr - DRAM_BASE, PROT_READ|PROT_WRITE|PROT_EXEC);
    
    
      current.mmap_max = current.brk_max = 
     MIN(DRAM_BASE, mem_size - (first_free_paddr - DRAM_BASE)); //
     //7fbe5000
     
     
     size_t stack_size = MIN(mem_pages >> 5, 2048) * RISCV_PGSIZE;
     
     
     size_t stack_bottom = __do_mmap(current.mmap_max - stack_size, stack_size, PROT_READ|PROT_WRITE|PROT_EXEC, MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED, 0, 0);
     kassert(stack_bottom != (uintptr_t)-1);
     
     //half of stack is shadow stack, this will split stack into 2 parts
     size_t shadow_stack_size = stack_size >> 1; 
     size_t shadow_stack_top = stack_bottom + stack_size;
     size_t shadow_stack_bottom = shadow_stack_top - shadow_stack_size;
     
     current.stack_top = stack_bottom + stack_size - shadow_stack_size;
     //current.stack_top = stack_bottom + stack_size;
     
     flush_tlb();
     write_csr(sptbr, ((uintptr_t)root_page_table >> RISCV_PGSHIFT) | SATP_MODE_CHOICE);
    
      uintptr_t kernel_stack_top = __page_alloc() + RISCV_PGSIZE;
      return kernel_stack_top;
    }

Adding supervisor interface to machine trap handler

    void mcall_trap(uintptr_t* regs, uintptr_t mcause, uintptr_t mepc)
    {
    ...
        case SBI_CHANGE_DOMAIN:
          retval = mcall_change_domain(arg0);
          break;
    ...
    }

And we put this custom function interface inside `mtrap.c`

    static uintptr_t mcall_change_domain(uint32_t value)
    {	
    	//Change the value of our CSR.
    	write_csr(0x404, value);
    }

While in supervisor mode, before entering the user mode,
we call our custom interface to change region.

    static void rest_of_boot_loader(uintptr_t kstack_top)
    {
      ...
      load_elf(args.argv[0], &current);
    
      //Call the our interface, change the CSR value to 2
      asm("li a7, 9");
      asm("li a0, 2");
      asm("ecall");
      
      run_loaded_program(argc, args.argv, kstack_top);
    }
Then rebuild the PK using 

    $ ./build-rv32g.sh

## Setup LLVM

Install all the required tools

```
$ sudo apt-get -y install \
  binutils build-essential libtool texinfo \
  gzip zip unzip patchutils curl git \
  make cmake ninja-build automake bison flex gperf \
  grep sed gawk python bc \
  zlib1g-dev libexpat1-dev libmpc-dev \
  libglib2.0-dev libfdt-dev libpixman-1-dev 
```
Clone the riscv-llvm
```
$ git clone https://github.com/llvm/llvm-project.git riscv-llvm
```
Then build the 32 bit riscv-llvm 
```
$ pushd riscv-llvm
$ ln -s ../../clang llvm/tools || true
$ mkdir _build
$ cd _build
$ cmake -G Ninja -DCMAKE_BUILD_TYPE="Release" -DLLVM_ENABLE_PROJECTS=clang -DBUILD_SHARED_LIBS=True -DLLVM_USE_SPLIT_DWARF=True -DCMAKE_INSTALL_PREFIX="/opt/riscv" -DLLVM_OPTIMIZED_TABLEGEN=True -DLLVM_BUILD_TESTS=False -DDEFAULT_SYSROOT="/opt/riscv/riscv32-unknown-elf" -DLLVM_DEFAULT_TARGET_TRIPLE="riscv32-unknown-elf" -DLLVM_TARGETS_TO_BUILD="RISCV" ../llvm
$ cmake --build . --target install
$ popd
```

## Modifying The LLVM

Adding lw1 and sw1 in .../llvm/lib/Target/RISCV/RISCVISelDAGToDAG.cpp

       void RISCVDAGToDAGISel::doPeepholeLoadStoreADDI() {
       ...
           // Only attempt this optimisation for I-type loads and S-type stores.
           switch (N->getMachineOpcode()) {
           default:
             continue;
           case RISCV::LB:
           case RISCV::LH:
           case RISCV::LW:
           case RISCV::LBU:
           case RISCV::LHU:
           case RISCV::LWU:
           case RISCV::SLW:
           case RISCV::LD:
           case RISCV::FLW:
           case RISCV::FLD:
             BaseOpIdx = 0;
             OffsetOpIdx = 1;
             break;
           case RISCV::SB:
           case RISCV::SH:
           case RISCV::SW:
           case RISCV::SSW:
       ...
       }

Adding lw1 and sw1 in .../llvm/lib/Target/RISCV/RISCVInstrInfo.td

       ...
       def LB  : Load_ri<0b000, "lb">, Sched<[WriteLDB, ReadMemBase]>;
       def LH  : Load_ri<0b001, "lh">, Sched<[WriteLDH, ReadMemBase]>;
       def LW  : Load_ri<0b010, "lw">, Sched<[WriteLDW, ReadMemBase]>;
       def SLW  : Load_ri<0b111, "slw">, Sched<[WriteLDW, ReadMemBase]>;
       def LBU : Load_ri<0b100, "lbu">, Sched<[WriteLDB, ReadMemBase]>;
       def LHU : Load_ri<0b101, "lhu">, Sched<[WriteLDH, ReadMemBase]>;
    
       def SB : Store_rri<0b000, "sb">, Sched<[WriteSTB, ReadStoreData, ReadMemBase]>;
       def SH : Store_rri<0b001, "sh">, Sched<[WriteSTH, ReadStoreData, ReadMemBase]>;
       def SW : Store_rri<0b010, "sw">, Sched<[WriteSTW, ReadStoreData, ReadMemBase]>;
       def SSW : Store_rri<0b111, "ssw">, Sched<[WriteSTW, ReadStoreData, ReadMemBase]>;
       ...
       //===----------------------------------------------------------------------===//
       // Assembler Pseudo Instructions (User-Level ISA, Version 2.2, Chapter 20)
       //===----------------------------------------------------------------------===//
       ...
       def PseudoLB  : PseudoLoad<"lb">;
       def PseudoLBU : PseudoLoad<"lbu">;
       def PseudoLH  : PseudoLoad<"lh">;
       def PseudoLHU : PseudoLoad<"lhu">;
       def PseudoLW  : PseudoLoad<"lw">;
       def PseudoSLW  : PseudoLoad<"slw">;
    
       def PseudoSB  : PseudoStore<"sb">;
       def PseudoSH  : PseudoStore<"sh">;
       def PseudoSW  : PseudoStore<"sw">;
       def PseudoSSW  : PseudoStore<"ssw">;
       ...
       let EmitPriority = 0 in {
       def : InstAlias<"lb $rd, (${rs1})",
                       (LB  GPR:$rd, GPR:$rs1, 0)>;
       def : InstAlias<"lh $rd, (${rs1})",
                       (LH  GPR:$rd, GPR:$rs1, 0)>;
       def : InstAlias<"lw $rd, (${rs1})",
                       (LW  GPR:$rd, GPR:$rs1, 0)>;
       def : InstAlias<"slw $rd, (${rs1})",
                       (SLW  GPR:$rd, GPR:$rs1, 0)>;
       def : InstAlias<"lbu $rd, (${rs1})",
                       (LBU  GPR:$rd, GPR:$rs1, 0)>;
       def : InstAlias<"lhu $rd, (${rs1})",
                       (LHU  GPR:$rd, GPR:$rs1, 0)>;
    
       def : InstAlias<"sb $rs2, (${rs1})",
                       (SB  GPR:$rs2, GPR:$rs1, 0)>;
       def : InstAlias<"sh $rs2, (${rs1})",
                       (SH  GPR:$rs2, GPR:$rs1, 0)>;
       def : InstAlias<"sw $rs2, (${rs1})",
                       (SW  GPR:$rs2, GPR:$rs1, 0)>;
       def : InstAlias<"ssw $rs2, (${rs1})",
                       (SSW  GPR:$rs2, GPR:$rs1, 0)>;
       ...
       defm : LdPat<sextloadi8, LB>;
       defm : LdPat<extloadi8, LB>;
       defm : LdPat<sextloadi16, LH>;
       defm : LdPat<extloadi16, LH>;
       defm : LdPat<load, LW>, Requires<[IsRV32]>;
       defm : LdPat<load, SLW>, Requires<[IsRV32]>;
       defm : LdPat<zextloadi8, LBU>;
       defm : LdPat<zextloadi16, LHU>;
    
       /// Stores
    
       multiclass StPat<PatFrag StoreOp, RVInst Inst, RegisterClass StTy> {
         def : Pat<(StoreOp StTy:$rs2, GPR:$rs1), (Inst StTy:$rs2, GPR:$rs1, 0)>;
         def : Pat<(StoreOp StTy:$rs2, AddrFI:$rs1), (Inst StTy:$rs2, AddrFI:$rs1, 0)>;
         def : Pat<(StoreOp StTy:$rs2, (add GPR:$rs1, simm12:$imm12)),
                   (Inst StTy:$rs2, GPR:$rs1, simm12:$imm12)>;
         def : Pat<(StoreOp StTy:$rs2, (add AddrFI:$rs1, simm12:$imm12)),
                   (Inst StTy:$rs2, AddrFI:$rs1, simm12:$imm12)>;
         def : Pat<(StoreOp StTy:$rs2, (IsOrAdd AddrFI:$rs1, simm12:$imm12)),
                   (Inst StTy:$rs2, AddrFI:$rs1, simm12:$imm12)>;
       }
       
       defm : StPat<truncstorei8, SB, GPR>;
       defm : StPat<truncstorei16, SH, GPR>;
       defm : StPat<store, SW, GPR>, Requires<[IsRV32]>;
       defm : StPat<store, SSW, GPR>, Requires<[IsRV32]>;
       ...
       defm : LdPat<sextloadi32, LW>;
       defm : LdPat<extloadi32, LW>;
       defm : LdPat<sextloadi32, SLW>;
       defm : LdPat<extloadi32, SLW>;
       defm : LdPat<zextloadi32, LWU>;
       defm : LdPat<load, LD>;
     
       /// Stores
     
       defm : StPat<truncstorei32, SW, GPR>;
       defm : StPat<truncstorei32, SSW, GPR>;
       ...

Modifying llvm to not save return address to original stack frame and to not use X31 register in .../llvm/lib/Target/RISCV/RISCVRegisterInfo.cpp

       BitVector RISCVRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
         ...
         markSuperRegs(Reserved, RISCV::X0); // zero
         markSuperRegs(Reserved, RISCV::X2); // sp
         markSuperRegs(Reserved, RISCV::X3); // gp
         markSuperRegs(Reserved, RISCV::X4); // tp
         //markSuperRegs(Reserved, RISCV::X31); // added
         Reserved.set(RISCV::X31);
         ...
       }

Modifying function prologue and epilogue in .../llvm/lib/Target/RISCV/RISCVFrameLowering.cpp

      void RISCVFrameLowering::emitPrologue(MachineFunction &MF,MachineBasicBlock &MBB) const {
      ...
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createDefCfaOffset(nullptr, -RealStackSize));
      BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
      BuildMI(MBB, MBBI, DL, TII->get(RISCV::ADDI), RISCV::X31)
          .addReg(RISCV::X31)
          .addImm(-4);
      BuildMI(MBB, MBBI, DL, TII->get(RISCV::SSW),RISCV::X1)
          .addReg(RISCV::X31)
          .addImm(0);
      ...
      }
      ...
      void RISCVFrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const {
      ...
      // Restore the stack pointer using the value of the frame pointer. Only
      // necessary if the stack pointer was modified, meaning the stack size is
      // unknown.
      if (RI->needsStackRealignment(MF) || MFI.hasVarSizedObjects()) {
        assert(hasFP(MF) && "frame pointer should not have been eliminated");
        adjustReg(MBB, LastFrameDestroy, DL, SPReg, FPReg, -FPOffset,
                  MachineInstr::FrameDestroy);
      }
      const RISCVInstrInfo *TII = STI.getInstrInfo();
    
      BuildMI(MBB, MBBI, DL, TII->get(RISCV::SLW),RISCV::X1)
          .addReg(RISCV::X31)
          .addImm(0);
      BuildMI(MBB, MBBI, DL, TII->get(RISCV::ADDI), RISCV::X31)
          .addReg(RISCV::X31)
          .addImm(4);
      ...
      }

  ...

## Setup Benchmark Environment

Make sure these tools already set up.

 - riscv-llvm
 - riscv-gnu-toolchain
 - riscv-pk

Clone the RIMI benchmark Repository

    $ git clone https://git.crypto.islab.re.kr/scm/cap/rimi-benchmarks.git

Update the beebs repository for RIMI Benchmarking

    $ source init_submodules.sh

Now open the `env.sh`, update the environment variables to match with our current RISCV toolchain configuration.

    export RISCV=/opt/riscv
    export PK=$RISCV/riscv32-unknown-elf/bin/pk
    export PATH=$RISCV/bin:$PATH
    export RISCV_XLEN=32

Now we can proceed to benchmarking process.

## Run Benchmark

Run the benchmark using

    $ source run-benchmark.sh

This will generate the logs and instruction histogram for each test in results.

In order to collect and summarize all of the test result for visualization

    $ python instruction-histogram.py

Then we can calculate the code size using

    $ source calculate-code-size.sh

