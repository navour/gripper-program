"""Run actual Keil ARM history functions with Unicorn; no HAL/peripheral simulation."""
from pathlib import Path
import struct
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_XPSR
from elftools.elf.elffile import ELFFile

class ArmHistory:
    def __init__(self, axf, slave):
        self.slave=slave
        self.u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        self.u.mem_map(0x08000000,0x20000)
        self.u.mem_map(0x20000000,0x20000)
        with open(axf,'rb') as f:
            elf=ELFFile(f)
            self.symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            for section in elf.iter_sections():
                if section['sh_flags']&2 and section['sh_type']!='SHT_NOBITS' and section['sh_size']:
                    address=section['sh_addr']
                    if 0x08000000<=address<0x08020000 or 0x20000000<=address<0x20020000:
                        self.u.mem_write(address,section.data())
        self.u.reg_write(UC_ARM_REG_XPSR,0x01000000)
        self.call('PressureHistory_Init',slave,slave)

    def call(self,name,*args):
        sp=0x2001E000
        for reg,arg in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):
            self.u.reg_write(reg,arg)
        for i,arg in enumerate(args[4:]):
            self.u.mem_write(sp+4*i,struct.pack('<I',arg))
        self.u.reg_write(UC_ARM_REG_SP,sp)
        self.u.reg_write(UC_ARM_REG_LR,0x08005001)
        self.u.emu_start(self.symbols[name]|1,0x08005000,count=1000000)
        if self.u.reg_read(UC_ARM_REG_PC)!=0x08005000:
            raise RuntimeError(f'{name} failed to return')
        return self.u.reg_read(UC_ARM_REG_R0)

    def read(self,start,count):
        self.call('PressureHistory_ReadRegs',start,count,0x20010000)
        return list(struct.unpack(f'>{count}H',self.u.mem_read(0x20010000,2*count)))

    def submit(self,op,session,seq,token):
        words=[op,*struct.unpack('>4H',struct.pack('>Q',session)),seq>>16,seq&65535,token]
        self.u.mem_write(0x20011000,struct.pack('<8H',*words))
        return self.call('PressureHistory_SubmitCommand',0x20011000)

    def boundary(self): self.call('PressureHistory_BeginCycle')
    def service(self): self.call('PressureHistory_Service')
    def push(self,offset=0,flags=15):
        values=[(i+offset)&65535 for i in range(128)]
        self.u.mem_write(0x20012000,struct.pack('<128H',*values))
        self.call('PressureHistory_PushCycle',0x20012000,0x20012040,0x20012080,0x200120c0,123,456,flags,15)
