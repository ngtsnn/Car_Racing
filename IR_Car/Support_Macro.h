#ifndef SUPPORT_MACRO_H
#define SUPPORT_MACRO_H

#define sbi(reg, bit)          reg|=(1<<bit)
#define cbi(reg, bit)          reg&=(~(1<<bit))
#define bit_is_set(reg, bit)   reg&(1<<bit)
#define bit_is_clear(reg, bit) !bit_is_set(reg,bit)

#endif /* SUPPORT_MACRO_H */
