#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/getpc.h"
#include "accel/tcg/cpu-ldst.h"

/* DMA 转置指令 */
void helper_dma(CPURISCVState *env, target_ulong tl_dst, target_ulong tl_src, target_ulong tl_len)
{
    // 计算矩阵规模: tl_len=0 -> 8x8, tl_len=1 -> 16x16, tl_len=2 -> 32x32
    int size = 8 << tl_len;
    
    // 获取当前 PC，用于异常处理
    uintptr_t ra = GETPC();
    
    // 矩阵转置: A[i][j] -> B[j][i]
    // 源矩阵: tl_src, 目标矩阵: tl_dst
    // 矩阵是 size x size，按行优先存储
    
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            // 从源矩阵读取 A[i][j]
            target_ulong src_addr = tl_src + (i * size + j) * 4;  // 4 bytes per FP32
            uint32_t value = cpu_ldl_le_data_ra(env, src_addr, ra);
            
            // 写入目标矩阵 B[j][i]
            target_ulong dst_addr = tl_dst + (j * size + i) * 4;
            cpu_stl_le_data_ra(env, dst_addr, value, ra);
        }
    }
}

/* Sort 冒泡排序指令 */
void helper_sort(CPURISCVState *env, target_ulong tl_addr, target_ulong tl_array_size, target_ulong tl_sort_num)
{
    // tl_addr: 数组起始地址
    // tl_array_size: 数组总大小
    // tl_sort_num: 参与排序的元素数量
    
    uintptr_t ra = GETPC();
    target_ulong sort_count = tl_sort_num < tl_array_size ? tl_sort_num : tl_array_size;
    
    // 冒泡排序算法 - 升序排序
    for (target_ulong i = 0; i < sort_count - 1; i++) {
        for (target_ulong j = 0; j < sort_count - 1 - i; j++) {
            // 读取 array[j]
            target_ulong addr_j = tl_addr + j * 4;  // INT32 = 4 bytes
            int32_t val_j = (int32_t)cpu_ldl_le_data_ra(env, addr_j, ra);
            
            // 读取 array[j+1]
            target_ulong addr_j1 = tl_addr + (j + 1) * 4;
            int32_t val_j1 = (int32_t)cpu_ldl_le_data_ra(env, addr_j1, ra);
            
            // 如果 array[j] > array[j+1]，交换
            if (val_j > val_j1) {
                cpu_stl_le_data_ra(env, addr_j, (uint32_t)val_j1, ra);
                cpu_stl_le_data_ra(env, addr_j1, (uint32_t)val_j, ra);
            }
        }
    }
}

/* Crush 压缩指令 */
void helper_crush(CPURISCVState *env, target_ulong tl_dst, target_ulong tl_src, target_ulong tl_num)
{
    // tl_src: 源数组起始地址
    // tl_num: 源数组元素数量
    // tl_dst: 目标数组起始地址
    
    uintptr_t ra = GETPC();
    
    // 两两打包：将相邻两个 8bit 元素的低 4bit 合并为一个 8bit
    for (target_ulong i = 0; i < tl_num; i += 2) {
        // 读取 src[i] 的低 4 位
        uint8_t val0 = cpu_ldub_data_ra(env, tl_src + i, ra) & 0xF;
        
        uint8_t packed;
        if (i + 1 < tl_num) {
            // 读取 src[i+1] 的低 4 位，放到高 4 位
            uint8_t val1 = cpu_ldub_data_ra(env, tl_src + i + 1, ra) & 0xF;
            packed = val0 | (val1 << 4);
        } else {

            packed = val0;
        }
        
        // 写入目标数组
        cpu_stb_data_ra(env, tl_dst + (i / 2), packed, ra);
    }
}

/* Expand 解压指令 */
void helper_expand(CPURISCVState *env, target_ulong tl_dst, target_ulong tl_src, target_ulong tl_num)
{
    // tl_src: 源数组起始地址
    // tl_num: 源数组元素数量
    // tl_dst: 目标数组起始地址
    
    uintptr_t ra = GETPC();
    
    // 每个 8bit 元素拆分为 4bit 元素
    for (target_ulong i = 0; i < tl_num; i++) {
        // 读取源数组的一个字节
        uint8_t src_val = cpu_ldub_data_ra(env, tl_src + i, ra);
        
        // 提取低 4 位
        uint8_t low4 = src_val & 0xF;
        // 提取高 4 位
        uint8_t high4 = (src_val >> 4) & 0xF;
        
        // 写入目
        cpu_stb_data_ra(env, tl_dst + (i * 2), low4, ra);
        cpu_stb_data_ra(env, tl_dst + (i * 2 + 1), high4, ra);
    }
}
