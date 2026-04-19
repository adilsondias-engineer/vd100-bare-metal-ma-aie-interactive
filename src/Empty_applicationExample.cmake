set(ddr_noc_1_ddr_memory_DDR_LOW_0 "0x0;0x80000000")
set(versal_cips_0_pspmc_0_psv_ocm_ram_0_memory_0 "0xfffc0000;0x40000")
set(DDR ddr_noc_1_ddr_memory_DDR_LOW_0)
set(CODE ddr_noc_1_ddr_memory_DDR_LOW_0)
set(DATA ddr_noc_1_ddr_memory_DDR_LOW_0)
set(TOTAL_MEM_CONTROLLERS "ddr_noc_1_ddr_memory_DDR_LOW_0;versal_cips_0_pspmc_0_psv_ocm_ram_0_memory_0")
set(MEMORY_SECTION "MEMORY
{
	psv_pmc_ram_psv_pmc_ram : ORIGIN = 0xF2000000, LENGTH = 0x20000
	psv_r5_0_atcm_global_MEM_0 : ORIGIN = 0xFFE00000, LENGTH = 0x40000
	psv_r5_1_atcm_global_MEM_0 : ORIGIN = 0xFFE90000, LENGTH = 0x10000
	psv_r5_1_btcm_global_MEM_0 : ORIGIN = 0xFFEB0000, LENGTH = 0x10000
	ddr_noc_1_ddr_memory_DDR_LOW_0 : ORIGIN = 0x0, LENGTH = 0x80000000
	versal_cips_0_pspmc_0_psv_ocm_ram_0_memory_0 : ORIGIN = 0xfffc0000, LENGTH = 0x40000
}")
set(STACK_SIZE 0x2000)
set(HEAP_SIZE 0x2000)
