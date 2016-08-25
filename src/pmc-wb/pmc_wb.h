#ifndef PMC_WB_DRIVER_H
#define PMC_WB_DRIVER_H

#include "wishbone.h"

#define PMC_WB "pmc_wb"
#define PMC_WB_VERSION	"0.1"

#define PMC_WB_VENDOR_ID	0x10dc
#define	PMC_WB_DEVICE_ID	0xc570

#define CONTROL_REGISTER_HIGH	0
#define CONTROL_REGISTER_LOW	4
#define ERROR_FLAG_HIGH		8
#define ERROR_FLAG_LOW		12
#define WINDOW_OFFSET_HIGH	16
#define WINDOW_OFFSET_LOW	20
#define SDWB_ADDRESS_HIGH	24
#define SDWB_ADDRESS_LOW	28

#define MASTER_CTL_HIGH		64
#define MASTER_CTL_LOW		68
#define MASTER_ADR_HIGH		72
#define MASTER_ADR_LOW		76
#define MASTER_DAT_HIGH		80
#define MASTER_DAT_LOW		84

#define WINDOW_HIGH	0xFFFF0000UL
#define WINDOW_LOW	0x0000FFFCUL

/* PCI core control and status registers in BAR0 */

#define PCI_STATUS_REG      0x04
#define PCI_CONF_IRQ        0x3C

#define WB_CONF_IRQ_STATUS_MASK 0x00000001

#define WB_CONF_INT_ACK_REG 0x1E8 /* PCI core WB interrupt Acknowledge register */    
#define WB_CONF_ICR_REG     0x1EC /* PCI core WB interrupt Control register */    
#define WB_CONF_ISR_REG     0x1F0 /* PCI core WB interrupt Status  register */


/* One per BAR */
struct pmc_wb_resource {
	unsigned long start;			/* start addr of BAR */
	unsigned long end;			/* end addr of BAR */
	unsigned long size;			/* size of BAR */
	void *addr;				/* remapped addr */
};

/* One per physical card */
struct pmc_wb_dev {
	struct pci_dev* pci_dev;
	struct pmc_wb_resource pci_res[3];
	int    msi;
	
	struct wishbone wb;
	//struct mutex mutex; /* only one user can open a cycle at a time */
	unsigned int window_offset;
	unsigned int low_addr, width, shift;
};

#endif
