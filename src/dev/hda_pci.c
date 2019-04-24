/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National  Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2019, XXX
 * All rights reserved.
 *
 * Authors: XXX 

 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#include <nautilus/nautilus.h>
#include <nautilus/dev.h>   // eventually snddev.h
#include <dev/pci.h>
#include <dev/hda_pci.h>

#ifndef NAUT_CONFIG_DEBUG_HDA_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif 

#define INFO(fmt, args...) INFO_PRINT("hda_pci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("hda_pci: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("hda_pci: " fmt, ##args)

#define GLOBAL_LOCK_CONF uint8_t _global_lock_flags
#define GLOBAL_LOCK() _global_lock_flags = spin_lock_irq_save(&global_lock)
#define GLOBAL_UNLOCK() spin_unlock_irq_restore(&global_lock, _global_lock_flags)

#define STATE_LOCK_CONF uint8_t _state_lock_flags
#define STATE_LOCK(state) _state_lock_flags = spin_lock_irq_save(&(state->lock))
#define STATE_UNLOCK(state) spin_unlock_irq_restore(&(state->lock), _state_lock_flags)

//
// On QEMU using -soundhw hda we see the following:
//
//0:4.0 : 8086:2668 0103c 0010s MSI(off,MSI64,nn=1,bv=0,nv=0,tc=0) legacy(l=11,p=1)

// for protection of global state in the driver
static spinlock_t global_lock;

// number of devices (used to assign names)
static int num_devs=0; 

// list of hda devices
static struct list_head dev_list;

struct hda_pci_dev {
  // for protection of per-device state
  spinlock_t      lock;

  // we are a (generic) nk dev so far
  struct nk_dev  *nk_dev;
  
  // we are a PCI device
  struct pci_dev *pci_dev;
  
  // we will be put on a list of all hda devices
  struct list_head hda_node;
  
  
  // the following is for legacy interrupts
  // we will try to use MSI first
  uint8_t   pci_intr;  // number on bus
  uint8_t   intr_vec;  // number we will see
  
  // The following hide the details of the PCI BARs, since
  // we only have one block of registers
  enum { NONE, IO, MEMORY}  method;
  
  // Where registers are mapped into the I/O address space
  // if at all
  uint16_t  ioport_start;
  uint16_t  ioport_end;  
  
  // Where registers are mapped into the physical memory address space
  // if at all
  uint64_t  mem_start;
  uint64_t  mem_end;
  
};

// accessor functions for device registers

static inline uint32_t hda_pci_read_regl(struct hda_pci_dev *dev, uint32_t offset)
{
  uint32_t result;
  if (dev->method==MEMORY) {
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movl (%1), %0" : "=r"(result) : "r"(addr) : "memory");
  } else {
    result = inl(dev->ioport_start+offset);
  }
  return result;
}

static inline uint16_t hda_pci_read_regw(struct hda_pci_dev *dev, uint32_t offset)
{
  uint16_t result;
  if (dev->method==MEMORY) {
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movw (%1), %0" : "=r"(result) : "r"(addr) : "memory");
  } else {
    result = inw(dev->ioport_start+offset);
  }
  return result;
}

static inline uint8_t hda_pci_read_regb(struct hda_pci_dev *dev, uint32_t offset)
{
  uint8_t result;
  if (dev->method==MEMORY) {
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movb (%1), %0" : "=r"(result) : "r"(addr) : "memory");
  } else {
    result = inb(dev->ioport_start+offset);
  }
  return result;
}

static inline void hda_pci_write_regl(struct hda_pci_dev *dev, uint32_t offset, uint32_t data)
{
  if (dev->method==MEMORY) { 
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movl %1, (%0)" : "=r"(addr): "r"(data) : "memory");
  } else {
    outl(data,dev->ioport_start+offset);
  }
}

static inline void hda_pci_write_regw(struct hda_pci_dev *dev, uint32_t offset, uint16_t data)
{
  if (dev->method==MEMORY) { 
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movw %1, (%0)" : "=r"(addr): "r"(data) : "memory");
  } else {
    outw(data,dev->ioport_start+offset);
  }
}

static inline void hda_pci_write_regb(struct hda_pci_dev *dev, uint32_t offset, uint8_t data)
{
  if (dev->method==MEMORY) { 
    uint64_t addr = dev->mem_start + offset;
    __asm__ __volatile__ ("movb %1, (%0)" : "=r"(addr): "r"(data) : "memory");
  } else {
    outb(data,dev->ioport_start+offset);
  }
}


//
// This dance will eventually get abstracted into the PCI
// subsystem so that we don't repeat it over and over...
static int discover_devices(struct pci_info *pci)
{
  struct list_head *curbus, *curdev;

  INIT_LIST_HEAD(&dev_list);

  if (!pci) { 
    ERROR("No PCI info\n");
    return -1;
  }

  DEBUG("Finding Intel High Definition Audio (HDA) devices\n");

  list_for_each(curbus,&(pci->bus_list)) { 
    struct pci_bus *bus = list_entry(curbus,struct pci_bus,bus_node);

    DEBUG("Searching PCI bus %u for HDA devices\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) { 
      struct pci_dev *pdev = list_entry(curdev,struct pci_dev,dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;

      DEBUG("Device %u is a %x:%x\n", pdev->num, cfg->vendor_id, cfg->device_id);

      // only detect specific chip at the moment
      if (cfg->vendor_id==0x8086 && cfg->device_id==0x2668) {
	DEBUG("Compatible HDA Device Found\n");
	struct hda_pci_dev *hdev;

	hdev = malloc(sizeof(struct hda_pci_dev));
	if (!hdev) {
	  ERROR("Cannot allocate device\n");
	  return -1;
	}

	memset(hdev,0,sizeof(*hdev));

	spinlock_init(&hdev->lock);
	
	// BAR handling will eventually be done by common code in PCI
	
	// we expect one bar exists, just memory-mapped registers
	// and this will be bar 0
	// check to see if there are no others
	int foundmem=0;
	int foundio=0;
	for (int i=0;i<6;i++) { 
	  uint32_t bar = pci_cfg_readl(bus->num,pdev->num, 0, 0x10 + i*4);
	  uint32_t size;
	  DEBUG("bar %d: 0x%0x\n",i, bar);
	  if (i>=1 && bar!=0) { 
	    DEBUG("Not expecting this to be a non-empty bar...\n");
	  }
	  if (!(bar & 0x1)) { 
	    uint8_t mem_bar_type = (bar & 0x6) >> 1;
	    if (mem_bar_type != 0) { 
	      ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
	      return -1;
	    }
	  }

	  // determine size
	  // write all 1s, get back the size mask
	  pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, 0xffffffff);
	  // size mask comes back + info bits
	  size = pci_cfg_readl(bus->num,pdev->num,0,0x10 + i*4);

	  // mask all but size mask
	  if (bar & 0x1) { 
	    // I/O
	    size &= 0xfffffffc;
	  } else {
	    // memory
	    size &= 0xfffffff0;
	  }
	  size = ~size;
	  size++; 

	  // now we have to put back the original bar
	  pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, bar);

	  if (!size) { 
	    // non-existent bar, skip to next one
	    continue;
	  }

	  if (size>0 && i>=1) { 
	    ERROR("unexpected hda pci bar with size>0!\n");
	    return -1;
	  }

	  if (bar & 0x1) { 
	    hdev->ioport_start = bar & 0xffffffc0;
	    hdev->ioport_end = hdev->ioport_start + size;
	    foundio=1;
	  } else {
	    hdev->mem_start = bar & 0xfffffff0;
	    hdev->mem_end = hdev->mem_start + size;
	    foundmem=1;
	  }

	}

	// for now, privilege the memory interface
	if (foundmem) {
	  hdev->method = MEMORY;
	} else if (foundio) {
	  hdev->method = IO;
	} else {
	  hdev->method = NONE;
	  ERROR("Device has no register access method... Impossible...\n");
	  panic("Device has no register access method... Impossible...\n");
	  return -1;
	}

	hdev->pci_dev = pdev;

	INFO("Found HDA device: bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p access_method=%s\n",
	     bus->num, pdev->num, 0,
	     hdev->pci_intr, hdev->intr_vec,
	     hdev->ioport_start, hdev->ioport_end,
	     hdev->mem_start, hdev->mem_end,
	     hdev->method==IO ? "IO" : hdev->method==MEMORY ? "MEMORY" : "NONE");
                 

	list_add(&hdev->hda_node,&dev_list);
      }
    }
  }
  return 0;
}

			 
static int bringup_device(struct hda_pci_dev *dev)
{
  DEBUG("Bringing up device %u:%u.%u\n",dev->pci_dev->bus->num,dev->pci_dev->num,dev->pci_dev->fun);
  if (dev->pci_dev->msi.type!=PCI_MSI_NONE) {
    // switch on early (and detect - we will no do legacy or MSI-X)
#if 0 // do interrupt setup later
    if (pci_dev_enable_msi(dev->pci_dev)) {
      ERROR("Failed to enable MSI on device...\n");
      return -1;
    }
#endif
  } else {
    ERROR("Device does not support MSI...\n");
    return -1;
  }

  // initialize device here...

  return 0;
}

// for a future sound device abstraction
// this would eventually go into snddev.h
struct nk_snd_dev_int {
  struct nk_dev_int  dev_int;
  // nothing specific to sound cards so far
};

static struct nk_snd_dev_int ops;
  
  
static int bringup_devices()
{
  struct list_head *curdev, tx_node;
  int rc;

  rc = 0;

  DEBUG("Bringing up HDA devices\n");
  int num = 0;

  list_for_each(curdev,&(dev_list)) { 
    struct hda_pci_dev *dev = list_entry(curdev,struct hda_pci_dev,hda_node);
    int ret = bringup_device(dev);
    if (ret) { 
      ERROR("Bringup of HDA device failed\n");
      rc = -1;
    }
    char buf[80];
    snprintf(buf,80,"hda%d",num_devs++);
    dev->nk_dev = nk_dev_register(buf,NK_DEV_GENERIC,0,(struct nk_dev_int *)&ops,dev);
    if (!dev->nk_dev) {
      ERROR("Unable to register device %s\n",buf);
      return -1;
    }
  }

  return rc;
  
}

int hda_pci_init(struct naut_info * naut)
{
  spinlock_init(&global_lock);
  
  if (discover_devices(naut->sys.pci)) {
    ERROR("Discovery failed\n");
    return -1;
  }

  return bringup_devices();
}
    

int hda_pci_deinit()
{
  // should really scan list of devices and tear down...
  INFO("deinited\n");
  return 0;
}



