/*
 * Apple M1 Pro/Max Interrupt Controller - Version 2
 * 
 * Copyright (c) 2022 Javier Alvarez <javier.alvarez@allthingsembedded.net>
 *
 * Based on apple-aic.c
 *   Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

/*
 * For a full description of the AIC v2 interrupt controller read the
 * irq-apple-aic.c driver in the Linux kernel, the code is unrelated
 * but the peripheral is documented full there.
 *
 * 0x0000 - 0x2000 - Global registers
 * 0x2000 - 0xC000 - IRQ Configuration registers. 
 *                   Supported operations: 
 *                     * Mask/Unmask interrupts
 *                     * Pend/Unpend interrupts
 *                     * HW state of the irq line
 *                     * Configuring IRQ target
 * 0xC000 - Aic Events. Allows the user to get the current interrupt number
 *
 * Right now for development I've placed the "views" into other CPU's versions
 * of these registers at 0x5000, so far I'm just putting the reason
 * registers there. The normal reason will be handled by aliases that
 * redirect that region to the per-cpu region. This is going to need to
 * be more complete in the future and use the real offsets but for now
 * this will be enough.
 * TODO: above is not actually done, right now we just use a hack
 * since for single core the interrupt executer will always be called
 * as the same CPU. We *cannot* get the CPU from qemu (at least in my
 * opinion) as that will create a situation where we assume the current
 * CPU is always an ARM cpu or compattible, which isn't ideal for a peripherial
 * instead it will just error if a CPU hasn't been hooked up properly.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "cpu.h"
#include "qemu/timer.h"
#include "hw/intc/apple-aic2.h"
#include "trace.h"

// Global registers
#define AIC_VERSION_REG         (0x0000)
#define AIC_INFO1_REG           (0x0004)
#define AIC_INFO2_REG           (0x0008)
#define AIC_INFO3_REG           (0x000C)
#define AIC_RESET_REG           (0x0010)
#define AIC_GLOB_CONFIG_REG     (0x0014)

/* Packed IRQ bitmaps have this number of bytes */
#define AIC_IRQ_REG_SIZE        ((AIC_NUM_IRQ + 7)/ 8)

/* IRQ Configuration bitmap */
#define AIC_CONFIG_REG_BASE     (0x2000)
#define AIC_CONFIG_REG_END      (AIC_CONFIG_REG_BASE + (AIC_NUM_IRQ * 4) - 1)

/* SW initiated HW IRQs */
#define AIC_SW_SET              (AIC_CONFIG_REG_BASE + (AIC_NUM_IRQ * 4))
#define AIC_SW_SET_END          (AIC_SW_SET + AIC_IRQ_REG_SIZE - 1)
#define AIC_SW_CLEAR            (AIC_SW_SET + AIC_IRQ_REG_SIZE)
#define AIC_SW_CLEAR_END        (AIC_SW_CLEAR + AIC_IRQ_REG_SIZE - 1)

/* HW IRQ mask set */
#define AIC_MASK_SET            (AIC_SW_CLEAR + AIC_IRQ_REG_SIZE)
#define AIC_MASK_SET_END        (AIC_MASK_SET + AIC_IRQ_REG_SIZE - 1)

/* HW IRQ mask clear */
#define AIC_MASK_CLEAR          (AIC_MASK_SET + AIC_IRQ_REG_SIZE)
#define AIC_MASK_CLEAR_END      (AIC_MASK_CLEAR + AIC_IRQ_REG_SIZE - 1)

/* HW IRQ state */
#define AIC_HW_STATE            (AIC_MASK_CLEAR + AIC_IRQ_REG_SIZE)
#define AIC_HW_STATE_END        (AIC_HW_STATE + AIC_IRQ_REG_SIZE - 1)

/* Event base */
#define AIC_EVENT               0xC000

#define AIC_EVENT_IRQ_NUM_OFF   (0)
#define AIC_EVENT_IRQ_NUM_MASK  (0xFFFF)
#define AIC_EVENT_TYPE_OFF      (16)
#define AIC_EVENT_TYPE_MASK     (0xFF0000)
#define AIC_EVENT_DIE_OFF       (24)
#define AIC_EVENT_DIE_MASK      (0xFF000000)

/* IRQ event types (reported by reason register) */
/* TODO: Rename to include event */
#define AIC_IRQ_HW  (1<<0)
#define AIC_IRQ_IPI (1<<2)

/* Helpers to avoid mystery constants */
#define AIC_IRQ_HIGH    1
#define AIC_IRQ_LOW     0

/*
 * Reports if the current HW IRQ line number is a valid number
 * Takes a state argument to support eventual use of a property
 * for the IRQ number argument
 */
static inline bool is_aic_irq_valid(AppleAIC2State *s, int n)
{
    /* IRQs go from 0..num_irq-1 */
    if ((n < 0) || (n >= s->num_irq)) {
        return false;
    }
    return true;
}

/* 
 * Handles raising/lowering an IRQ output only if the value changed
 * this might make tracing annoying though
 */
static inline void aic_set_irq(AppleAIC2State *s, int cpu, int level)
{
    if (cpu > AIC_MAX_CPUS) {
        return;
    }
    /* TODO Assert that level is 1 or 0? */

    /* Determine if level has changed and update with new value */
    int last_level = s->core_state[cpu].irq_status;
    int changed = last_level != level;
    s->core_state[cpu].irq_status = level;

    /* If the value has changed we propegate to the IRQ handlers */
    if (changed) {
        /* TODO: Add tracing */
        qemu_set_irq(s->core_state[cpu].irq, level);
    }
}

static uint32_t find_next_bit32(uint32_t* addr) 
{
    const uint32_t value = *addr;
    if (value == 0) { return 32; }

    const uint32_t bit = ctz32(value);
    *addr = value & ~(1 << bit);

    return bit;
}


/* Handle updating and initiating IRQs after applying masks */
static void aic_update_irq(AppleAIC2State *s)
{
    /* 
     * NOTE: This code is like this because it's a slightly more
     * optimal approach visually than literally testing every bit
     * through the accessor functions. However it might actually
     * be less optimal because it might confuse the compiler more
     */
    /* 
     * Holds the value of the lines for each output IRQ line after
     * processing all the potential pending IRQs, if it's different
     * than the last state of that line, we raise or lower that line
     */
    uint32_t result_lines = 0;

    /* Update the IRQ lines */
    for (int i = 0; i < AIC_IRQ_REG_COUNT; i++) {
        /* Both HW and SW triggered HW are OR'd into one */
        uint32_t pending = s->irq_pending[i] | s->sw_pending[i];
        /* Apply the mask */
        pending &= ~s->irq_mask[i];
        if (pending) {
            //printf("Found unmasked pending: %0x\n", pending);
            /* Base number for IRQs stored in this register */
            int irq_base = i * 32;
            for (int j = find_next_bit32(&pending); 
                     j < 32; 
                     j = find_next_bit32(&pending)) {
                // TODO(javier-varez): This is hardcoded for 1 cpu. Handle mutliple cpus later
                uint32_t dist = 0x1;

                /* 
                 * Only process the lines if some of them might be unset 
                 * (this clears all the values already set in result_lines)
                 * Already set lines alerady have a higher priority (lower #)
                 * IRQ waiting to be raised until after all lines are processed
                 * TODO: should we just raise immediately?
                 */
                dist &= ~result_lines;
                /*
                 * Now that we have masked the previously set lines
                 * we can just OR in the new ones and process the
                 * new lines to set the active IRQ numbers
                 */
                result_lines |= dist;
                /* Try to distribute the first set interrupt */
                if (dist) {
                    for (int k = ctz32(dist); k < 32; k++) {
                        /* 
                         * We only accelerate which bit we start
                         * with, have to check the rest of the bits
                         * too
                         */
                        if (!(dist & (1<<k))) {
                            continue;
                        }
                        /* Store the IRQ number that is responsible
                         * for raising the line, just store directly
                         * in reason.
                         */
                        s->core_state[k].irq_event = irq_base+j;
                    }
                }
            }
        }
        /* 
         * TODO: If all the lines are set then we don't need to iterate
         * anymore (but this is only useful if we can ignore the lines
         * not being used by CPUs)
         */
    }
    /* TODO Remove for merge */
    if (result_lines != 0) {
        //printf("AIC: Computed IRQs after search: %0x\n", result_lines);
    }

    /* This can't really be faster since we need to apply every bit */
    for (int i = 0; i < s->num_cpu; i++) {
        /* TODO: order is unknown between IPIs and IRQs but we apply
         * hw IRQs first
         */
        if (result_lines & (1<<i)) {
            /* Add in the IRQ_HW type since we know it's hardware now */
            s->core_state[i].irq_event |= AIC_IRQ_HW << 16;
            aic_set_irq(s, i, AIC_IRQ_HIGH);
        } else {
            /* TODO: move this out of nesting */
            /* Nothing is raising the IRQ line */
            /* Clear reason */
            s->core_state[i].irq_event = 0;
            /* Set the line low */
            aic_set_irq(s, i, AIC_IRQ_LOW);
        }
    }
}

/* 
 * This is marked as read because that's what it handles, but it
 * really processes an "update" since reads modify state (masking the irq that caused the event)
 */
static uint32_t aic_read_event(AppleAIC2State *s, int cpu_index) {
    /* TODO Clean up */
    if (cpu_index > AIC_MAX_CPUS) {
        error_report("CPU calling us had an index of 0x%x which was too high",
                     cpu_index);
        abort();
    }

    /* Get the current reason */
    uint32_t event = s->core_state[cpu_index].irq_event;

    if (!event) {
        /* 
         * If nothing is pending this is unspecified but just return 0
         * and skip processing.
         * 
         * (NOTE linux driver actually seems to assume that returning 0
         * is a valid way to indiciate no more IRQs pending so maybe it's
         * not unspecified)
         */
        return 0;
    }

    /*
     * Now we have to mask the reason since the IRQ handler has now processed
     * this IRQ event and must tell the AIC it's ready to process another
     */
    const uint32_t event_type = (event & AIC_EVENT_TYPE_MASK) >> AIC_EVENT_TYPE_OFF;
    const uint32_t event_irq = (event & AIC_EVENT_IRQ_NUM_MASK) >> AIC_EVENT_IRQ_NUM_OFF;

    if (event_type == AIC_IRQ_HW) {
        /* Mask the provided IRQ number */
        aic_set_bit(event_irq, s->irq_mask);
    } else {
      printf("AIC Event type not supported %d", event_type);
      abort();
    }
    /* Trigger an update in case a new IRQ is triggered here (updates reason) */
    aic_update_irq(s);

    /* Return the original reason, next pass will get new reason */
    return event;
}

/* Handle updating and apply a single 32-bit register of IRQ */
static void aic_update_irq_reg(AppleAIC2State *s, int reg)
{
    assert(reg < AIC_IRQ_REG_COUNT);

    /*
     * Right now without a conception of what else might be
     * driving the IRQ line, we cannot do much meaningful
     * processing simply handling it in a per-reg update.
     * 
     * Theoretically we could set IRQ lines and only do
     * updates for cleared ones, but that means this only
     * does anything meaningful for fully set IRQ state
     * registers, which is highly unlikely to say the least.
     * 
     * Having an idea of if other interrupts are driving
     * the IRQ line for this CPU would change that but that
     * could get messy, we would keep a counter for each output
     * IRQ line (one per CPU then) and increment it when
     * an IRQ is raised on that line, and decrement it when
     * an IRQ stops asserting. However that gets complex fast
     * since we have to make sure to update it after masking
     * and overall there's a potential for error. Instead lets
     * just hope iterating is fast enough
     */
    aic_update_irq(s);
}

/* 
 * FIXME This whole concept is broken by design, we need to use aliases
 * and per-cpu memory regions that handle the dispatch for us properly
 * for now we want things to work so use this (other code uses this too)
 * and for us this is the cleanest structure without overcomplicating the
 * SoC side of things.
 * 
 * This method should be only used in the read/write mem methods since
 * everything else should have this detail abstracted out as a parameter
 * for when the proper code is written.
 */
static CPUState* get_current_cpu(void) {
    /* FIXME this is broken by design */
    CPUState *cpu = CPU(current_cpu);
    return cpu;
}

/* TODO MMIO for real, each type of IO region should probably be split */
static uint64_t aic_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleAIC2State *s = APPLE_AIC2(opaque);
    CPUState *cpu = get_current_cpu();
    int cpu_index = cpu->cpu_index;

    switch (offset) {
    case AIC_INFO1_REG:
        return s->num_irq - 1;
    case AIC_GLOB_CONFIG_REG:
        // TODO(javier-varez): Add other bits
        return s->enabled;
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        return *aic_offset_ptr(offset, AIC_SW_SET, s->sw_pending);
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        return *aic_offset_ptr(offset, AIC_SW_CLEAR, s->sw_pending);
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        return *aic_offset_ptr(offset, AIC_MASK_SET, s->irq_mask);
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        return *aic_offset_ptr(offset, AIC_MASK_CLEAR, s->irq_mask);
    case AIC_HW_STATE ... AIC_HW_STATE_END: /* State of input HW lines */
        /* Get the current pending value which is what I think this
         * does on hardware
         */
        /* FIXME: bounds checking? */
        //printf("accessing irq_pending[%ld] = %d\n", (offset-AIC_HW_STATE)/4, s->irq_pending[(offset - AIC_HW_STATE)/4]);
        return *aic_offset_ptr(offset, AIC_HW_STATE, s->irq_pending);
    case AIC_EVENT:
        return aic_read_event(s, cpu_index);
    default:
        printf("aic_mem_read: Unhandled read from @%0" HWADDR_PRIx " of "
               "size=%d\n", offset, size);
    }
    return 0;
}

static void aic_mem_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AppleAIC2State *s = APPLE_AIC2(opaque);
    
    uint32_t value = val;
    bool changed = false;
    switch (offset) {
    case AIC_GLOB_CONFIG_REG: {
        // TODO(javier-varez): Handle other bits
        const bool previous = s->enabled;
        s->enabled = (val & 1) != 0;

        // If value changed, update interrupts
        if (s->enabled != previous) {
          aic_update_irq(s);
        }
        break;
    }
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        changed = aic_set_reg(offset, AIC_SW_SET, s->sw_pending, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_SW_SET));
        }
        break;
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        changed = aic_clear_reg(offset, AIC_SW_CLEAR, s->sw_pending, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_SW_CLEAR));
        }
        break;
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        changed = aic_set_reg(offset, AIC_MASK_SET, s->irq_mask, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_MASK_SET));
        }
        break;
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        changed = aic_clear_reg(offset, AIC_MASK_CLEAR, s->irq_mask, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_MASK_CLEAR));
        }
        break;
    case AIC_EVENT:
        // Ignore writes to the AIC event register
        break;
    default:
        printf("aic_mem_write: Unhandled write of %0" PRIx64
               " to @%0" HWADDR_PRIx " of size=%d\n", val, offset, size);
    }
}

static const MemoryRegionOps aic_io_ops = {
    .read = aic_mem_read,
    .write = aic_mem_write,
    .impl.min_access_size = 4, /* We only want to think about 32 bits */
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8, /* I don't see why not */
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Handle incoming HW interrupts changes here */
static void aic_irq_handler(void *opaque, int n, int level)
{
    /* TODO: Do this right? */
    AppleAIC2State *s = APPLE_AIC2(opaque);
    /* TODO: handle aborting if n>AIC_NUM_IRQ */
    /* Update pending */
    if (is_aic_irq_valid(s, n)) {
        /* The logic below is too confusing, wrap in macros or
         * an inline function (inline functions seem cleaner */
        if (level) {
            aic_set_bit(n, s->irq_pending);
            //printf("irq_pending[%d] |= %d\n", n/32, n & 0x1F);
        } else {
            aic_clear_bit(n, s->irq_pending);
            //printf("irq_pending[%d] &= ~(%0x)\n", n/32, 1<<(n & 0x1F));
        }
    }
    aic_update_irq(s);
}

static void apple_aic_realize(DeviceState *dev, Error **errp)
{
    AppleAIC2State *s = APPLE_AIC2(dev);
    
    /* Initialize properties */
    /* TODO: make these top level properties instead */
    s->num_irq = AIC_NUM_IRQ;
    s->num_cpu = AIC_MAX_CPUS;
    
    /* Clear state */
    /* NOTE: These use compile time constants to get better
     * generated code even if it is less generic
     */
    aic_zero_bits(s->sw_pending, AIC_NUM_IRQ);
    aic_zero_bits(s->irq_pending, AIC_NUM_IRQ);
    aic_fill_bits(s->irq_mask, AIC_NUM_IRQ);
    
    /* Use memset to clear all the per-CPU data */
    /* TODO: Allocate dynamically? */
    memset(&s->core_state, 0, sizeof(s->core_state));
    for (int i = 0; i < s->num_cpu; i++) {
        s->core_state[i].ipi_mask = ~0U;
    }
    
    memory_region_init_io(&s->mmio_mr, OBJECT(s), &aic_io_ops, s, "aic2-mmio",
                          AIC_MMIO_SIZE);
    
    /* Init input HW interrupts */
    qdev_init_gpio_in(dev, aic_irq_handler, s->num_irq);
    
    SysBusDevice *busdev = SYS_BUS_DEVICE(s);
    sysbus_init_mmio(busdev, &s->mmio_mr);
    
    for (int i = 0; i < s->num_cpu; i++) {
        sysbus_init_irq(busdev, &s->core_state[i].irq);
    }
}

static void apple_aic_class_init(ObjectClass *oc, void *data) {
    DeviceClass *dc = DEVICE_CLASS(oc);
    
    dc->realize = apple_aic_realize;
}

/* TODO: VM State stuff seems important for interrupt controllers */

static const TypeInfo apple_aic_type_info = {
    .name = TYPE_APPLE_AIC2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AppleAIC2State),
    .class_init = apple_aic_class_init,
};

static void apple_aic_register_types(void)
{
    type_register_static(&apple_aic_type_info);
}

type_init(apple_aic_register_types);
