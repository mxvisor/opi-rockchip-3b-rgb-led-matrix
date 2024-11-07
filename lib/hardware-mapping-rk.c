/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Copyright (C) 2013, 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http: *gnu.org/licenses/gpl-2.0.txt>
 */

/*
 * We do this in plain C so that we can use designated initializers.
 */
#include "hardware-mapping.h"

#define GPIO_BIT(b) ((uint64_t)1<<(b))

struct HardwareMapping matrix_hardware_mappings[] = {
  {
    .name          = "regular",

    .output_enable = GPIO_BIT(16),
    .clock         = GPIO_BIT(6),
    .strobe        = GPIO_BIT(7),


    /* Address lines */
    .a             = GPIO_BIT(11),
    .b             = GPIO_BIT(3),
    .c             = GPIO_BIT(1),
    .d             = GPIO_BIT(4),
    .e             = GPIO_BIT(8),

    /* Parallel chain 0, RGB for both sub-panels */
    .p0_r1         = GPIO_BIT(12),
    .p0_g1         = GPIO_BIT(13),
    .p0_b1         = GPIO_BIT(19),
    .p0_r2         = GPIO_BIT(0),
    .p0_g2         = GPIO_BIT(2),
    .p0_b2         = GPIO_BIT(10),
    },

    {0}};
