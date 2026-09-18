/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// DMAMUX1
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0 /* DMA1:37 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0 /* DMA1:38 */

#define DMAMAP_USART6_RX   DMAMAP_DMA12_USART6RX_1 /* DMA1:71 */
#define DMAMAP_USART6_TX   DMAMAP_DMA12_USART6TX_1 /* DMA1:72 */

#define DMAMAP_SPI5_RX    DMAMAP_DMA12_SPI5RX_0 /* DMA1:83 */
#define DMAMAP_SPI5_TX    DMAMAP_DMA12_SPI5TX_0 /* DMA1:84 */

// DMAMUX1 -> DMA2 (previously unused): UART transmit DMA on the TELEM ports.
// Without these every transmitted byte costs one interrupt, and sustained
// TELEM traffic (uXRCE-DDS / MAVLink at 921600) corrupts in-flight SPI1/SPI5
// DMA reads of the IMUs (grazer10 accel error investigation, Sep 2026).
#define DMAMAP_USART2_TX   DMAMAP_DMA12_USART2TX_1 /* DMA2:44 TELEM1 */
#define DMAMAP_USART3_TX   DMAMAP_DMA12_USART3TX_1 /* DMA2:46 TELEM2 */
#define DMAMAP_UART7_TX    DMAMAP_DMA12_UART7TX_1  /* DMA2:80 TELEM3 */
