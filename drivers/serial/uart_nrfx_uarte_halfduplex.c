/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Nordic Semiconductor nRF UARTE
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_uarte.h>
#include <nrfx_timer.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_nrfx_uarte_half_duplex, CONFIG_UART_LOG_LEVEL);

#define RX_FLUSH_WORKAROUND 1

#define UARTE(idx)                DT_NODELABEL(uart##idx)
#define UARTE_HAS_PROP(idx, prop) DT_NODE_HAS_PROP(UARTE(idx), prop)
#define UARTE_PROP(idx, prop)     DT_PROP(UARTE(idx), prop)

/* Execute macro f(x) for all instances. */
#define UARTE_FOR_EACH_INSTANCE(f, sep, off_code, ...)                                             \
	NRFX_FOREACH_PRESENT(UARTE, f, sep, off_code, __VA_ARGS__)

/* Determine if any instance is using interrupt driven API. */
#define IS_INT_DRIVEN(unused, prefix, i, _)                                                        \
	(IS_ENABLED(CONFIG_HAS_HW_NRF_UARTE_HALF_DUPLEX##prefix##i) &&                             \
	 IS_ENABLED(CONFIG_UART_##prefix##i##_INTERRUPT_DRIVEN))

#if UARTE_FOR_EACH_INSTANCE(IS_INT_DRIVEN, (||), (0))
#define UARTE_INTERRUPT_DRIVEN 1
#endif

/* Determine if any instance is not using asynchronous API. */
#define IS_NOT_ASYNC(unused, prefix, i, _)                                                         \
	(IS_ENABLED(CONFIG_HAS_HW_NRF_UARTE_HALF_DUPLEX##prefix##i) &&                             \
	 !IS_ENABLED(CONFIG_UART_##prefix##i##_ASYNC))

#if UARTE_FOR_EACH_INSTANCE(IS_NOT_ASYNC, (||), (0))
#define UARTE_ANY_NONE_ASYNC 1
#endif

/* Determine if any instance is using asynchronous API. */
#define IS_ASYNC(unused, prefix, i, _)                                                             \
	(IS_ENABLED(CONFIG_HAS_HW_NRF_UARTE_HALF_DUPLEX##prefix##i) &&                             \
	 IS_ENABLED(CONFIG_UART_##prefix##i##_ASYNC))

#if UARTE_FOR_EACH_INSTANCE(IS_ASYNC, (||), (0))
BUILD_ASSERT(0, "nRF UARTE half-duplex driver does not support async API");
#endif

/* Determine if any instance is using enhanced poll_out feature. */
#define IS_ENHANCED_POLL_OUT(unused, prefix, i, _)                                                 \
	IS_ENABLED(CONFIG_UART_##prefix##i##_ENHANCED_POLL_OUT)

#if UARTE_FOR_EACH_INSTANCE(IS_ENHANCED_POLL_OUT, (||), (0))
#define UARTE_ENHANCED_POLL_OUT 1
#endif

#define INSTANCE_PROP(unused, prefix, i, prop)    UARTE_PROP(prefix##i, prop)
#define INSTANCE_PRESENT(unused, prefix, i, prop) 1

/* Driver supports case when all or none instances support that HW feature. */
#if (UARTE_FOR_EACH_INSTANCE(INSTANCE_PROP, (+), (0), endtx_stoptx_supported)) ==                  \
	(UARTE_FOR_EACH_INSTANCE(INSTANCE_PRESENT, (+), (0), endtx_stoptx_supported))
#define UARTE_HAS_ENDTX_STOPTX_SHORT 1
#endif

#if (UARTE_FOR_EACH_INSTANCE(INSTANCE_PROP, (+), (0), frame_timeout_supported)) ==                 \
	(UARTE_FOR_EACH_INSTANCE(INSTANCE_PRESENT, (+), (0), frame_timeout_supported))
#define UARTE_HAS_FRAME_TIMEOUT 1
#endif

/*
 * RX timeout is divided into time slabs, this define tells how many divisions
 * should be made. More divisions - higher timeout accuracy and processor usage.
 */
#define RX_TIMEOUT_DIV 5

/* Size of hardware fifo in RX path. */
#define UARTE_HW_RX_FIFO_SIZE 5

#define PINCTRL_STATE_RX PINCTRL_STATE_DEFAULT
#define PINCTRL_STATE_TX PINCTRL_STATE_PRIV_START

#ifdef UARTE_INTERRUPT_DRIVEN
struct uarte_nrfx_int_driven {
	uart_irq_callback_user_data_t cb; /**< Callback function pointer */
	void *cb_data;                    /**< Callback function arg */
	uint8_t *tx_buffer;
	uint16_t tx_buff_size;
	volatile bool disable_tx_irq;
	bool tx_irq_enabled;
#ifdef CONFIG_PM_DEVICE
	bool rx_irq_enabled;
#endif
	atomic_t fifo_fill_lock;
	bool rx_irq_processed;
};
#endif

/* Device data structure */
struct uarte_nrfx_data {
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	struct uart_config uart_config;
#endif
#ifdef UARTE_INTERRUPT_DRIVEN
	struct uarte_nrfx_int_driven *int_driven;
	struct k_work_delayable tx_ready_work;
	struct device *dev;
#endif
	atomic_val_t poll_out_lock;
	atomic_t flags;
#ifdef UARTE_ENHANCED_POLL_OUT
	uint8_t ppi_ch_endtx;
#endif
	enum {
		UARTE_MODE_IDLE,
		UARTE_MODE_TX,
		UARTE_MODE_RX
	} mode;
	uint32_t last_rx_end_time_ticks;
};

/* If enabled then ENDTX is PPI'ed to TXSTOP */
#define UARTE_CFG_FLAG_PPI_ENDTX BIT(0)

/* If enabled then TIMER and PPI is used for byte counting. */
#define UARTE_CFG_FLAG_HW_BYTE_COUNTING BIT(1)

/* Macro for converting numerical baudrate to register value. It is convenient
 * to use this approach because for constant input it can calculate nrf setting
 * at compile time.
 */
#define NRF_BAUDRATE(baudrate)                                                                     \
	((baudrate) == 300       ? 0x00014000                                                      \
	 : (baudrate) == 600     ? 0x00027000                                                      \
	 : (baudrate) == 1200    ? NRF_UARTE_BAUDRATE_1200                                         \
	 : (baudrate) == 2400    ? NRF_UARTE_BAUDRATE_2400                                         \
	 : (baudrate) == 4800    ? NRF_UARTE_BAUDRATE_4800                                         \
	 : (baudrate) == 9600    ? NRF_UARTE_BAUDRATE_9600                                         \
	 : (baudrate) == 14400   ? NRF_UARTE_BAUDRATE_14400                                        \
	 : (baudrate) == 19200   ? NRF_UARTE_BAUDRATE_19200                                        \
	 : (baudrate) == 28800   ? NRF_UARTE_BAUDRATE_28800                                        \
	 : (baudrate) == 31250   ? NRF_UARTE_BAUDRATE_31250                                        \
	 : (baudrate) == 38400   ? NRF_UARTE_BAUDRATE_38400                                        \
	 : (baudrate) == 56000   ? NRF_UARTE_BAUDRATE_56000                                        \
	 : (baudrate) == 57600   ? NRF_UARTE_BAUDRATE_57600                                        \
	 : (baudrate) == 76800   ? NRF_UARTE_BAUDRATE_76800                                        \
	 : (baudrate) == 115200  ? NRF_UARTE_BAUDRATE_115200                                       \
	 : (baudrate) == 230400  ? NRF_UARTE_BAUDRATE_230400                                       \
	 : (baudrate) == 250000  ? NRF_UARTE_BAUDRATE_250000                                       \
	 : (baudrate) == 460800  ? NRF_UARTE_BAUDRATE_460800                                       \
	 : (baudrate) == 921600  ? NRF_UARTE_BAUDRATE_921600                                       \
	 : (baudrate) == 1000000 ? NRF_UARTE_BAUDRATE_1000000                                      \
				 : 0)

/**
 * @brief Structure for UARTE configuration.
 */
struct uarte_nrfx_config {
	NRF_UARTE_Type *uarte_regs; /* Instance address */
	uint32_t flags;
	bool disable_rx;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	/* None-zero in case of high speed instances. Baudrate is adjusted by that
	 * ratio. */
	uint32_t clock_freq;
#else
#ifdef UARTE_HAS_FRAME_TIMEOUT
	uint32_t baudrate;
#endif
	nrf_uarte_baudrate_t nrf_baudrate;
	nrf_uarte_config_t hw_config;
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

	uint8_t *poll_out_byte;
	uint8_t *poll_in_byte;
	uint32_t switching_delay_us;
};

static inline NRF_UARTE_Type *get_uarte_instance(const struct device *dev)
{
	const struct uarte_nrfx_config *config = dev->config;

	return config->uarte_regs;
}

static void endtx_isr(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	unsigned int key = irq_lock();

	if (nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDTX)) {
		nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDTX);
		nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STOPTX);
	}

	irq_unlock(key);
}

static int switch_to_tx_mode(const struct device *dev)
{
	struct uarte_nrfx_data *data = dev->data;
	const struct uarte_nrfx_config *config = dev->config;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	if (data->mode == UARTE_MODE_TX) {
		return 0;
	}
	if (data->mode == UARTE_MODE_IDLE) {
		LOG_WRN("Cannot switch from IDLE to TX mode");
		return -ECANCELED;
	}
	nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STOPRX);
	// LOG_WRN("Switching to TX mode");
	nrf_uarte_disable(uarte);
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_TX);
	if (ret < 0) {
		LOG_WRN("Failed to apply PINCTRL_STATE_TX");
		return ret;
	}
	data->mode = UARTE_MODE_TX;

	nrf_uarte_enable(uarte);
	return 0;
}

static int switch_to_rx_mode(const struct device *dev)
{
	struct uarte_nrfx_data *data = dev->data;
	const struct uarte_nrfx_config *config = dev->config;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	if (data->mode == UARTE_MODE_RX) {
		return 0;
	}
	if (data->mode == UARTE_MODE_IDLE) {
		LOG_WRN("Cannot switch from IDLE to RX mode");
		return -ECANCELED;
	}
	// LOG_WRN("Switching to RX mode");
	nrf_uarte_disable(uarte);
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_RX);
	if (ret < 0) {
		LOG_WRN("Failed to apply PINCTRL_STATE_RX");
		return ret;
	}
	data->mode = UARTE_MODE_RX;

	nrf_uarte_enable(uarte);
	if (!config->disable_rx) {
		nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);
		nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTRX);
	}
	return 0;
}

#ifdef UARTE_ANY_NONE_ASYNC
/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
static void uarte_nrfx_isr_int(const void *arg)
{
	const struct device *dev = arg;
	struct uarte_nrfx_data *data = dev->data;
	const struct uarte_nrfx_config *config = dev->config;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	/* If interrupt driven and asynchronous APIs are disabled then UART
	 * interrupt is still called to stop TX. Unless it is done using PPI.
	 */
	if (!IS_ENABLED(UARTE_HAS_ENDTX_STOPTX_SHORT) &&
	    nrf_uarte_int_enable_check(uarte, NRF_UARTE_INT_ENDTX_MASK) &&
	    nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDTX)) {
		endtx_isr(dev);
	}

#ifdef UARTE_INTERRUPT_DRIVEN

	if (!data->int_driven) {
		return;
	}

	bool switch_to_rx = false;
	if (nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_TXSTOPPED)) {
		switch_to_rx = true;
		data->int_driven->fifo_fill_lock = 0; // accepts new transfer
		if (data->int_driven->disable_tx_irq) {
			nrf_uarte_int_disable(uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
			data->int_driven->disable_tx_irq = false;
			switch_to_rx_mode(dev);
			return;
		}
	}

	if (nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ERROR)) {
		switch_to_rx = true;
		// LOG_ERR("UARTE error occurred (on tx end %d)", switch_to_rx ? 1 : 0);
		// switch_to_rx_mode(dev); // TODO: it's just in case of TX error
		nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ERROR);
	}

	if (!data->int_driven->rx_irq_processed &&
	    nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDRX)) {
		data->int_driven->rx_irq_processed = true;
		data->last_rx_end_time_ticks = k_cycle_get_32();
		k_work_reschedule(&data->tx_ready_work, K_USEC(config->switching_delay_us));
	}

	if (data->int_driven->cb) {
		data->int_driven->cb(dev, data->int_driven->cb_data);
	}
	if (switch_to_rx) {
		switch_to_rx_mode(dev);
	}
#endif /* UARTE_INTERRUPT_DRIVEN */
}
#endif /* UARTE_ANY_NONE_ASYNC */

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
/**
 * @brief Set the baud rate
 *
 * This routine set the given baud rate for the UARTE.
 *
 * @param dev UARTE device struct
 * @param baudrate Baud rate
 *
 * @return 0 on success or error code
 */
static int baudrate_set(const struct device *dev, uint32_t baudrate)
{
	const struct uarte_nrfx_config *config = dev->config;
	/* calculated baudrate divisor */
	nrf_uarte_baudrate_t nrf_baudrate = NRF_BAUDRATE(baudrate);
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	if (nrf_baudrate == 0) {
		return -EINVAL;
	}

	/* scale baudrate setting */
	if (config->clock_freq > 0U) {
		nrf_baudrate /= config->clock_freq / NRF_TIMER_BASE_FREQUENCY_16MHZ;
	}

	nrf_uarte_baudrate_set(uarte, nrf_baudrate);

	return 0;
}

static int uarte_nrfx_configure(const struct device *dev, const struct uart_config *cfg)
{
	struct uarte_nrfx_data *data = dev->data;
	nrf_uarte_config_t uarte_cfg;

#if defined(UARTE_CONFIG_STOP_Msk)
	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		uarte_cfg.stop = NRF_UARTE_STOP_ONE;
		break;
	case UART_CFG_STOP_BITS_2:
		uarte_cfg.stop = NRF_UARTE_STOP_TWO;
		break;
	default:
		return -ENOTSUP;
	}
#else
	if (cfg->stop_bits != UART_CFG_STOP_BITS_1) {
		return -ENOTSUP;
	}
#endif

	if (cfg->data_bits != UART_CFG_DATA_BITS_8) {
		return -ENOTSUP;
	}

	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		uarte_cfg.hwfc = NRF_UARTE_HWFC_DISABLED;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		uarte_cfg.hwfc = NRF_UARTE_HWFC_ENABLED;
		break;
	default:
		return -ENOTSUP;
	}

#if defined(UARTE_CONFIG_PARITYTYPE_Msk)
	uarte_cfg.paritytype = NRF_UARTE_PARITYTYPE_EVEN;
#endif
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		uarte_cfg.parity = NRF_UARTE_PARITY_EXCLUDED;
		break;
	case UART_CFG_PARITY_EVEN:
		uarte_cfg.parity = NRF_UARTE_PARITY_INCLUDED;
		break;
#if defined(UARTE_CONFIG_PARITYTYPE_Msk)
	case UART_CFG_PARITY_ODD:
		uarte_cfg.parity = NRF_UARTE_PARITY_INCLUDED;
		uarte_cfg.paritytype = NRF_UARTE_PARITYTYPE_ODD;
		break;
#endif
	default:
		return -ENOTSUP;
	}

	if (baudrate_set(dev, cfg->baudrate) != 0) {
		return -ENOTSUP;
	}

#ifdef UARTE_HAS_FRAME_TIMEOUT
	uarte_cfg.frame_timeout = NRF_UARTE_FRAME_TIMEOUT_EN;
#endif
	nrf_uarte_configure(get_uarte_instance(dev), &uarte_cfg);

	data->uart_config = *cfg;

	return 0;
}

static int uarte_nrfx_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uarte_nrfx_data *data = dev->data;

	*cfg = data->uart_config;
	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uarte_nrfx_err_check(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	/* register bitfields maps to the defines in uart.h */
	return nrf_uarte_errorsrc_get_and_clear(uarte);
}

/* Function returns true if new transfer can be started. Since TXSTOPPED
 * (and ENDTX) is cleared before triggering new transfer, TX is ready for new
 * transfer if any event is set.
 */
static bool is_tx_ready(const struct device *dev)
{
	const struct uarte_nrfx_data *data = dev->data;
	const struct uarte_nrfx_config *config = dev->config;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	bool ppi_endtx = config->flags & UARTE_CFG_FLAG_PPI_ENDTX ||
			 IS_ENABLED(UARTE_HAS_ENDTX_STOPTX_SHORT);

	const uint64_t duration = k_cycle_get_32() - data->last_rx_end_time_ticks;
	const uint64_t elapsed_us = duration * USEC_PER_SEC / sys_clock_hw_cycles_per_sec();
	if (elapsed_us < config->switching_delay_us) {
		LOG_WRN("Switching delay not elapsed yet (remaining %llu us)",
			config->switching_delay_us - elapsed_us);
		return false;
	}

	return nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_TXSTOPPED) ||
	       (!ppi_endtx ? nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDTX) : 0);
}

/* Wait until the transmitter is in the idle state. When this function returns,
 * IRQ's are locked with the returned key.
 */
static int wait_tx_ready(const struct device *dev)
{
	unsigned int key;

	do {
		/* wait arbitrary time before back off. */
		bool res;

#if defined(CONFIG_ARCH_POSIX)
		NRFX_WAIT_FOR(is_tx_ready(dev), 33, 3, res);
#else
		NRFX_WAIT_FOR(is_tx_ready(dev), 100, 1, res);
#endif

		if (res) {
			key = irq_lock();
			if (is_tx_ready(dev)) {
				break;
			}

			irq_unlock(key);
		}
		if (IS_ENABLED(CONFIG_MULTITHREADING)) {
			k_msleep(1);
		}
	} while (1);

	return key;
}

/* At this point we should have irq locked and any previous transfer completed.
 * Transfer can be started, no need to wait for completion.
 */
static void tx_start(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct uarte_nrfx_config *config = dev->config;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return;
	}
#endif
	if (len > 0 && switch_to_tx_mode(dev) != 0) {
		LOG_WRN("Failed to switch to TX mode");
		return;
	}

	nrf_uarte_tx_buffer_set(uarte, buf, len);
	nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDTX);
	nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_TXSTOPPED);

	nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTTX);
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UARTE device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer is empty.
 */
static int uarte_nrfx_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uarte_nrfx_config *config = dev->config;
	struct uarte_nrfx_data *data = dev->data;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	if (!nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDRX)) {
		return -1;
	}

#ifdef UARTE_INTERRUPT_DRIVEN
	if (!data->int_driven->rx_irq_enabled) {
		data->last_rx_end_time_ticks = k_cycle_get_32();
	}
#endif

	*c = *config->poll_in_byte;

	/* clear the interrupt */

	nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);
	nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTRX);
#ifdef UARTE_INTERRUPT_DRIVEN
	data->int_driven->rx_irq_processed = false;
#endif

	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * @param dev UARTE device struct
 * @param c Character to send
 */
static void uarte_nrfx_poll_out(const struct device *dev, unsigned char c)
{
	const struct uarte_nrfx_config *config = dev->config;
	bool isr_mode = k_is_in_isr() || k_is_pre_kernel();
	unsigned int key;

	if (isr_mode) {
		while (1) {
			key = irq_lock();
			if (is_tx_ready(dev)) {
				break;
			}

			irq_unlock(key);
			Z_SPIN_DELAY(3);
		}
	} else {
		key = wait_tx_ready(dev);
	}
	// TODO: wait until tx ready and change to TX mode
	*config->poll_out_byte = c;
	tx_start(dev, config->poll_out_byte, 1);

	irq_unlock(key);
}

#ifdef UARTE_INTERRUPT_DRIVEN

static void uarte_tx_ready_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uarte_nrfx_data *data = CONTAINER_OF(dwork, struct uarte_nrfx_data, tx_ready_work);

	if (data->int_driven->cb) {
		data->int_driven->cb(data->dev, data->int_driven->cb_data);
	}
}

/** Interrupt driven FIFO fill function */
static int uarte_nrfx_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	struct uarte_nrfx_data *data = dev->data;

	len = MIN(len, data->int_driven->tx_buff_size);
	if (!atomic_cas(&data->int_driven->fifo_fill_lock, 0, 1)) {
		return 0;
	}

	/* Copy data to RAM buffer for EasyDMA transfer */
	memcpy(data->int_driven->tx_buffer, tx_data, len);

	unsigned int key = irq_lock();

	if (!is_tx_ready(dev)) {
		data->int_driven->fifo_fill_lock = 0;
		len = 0;
	} else {
		tx_start(dev, data->int_driven->tx_buffer, len);
	}

	irq_unlock(key);

	return len;
}

/** Interrupt driven FIFO read function */
static int uarte_nrfx_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	int num_rx = 0;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	const struct uarte_nrfx_config *config = dev->config;
	struct uarte_nrfx_data *data = dev->data;

	if (size > 0 && nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDRX)) {
		/* Clear the interrupt */
		data->int_driven->rx_irq_processed = false;
		nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);

		/* Receive a character */
		rx_data[num_rx++] = *config->poll_in_byte;

		nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTRX);
	}

	return num_rx;
}

/** Interrupt driven transfer enabling function */
static void uarte_nrfx_irq_tx_enable(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	struct uarte_nrfx_data *data = dev->data;
	const struct uarte_nrfx_config *cfg = dev->config;
	unsigned int key = irq_lock();

	data->int_driven->disable_tx_irq = false;
	data->int_driven->tx_irq_enabled = true;
	nrf_uarte_int_enable(uarte, NRF_UARTE_INT_TXSTOPPED_MASK);

	// fake event to set TXSTOPPED
	tx_start(dev, cfg->poll_out_byte, 0);

	irq_unlock(key);
}

/** Interrupt driven transfer disabling function */
static void uarte_nrfx_irq_tx_disable(const struct device *dev)
{
	struct uarte_nrfx_data *data = dev->data;
	/* TX IRQ will be disabled after current transmission is finished */
	data->int_driven->disable_tx_irq = true;
	data->int_driven->tx_irq_enabled = false;
}

/** Interrupt driven transfer ready function */
static int uarte_nrfx_irq_tx_ready_complete(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	struct uarte_nrfx_data *data = dev->data;

	/* ENDTX flag is always on so that ISR is called when we enable TX IRQ.
	 * Because of that we have to explicitly check if ENDTX interrupt is
	 * enabled, otherwise this function would always return true no matter
	 * what would be the source of interrupt.
	 */
	bool ready = data->int_driven->tx_irq_enabled &&
		     nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_TXSTOPPED);

	if (ready) {
		data->int_driven->fifo_fill_lock = 0;
	}

	return ready ? data->int_driven->tx_buff_size : 0;
}

static int uarte_nrfx_irq_rx_ready(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	return nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_ENDRX);
}

/** Interrupt driven receiver enabling function */
static void uarte_nrfx_irq_rx_enable(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	nrf_uarte_int_enable(uarte, NRF_UARTE_INT_ENDRX_MASK);
}

/** Interrupt driven receiver disabling function */
static void uarte_nrfx_irq_rx_disable(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	nrf_uarte_int_disable(uarte, NRF_UARTE_INT_ENDRX_MASK);
}

/** Interrupt driven error enabling function */
static void uarte_nrfx_irq_err_enable(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	nrf_uarte_int_enable(uarte, NRF_UARTE_INT_ERROR_MASK);
}

/** Interrupt driven error disabling function */
static void uarte_nrfx_irq_err_disable(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	nrf_uarte_int_disable(uarte, NRF_UARTE_INT_ERROR_MASK);
}

/** Interrupt driven pending status function */
static int uarte_nrfx_irq_is_pending(const struct device *dev)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);

	return ((nrf_uarte_int_enable_check(uarte, NRF_UARTE_INT_TXSTOPPED_MASK) &&
		 uarte_nrfx_irq_tx_ready_complete(dev)) ||
		(nrf_uarte_int_enable_check(uarte, NRF_UARTE_INT_ENDRX_MASK) &&
		 uarte_nrfx_irq_rx_ready(dev)));
}

/** Interrupt driven interrupt update function */
static int uarte_nrfx_irq_update(const struct device *dev)
{
	return 1;
}

/** Set the callback function */
static void uarte_nrfx_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uarte_nrfx_data *data = dev->data;

	data->int_driven->cb = cb;
	data->int_driven->cb_data = cb_data;
}
#endif /* UARTE_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_nrfx_uarte_driver_api = {
	.poll_in = uarte_nrfx_poll_in,
	.poll_out = uarte_nrfx_poll_out,
	.err_check = uarte_nrfx_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uarte_nrfx_configure,
	.config_get = uarte_nrfx_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef UARTE_INTERRUPT_DRIVEN
	.fifo_fill = uarte_nrfx_fifo_fill,
	.fifo_read = uarte_nrfx_fifo_read,
	.irq_tx_enable = uarte_nrfx_irq_tx_enable,
	.irq_tx_disable = uarte_nrfx_irq_tx_disable,
	.irq_tx_ready = uarte_nrfx_irq_tx_ready_complete,
	.irq_rx_enable = uarte_nrfx_irq_rx_enable,
	.irq_rx_disable = uarte_nrfx_irq_rx_disable,
	.irq_tx_complete = uarte_nrfx_irq_tx_ready_complete,
	.irq_rx_ready = uarte_nrfx_irq_rx_ready,
	.irq_err_enable = uarte_nrfx_irq_err_enable,
	.irq_err_disable = uarte_nrfx_irq_err_disable,
	.irq_is_pending = uarte_nrfx_irq_is_pending,
	.irq_update = uarte_nrfx_irq_update,
	.irq_callback_set = uarte_nrfx_irq_callback_set,
#endif /* UARTE_INTERRUPT_DRIVEN */
};

#ifdef UARTE_ENHANCED_POLL_OUT
static int endtx_stoptx_ppi_init(NRF_UARTE_Type *uarte, struct uarte_nrfx_data *data)
{
	nrfx_err_t ret;

	ret = nrfx_gppi_channel_alloc(&data->ppi_ch_endtx);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("Failed to allocate PPI Channel");
		return -EIO;
	}

	nrfx_gppi_channel_endpoints_setup(data->ppi_ch_endtx,
					  nrf_uarte_event_address_get(uarte, NRF_UARTE_EVENT_ENDTX),
					  nrf_uarte_task_address_get(uarte, NRF_UARTE_TASK_STOPTX));
	nrfx_gppi_channels_enable(BIT(data->ppi_ch_endtx));

	return 0;
}
#endif /* UARTE_ENHANCED_POLL_OUT */

static int uarte_instance_init(const struct device *dev, uint8_t interrupts_active)
{
	int err;
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	const struct uarte_nrfx_config *cfg = dev->config;
	struct uarte_nrfx_data *data = dev->data;

	nrf_uarte_disable(uarte);

#ifdef UARTE_INTERRUPT_DRIVEN
	k_work_init_delayable(&data->tx_ready_work, uarte_tx_ready_work_handler);
#endif

#ifdef CONFIG_ARCH_POSIX
	/* For simulation the DT provided peripheral address needs to be corrected
	 */
	((struct pinctrl_dev_config *)cfg->pcfg)->reg = (uintptr_t)cfg->uarte_regs;
#endif

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_RX);
	if (err < 0) {
		LOG_ERR("Failed to apply PINCTRL_STATE_RX for %s", dev->name);
		return err;
	}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	err = uarte_nrfx_configure(dev, &data->uart_config);
	if (err) {
		LOG_ERR("Failed to configure UARTE for %s", dev->name);
		return err;
	}
#else
	nrf_uarte_baudrate_set(uarte, cfg->nrf_baudrate);
	nrf_uarte_configure(uarte, &cfg->hw_config);
#endif

#ifdef UARTE_HAS_ENDTX_STOPTX_SHORT
	nrf_uarte_shorts_enable(uarte, NRF_UARTE_SHORT_ENDTX_STOPTX);
#elif defined(UARTE_ENHANCED_POLL_OUT)
	if (cfg->flags & UARTE_CFG_FLAG_PPI_ENDTX) {
		err = endtx_stoptx_ppi_init(uarte, data);
		if (err < 0) {
			LOG_ERR("Failed to initialize ENDTX->STOPTX PPI for %s", dev->name);
			return err;
		}
	}
#endif
	{
		// Enable as RX mode
		data->mode = UARTE_MODE_RX;
		nrf_uarte_enable(uarte);

		if (!cfg->disable_rx) {
#ifdef UARTE_INTERRUPT_DRIVEN
			data->int_driven->rx_irq_processed = false;
#endif
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);

			nrf_uarte_rx_buffer_set(uarte, cfg->poll_in_byte, 1);
			nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTRX);
		}
	}

	if (!IS_ENABLED(UARTE_HAS_ENDTX_STOPTX_SHORT) && !(cfg->flags & UARTE_CFG_FLAG_PPI_ENDTX)) {
		nrf_uarte_int_enable(uarte, NRF_UARTE_INT_ENDTX_MASK);
	}

	/* Set TXSTOPPED event by requesting fake (zero-length) transfer.
	 * Pointer to RAM variable (data->tx_buffer) is set because otherwise
	 * such operation may result in HardFault or RAM corruption.
	 */
	tx_start(dev, cfg->poll_out_byte, 0);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
/** @brief Pend until TX is stopped.
 *
 * There are 2 configurations that must be handled:
 * - ENDTX->TXSTOPPED PPI enabled - just pend until TXSTOPPED event is set
 * - disable ENDTX interrupt and manually trigger STOPTX, pend for TXSTOPPED
 */
static void wait_for_tx_stopped(const struct device *dev)
{
	const struct uarte_nrfx_config *config = dev->config;
	bool ppi_endtx = (config->flags & UARTE_CFG_FLAG_PPI_ENDTX) ||
			 IS_ENABLED(UARTE_HAS_ENDTX_STOPTX_SHORT);
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
	bool res;

	if (!ppi_endtx) {
		/* We assume here that it can be called from any context,
		 * including the one that uarte interrupt will not preempt.
		 * Disable endtx interrupt to ensure that it will not be triggered
		 * (if in lower priority context) and stop TX if necessary.
		 */
		nrf_uarte_int_disable(uarte, NRF_UARTE_INT_ENDTX_MASK);
		NRFX_WAIT_FOR(is_tx_ready(dev), 1000, 1, res);
		if (!nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_TXSTOPPED)) {
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDTX);
			nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STOPTX);
		}
	}

	NRFX_WAIT_FOR(nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_TXSTOPPED), 1000, 1, res);

	if (!ppi_endtx) {
		nrf_uarte_int_enable(uarte, NRF_UARTE_INT_ENDTX_MASK);
	}
}

static int uarte_nrfx_pm_action(const struct device *dev, enum pm_device_action action)
{
	NRF_UARTE_Type *uarte = get_uarte_instance(dev);
#if defined(UARTE_INTERRUPT_DRIVEN)
	struct uarte_nrfx_data *data = dev->data;
#endif
	const struct uarte_nrfx_config *cfg = dev->config;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
		data->mode = UARTE_MODE_RX;

		nrf_uarte_enable(uarte);

		if (!cfg->disable_rx) {
#ifdef UARTE_INTERRUPT_DRIVEN
			data->int_driven->rx_irq_processed = false;
#endif
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);
			nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STARTRX);
#ifdef UARTE_INTERRUPT_DRIVEN
			if (data->int_driven && data->int_driven->rx_irq_enabled) {
				nrf_uarte_int_enable(uarte, NRF_UARTE_INT_ENDRX_MASK);
			}
#endif
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Disabling UART requires stopping RX, but stop RX event is
		 * only sent after each RX if async UART API is used.
		 */
		if (nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_RXSTARTED)) {
#ifdef UARTE_INTERRUPT_DRIVEN
			if (data->int_driven) {
				data->int_driven->rx_irq_enabled =
					nrf_uarte_int_enable_check(uarte, NRF_UARTE_INT_ENDRX_MASK);
				if (data->int_driven->rx_irq_enabled) {
					nrf_uarte_int_disable(uarte, NRF_UARTE_INT_ENDRX_MASK);
				}
			}
#endif
			nrf_uarte_task_trigger(uarte, NRF_UARTE_TASK_STOPRX);
			while (!nrf_uarte_event_check(uarte, NRF_UARTE_EVENT_RXTO)) {
				/* Busy wait for event to register */
				Z_SPIN_DELAY(2);
			}
#ifdef UARTE_INTERRUPT_DRIVEN
			data->int_driven->rx_irq_processed = false;
#endif
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_RXSTARTED);
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_RXTO);
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ENDRX);
			nrf_uarte_event_clear(uarte, NRF_UARTE_EVENT_ERROR);
		}

		wait_for_tx_stopped(dev);
		nrf_uarte_disable(get_uarte_instance(dev));
		data->mode = UARTE_MODE_IDLE;
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if (ret < 0) {
			return ret;
		}

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

#define UARTE_IRQ_CONFIGURE(idx, isr_handler)                                                      \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQN(UARTE(idx)), DT_IRQ(UARTE(idx), priority), isr_handler,        \
			    DEVICE_DT_GET(UARTE(idx)), 0);                                         \
		irq_enable(DT_IRQN(UARTE(idx)));                                                   \
	} while (false)

#define UARTE_DISABLE_RX_INIT(node_id) .disable_rx = DT_PROP(node_id, disable_rx)

#define UARTE_GET_FREQ(idx) DT_PROP(DT_CLOCKS_CTLR(UARTE(idx)), clock_frequency)

#define UARTE_GET_BAUDRATE_DIV(idx)                                                                \
	COND_CODE_1(DT_CLOCKS_HAS_IDX(UARTE(idx), 0),                                              \
		    ((UARTE_GET_FREQ(idx) / NRF_UARTE_BASE_FREQUENCY_16MHZ)), (1))

/* When calculating baudrate we need to take into account that high speed
 * instances must have baudrate adjust to the ratio between UARTE clocking
 * frequency and 16 MHz.
 */
#define UARTE_GET_BAUDRATE(idx)                                                                    \
	(NRF_BAUDRATE(UARTE_PROP(idx, current_speed)) / UARTE_GET_BAUDRATE_DIV(idx))

/* Macro for setting nRF specific configuration structures. */
#define UARTE_NRF_CONFIG(idx)                                                                      \
	{.hwfc = (UARTE_PROP(idx, hw_flow_control) == UART_CFG_FLOW_CTRL_RTS_CTS)                  \
			 ? NRF_UARTE_HWFC_ENABLED                                                  \
			 : NRF_UARTE_HWFC_DISABLED,                                                \
	 .parity = IS_ENABLED(CONFIG_UART_##idx##_NRF_PARITY_BIT) ? NRF_UARTE_PARITY_INCLUDED      \
								  : NRF_UARTE_PARITY_EXCLUDED,     \
	 IF_ENABLED(UARTE_HAS_STOP_CONFIG, (.stop = NRF_UARTE_STOP_ONE, ))                         \
		 IF_ENABLED(UARTE_ODD_PARITY_ALLOWED, (.paritytype = NRF_UARTE_PARITYTYPE_EVEN, )) \
			 IF_ENABLED(UARTE_HAS_FRAME_TIMEOUT,                                       \
				    (.frame_timeout = NRF_UARTE_FRAME_TIMEOUT_EN, ))}

/* Macro for setting zephyr specific configuration structures. */
#define UARTE_CONFIG(idx)                                                                          \
	{                                                                                          \
		.baudrate = UARTE_PROP(idx, current_speed),                                        \
		.data_bits = UART_CFG_DATA_BITS_8,                                                 \
		.stop_bits = UART_CFG_STOP_BITS_1,                                                 \
		.parity = IS_ENABLED(CONFIG_UART_##idx##_NRF_PARITY_BIT) ? UART_CFG_PARITY_EVEN    \
									 : UART_CFG_PARITY_NONE,   \
		.flow_ctrl = UARTE_PROP(idx, hw_flow_control) ? UART_CFG_FLOW_CTRL_RTS_CTS         \
							      : UART_CFG_FLOW_CTRL_NONE,           \
	}

#define UART_NRF_UARTE_DEVICE(idx)                                                                 \
	NRF_DT_CHECK_NODE_HAS_PINCTRL_SLEEP(UARTE(idx));                                           \
	UARTE_INT_DRIVEN(idx);                                                                     \
	PINCTRL_DT_DEFINE(UARTE(idx));                                                             \
	IF_ENABLED(                                                                                \
		CONFIG_UART_##idx##_ASYNC,                                                         \
		(static uint8_t uarte##idx##_tx_cache                                              \
			 [CONFIG_UART_ASYNC_TX_CACHE_SIZE] UARTE_MEMORY_SECTION(idx);              \
		 static uint8_t                                                                    \
			 uarte##idx##_flush_buf[UARTE_HW_RX_FIFO_SIZE] UARTE_MEMORY_SECTION(idx);  \
		 struct uarte_async_cb uarte##idx##_async;))                                       \
	static uint8_t uarte##idx##_poll_out_byte UARTE_MEMORY_SECTION(idx);                       \
	static uint8_t uarte##idx##_poll_in_byte UARTE_MEMORY_SECTION(idx);                        \
	static struct uarte_nrfx_data uarte_##idx##_data = {                                       \
		IF_ENABLED(CONFIG_UART_USE_RUNTIME_CONFIGURE,                                      \
			   (.uart_config = UARTE_CONFIG(idx), ))                                   \
			IF_ENABLED(CONFIG_UART_##idx##_INTERRUPT_DRIVEN,                           \
				   (.int_driven = &uarte##idx##_int_driven, ))                     \
				.mode = UARTE_MODE_IDLE,                                           \
		IF_ENABLED(INTERRUUPT_DRIVEN, (.dev = DEVICE_DT_GET(UARTE(idx)), ))};              \
	COND_CODE_1(CONFIG_UART_USE_RUNTIME_CONFIGURE, (),                                         \
		    (BUILD_ASSERT(NRF_BAUDRATE(UARTE_PROP(idx, current_speed)) > 0,                \
				  "Unsupported baudrate");))                                       \
	static const struct uarte_nrfx_config uarte_##idx##z_config = {                            \
		COND_CODE_1(CONFIG_UART_USE_RUNTIME_CONFIGURE,                                     \
			    (IF_ENABLED(DT_CLOCKS_HAS_IDX(UARTE(idx), 0),                          \
					(.clock_freq = UARTE_GET_FREQ(idx), ))),                   \
			    (IF_ENABLED(UARTE_HAS_FRAME_TIMEOUT,                                   \
					(.baudrate = UARTE_PROP(idx, current_speed), ))            \
				     .nrf_baudrate = UARTE_GET_BAUDRATE(idx),                      \
			     .hw_config = UARTE_NRF_CONFIG(idx), ))                                \
			.pcfg = PINCTRL_DT_DEV_CONFIG_GET(UARTE(idx)),                             \
		.uarte_regs = _CONCAT(NRF_UARTE, idx),                                             \
		.flags = (IS_ENABLED(CONFIG_UART_##idx##_ENHANCED_POLL_OUT)                        \
				  ? UARTE_CFG_FLAG_PPI_ENDTX                                       \
				  : 0) |                                                           \
			 (IS_ENABLED(CONFIG_UART_##idx##_NRF_HW_ASYNC)                             \
				  ? UARTE_CFG_FLAG_HW_BYTE_COUNTING                                \
				  : 0),                                                            \
		UARTE_DISABLE_RX_INIT(UARTE(idx)),                                                 \
		.poll_out_byte = &uarte##idx##_poll_out_byte,                                      \
		.poll_in_byte = &uarte##idx##_poll_in_byte,                                        \
		.switching_delay_us = UARTE_PROP(idx, switching_delay_us),                         \
		IF_ENABLED(CONFIG_UART_##idx##_ASYNC, (.tx_cache = uarte##idx##_tx_cache,          \
						       .rx_flush_buf = uarte##idx##_flush_buf, ))  \
			IF_ENABLED(CONFIG_UART_##idx##_NRF_HW_ASYNC,                               \
				   (.timer = NRFX_TIMER_INSTANCE(                                  \
					    CONFIG_UART_##idx##_NRF_HW_ASYNC_TIMER), ))};          \
	static int uarte_##idx##_init(const struct device *dev)                                    \
	{                                                                                          \
		COND_CODE_1(CONFIG_UART_##idx##_ASYNC,                                             \
			    (UARTE_IRQ_CONFIGURE(idx, uarte_nrfx_isr_async);),                     \
			    (UARTE_IRQ_CONFIGURE(idx, uarte_nrfx_isr_int);))                       \
		return uarte_instance_init(dev, IS_ENABLED(CONFIG_UART_##idx##_INTERRUPT_DRIVEN)); \
	}                                                                                          \
                                                                                                   \
	PM_DEVICE_DT_DEFINE(UARTE(idx), uarte_nrfx_pm_action);                                     \
                                                                                                   \
	DEVICE_DT_DEFINE(UARTE(idx), uarte_##idx##_init, PM_DEVICE_DT_GET(UARTE(idx)),             \
			 &uarte_##idx##_data, &uarte_##idx##z_config, PRE_KERNEL_1,                \
			 CONFIG_SERIAL_INIT_PRIORITY, &uart_nrfx_uarte_driver_api)

#define UARTE_INT_DRIVEN(idx)                                                                      \
	IF_ENABLED(CONFIG_UART_##idx##_INTERRUPT_DRIVEN,                                           \
		   (static uint8_t uarte##idx##_tx_buffer[MIN(                                     \
			   CONFIG_UART_##idx##_NRF_TX_BUFFER_SIZE,                                 \
			   BIT_MASK(UARTE##idx##_EASYDMA_MAXCNT_SIZE))] UARTE_MEMORY_SECTION(idx); \
		    static struct uarte_nrfx_int_driven uarte##idx##_int_driven = {                \
			    .tx_buffer = uarte##idx##_tx_buffer,                                   \
			    .tx_buff_size = sizeof(uarte##idx##_tx_buffer),                        \
		    };))

#define UARTE_MEMORY_SECTION(idx)                                                                  \
	COND_CODE_1(UARTE_HAS_PROP(idx, memory_regions),                                           \
		    (__attribute__((__section__(LINKER_DT_NODE_REGION_NAME(                        \
			    DT_PHANDLE(UARTE(idx), memory_regions)))))),                           \
		    ())

#define COND_UART_NRF_UARTE_DEVICE(unused, prefix, i, _)                                           \
	IF_ENABLED(CONFIG_HAS_HW_NRF_UARTE_HALF_DUPLEX##prefix##i,                                 \
		   (UART_NRF_UARTE_DEVICE(prefix##i);))

UARTE_FOR_EACH_INSTANCE(COND_UART_NRF_UARTE_DEVICE, (), ())
